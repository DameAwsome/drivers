//! `:input`
//!
//! A seperate scheme is required since all of the input from different input devices is required
//! to be combined into a single stream which is later going to be processed by the "consumer"
//! which usually is Orbital.
//!
//! ## Input Device ("producer")
//! Write events to `input:producer`.
//!
//! ## Input Consumer ("consumer")
//! Read events from `input:consumer`. Optionally, set the `EVENT_READ` flag to be notified when
//! events are available.

use std::collections::BTreeMap;
use std::fs::File;
use std::io::{self, Read, Write};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Arc;

use inputd::{Command, CommandTy};
use orbclient::{Event, EventOption};
use syscall::{Error as SysError, EventFlags, Packet, SchemeMut, EINVAL};

enum Handle {
    Producer,
    Consumer {
        events: EventFlags,
        pending: Vec<u8>,
        notified: bool,
        vt: usize,
    },
    Device(Arc<String>),
}

impl Handle {
    pub fn is_producer(&self) -> bool {
        matches!(self, Handle::Producer)
    }
}

struct InputScheme {
    handles: BTreeMap<usize, Handle>,

    next_id: AtomicUsize,
    next_vt_id: AtomicUsize,

    vts: BTreeMap<usize, Arc<String>>,
    super_key: bool,
    active_vt: Option<(Arc<String>, File, usize /* VT index */)>,
}

impl InputScheme {
    pub fn new() -> Self {
        Self {
            next_id: AtomicUsize::new(0),
            next_vt_id: AtomicUsize::new(1),

            handles: BTreeMap::new(),
            vts: BTreeMap::new(),
            super_key: false,
            active_vt: None,
        }
    }
}

impl SchemeMut for InputScheme {
    fn open(&mut self, path: &str, _flags: usize, _uid: u32, _gid: u32) -> syscall::Result<usize> {
        let mut path_parts = path.split('/');

        let command = path_parts.next().ok_or(SysError::new(EINVAL))?;
        let fd = self.next_id.fetch_add(1, Ordering::SeqCst);

        let handle_ty = match command {
            "producer" => Handle::Producer,
            "consumer" => {
                let vt = if let Some((_, _, vt)) = self.active_vt {
                    vt
                } else {
                    // No one is currently set as active, so we assume that the last allocated
                    // one will be willing to take this consumer.
                    self.next_vt_id.load(Ordering::SeqCst) - 1
                };

                dbg!(vt);
                Handle::Consumer {
                    events: EventFlags::empty(),
                    pending: Vec::new(),
                    notified: false,
                    vt,
                }
            }
            "handle" => {
                let value = path_parts.collect::<Vec<_>>().join("/");
                Handle::Device(Arc::new(value))
            }

            _ => unreachable!("inputd: invalid path {path}"),
        };

        log::info!("inputd: {path} channel has been opened");

        self.handles.insert(fd, handle_ty);
        Ok(fd)
    }

    fn read(&mut self, id: usize, buf: &mut [u8]) -> syscall::Result<usize> {
        let handle = self.handles.get_mut(&id).ok_or(SysError::new(EINVAL))?;

        match handle {
            Handle::Consumer { pending, .. } => {
                let copy = core::cmp::min(pending.len(), buf.len());

                for (i, byte) in pending.drain(..copy).enumerate() {
                    buf[i] = byte;
                }

                Ok(copy)
            }

            Handle::Device(device) => {
                let vt = self.next_vt_id.fetch_add(1, Ordering::SeqCst);
                self.vts.insert(vt, device.clone());
                Ok(vt)
            }

            _ => unreachable!(),
        }
    }

    fn write(&mut self, id: usize, buf: &[u8]) -> syscall::Result<usize> {
        if buf.len() == 1 && buf[0] > 0xf4 {
            return Ok(1);
        }

        let events = unsafe {
            core::slice::from_raw_parts(
                buf.as_ptr() as *const Event,
                buf.len() / core::mem::size_of::<Event>(),
            )
        };

        for event in events.iter() {
            let mut new_active_opt = None;
            match event.to_option() {
                EventOption::Key(key_event) => match key_event.scancode {
                    f @ 0x3B..=0x44 if self.super_key => {
                        // F1 through F10
                        new_active_opt = Some((f - 0x3A) as usize);
                    }

                    0x57 if self.super_key => {
                        // F11
                        new_active_opt = Some(11);
                    }

                    0x58 if self.super_key => {
                        // F12
                        new_active_opt = Some(12);
                    }

                    0x5B => {
                        // Super
                        self.super_key = key_event.pressed;
                    }

                    _ => (),
                },

                _ => continue,
            }

            let deactivate = |file: &mut File, id: usize| -> io::Result<()> {
                let command = Command::new(CommandTy::Deactivate, id).into_bytes();
                file.write(&command)?;
                Ok(())
            };

            if let Some(new_active) = new_active_opt {
                if let Some(vt) = self.vts.get(&new_active).cloned() {
                    if let Some((_, file, current)) = self.active_vt.as_mut() {
                        // If the VT is already active, don't do anything.
                        if new_active == *current {
                            continue;
                        }

                        deactivate(file, *current).unwrap();
                    } else {
                        let id = self.next_vt_id.load(Ordering::SeqCst) - 1;
                        let mut vt = File::open(self.vts.get_mut(&id).unwrap().as_ref()).unwrap();

                        deactivate(&mut vt, id).unwrap();
                    }

                    log::info!("inputd: switching to VT #{new_active} (`{vt}`)");

                    let mut file = File::open(vt.as_ref()).unwrap();
                    file.write(&Command::new(CommandTy::Activate, new_active).into_bytes())
                        .unwrap();

                    self.active_vt = Some((vt.clone(), file, new_active));
                } else {
                    log::warn!("inputd: switch to non-existent VT #{new_active} was requested");
                }
            }
        }

        let handle = self.handles.get_mut(&id).ok_or(SysError::new(EINVAL))?;
        assert!(handle.is_producer());

        for handle in self.handles.values_mut() {
            match handle {
                Handle::Consumer {
                    ref mut pending,
                    ref mut notified,
                    ..
                } => {
                    pending.extend_from_slice(buf);
                    *notified = false;
                }
                _ => continue,
            }
        }

        Ok(buf.len())
    }

    fn fevent(
        &mut self,
        id: usize,
        flags: syscall::EventFlags,
    ) -> syscall::Result<syscall::EventFlags> {
        let handle = self.handles.get_mut(&id).ok_or(SysError::new(EINVAL))?;

        match handle {
            Handle::Consumer {
                ref mut events,
                ref mut notified,
                ..
            } => {
                *events = flags;
                *notified = false;
            }
            _ => unreachable!(),
        }

        Ok(EventFlags::empty())
    }

    fn close(&mut self, _id: usize) -> syscall::Result<usize> {
        todo!()
    }
}

fn deamon(deamon: redox_daemon::Daemon) -> anyhow::Result<()> {
    // Create the ":input" scheme.
    let mut socket_file = File::create(":input")?;
    let mut scheme = InputScheme::new();

    deamon.ready().unwrap();

    loop {
        let mut should_handle = false;
        let mut packet = Packet::default();
        socket_file.read(&mut packet)?;

        // The producer has written to the channel; the consumers should be notified.
        if packet.a == syscall::SYS_WRITE {
            should_handle = true;
        }

        scheme.handle(&mut packet);
        socket_file.write(&packet)?;

        if !should_handle {
            continue;
        }

        for (id, handle) in scheme.handles.iter_mut() {
            if let Handle::Consumer {
                events,
                pending,
                ref mut notified,
                vt,
            } = handle
            {
                if pending.is_empty() || *notified || !events.contains(EventFlags::EVENT_READ) {
                    continue;
                }

                let should_notify = if let Some((_, _, active_vt)) = scheme.active_vt {
                    active_vt == *vt
                } else {
                    let last_allocated = scheme.next_vt_id.load(Ordering::SeqCst) - 1;
                    last_allocated == *vt
                };

                // The activate VT is not the same as the VT that the consumer is listening to 
                // for events.
                if !should_notify {
                    continue;
                }

                // Notify the consumer that we have some events to read. Yum yum.
                let mut event_packet = Packet::default();
                event_packet.a = syscall::SYS_FEVENT;
                event_packet.b = *id;
                event_packet.c = EventFlags::EVENT_READ.bits();
                // Specifies the number of bytes that can be read non-blocking.
                event_packet.d = pending.len();
                socket_file.write(&event_packet)?;

                *notified = true;
            }
        }
    }
}

fn daemon_runner(redox_daemon: redox_daemon::Daemon) -> ! {
    deamon(redox_daemon).unwrap();
    unreachable!();
}

#[cfg(target_os = "redox")]
pub fn setup_logging(level: log::LevelFilter, name: &str) {
    use redox_log::{OutputBuilder, RedoxLogger};

    let mut logger = RedoxLogger::new().with_output(
        OutputBuilder::stderr()
            .with_filter(level)
            .with_ansi_escape_codes()
            .flush_on_newline(true)
            .build(),
    );

    match OutputBuilder::in_redox_logging_scheme("disk", "pcie", format!("{name}.log")) {
        Ok(builder) => {
            logger = logger.with_output(builder.with_filter(level).flush_on_newline(true).build())
        }
        Err(err) => eprintln!("inputd: failed to create log: {}", err),
    }

    match OutputBuilder::in_redox_logging_scheme("disk", "pcie", format!("{name}.ansi.log")) {
        Ok(builder) => {
            logger = logger.with_output(
                builder
                    .with_filter(level)
                    .with_ansi_escape_codes()
                    .flush_on_newline(true)
                    .build(),
            )
        }
        Err(err) => eprintln!("inputd: failed to create ANSI log: {}", err),
    }

    logger.enable().unwrap();
    log::info!("inputd: enabled logger");
}

pub fn main() {
    #[cfg(target_os = "redox")]
    setup_logging(log::LevelFilter::Trace, "inputd");
    redox_daemon::Daemon::new(daemon_runner).expect("virtio-core: failed to daemonize");
}
