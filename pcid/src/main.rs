#![feature(asm)]

extern crate bitflags;
extern crate byteorder;
extern crate syscall;
extern crate toml;

use std::fs::{File, metadata, read_dir};
use std::io::prelude::*;
use std::os::unix::io::{FromRawFd, RawFd};
use std::process::Command;
use std::sync::{Arc, Mutex};
use std::{env, io, i64, thread};

use syscall::iopl;

use crate::config::Config;
use crate::pci::{Pci, PciBar, PciBus, PciClass, PciDev, PciFunc, PciHeader, PciHeaderError, PciHeaderType};
use crate::pci::cap::Capability as PciCapability;

mod config;
mod driver_interface;
mod pci;

pub struct DriverHandler {
    config: config::DriverConfig,
    bus_num: u8,
    dev_num: u8,
    func_num: u8,
    header: PciHeader,
    capabilities: Vec<(u8, PciCapability)>,

    state: Arc<State>,
}
fn with_pci_func_raw<T, F: FnOnce(&PciFunc) -> T>(pci: &Pci, bus_num: u8, dev_num: u8, func_num: u8, function: F) -> T {
    let bus = PciBus {
        pci,
        num: bus_num,
    };
    let dev = PciDev {
        bus: &bus,
        num: dev_num,
    };
    let func = PciFunc {
        dev: &dev,
        num: func_num,
    };
    function(&func)
}
impl DriverHandler {
    fn with_pci_func_raw<T, F: FnOnce(&PciFunc) -> T>(&self, function: F) -> T {
        with_pci_func_raw(&self.state.pci, self.bus_num, self.dev_num, self.func_num, function)
    }
    fn respond(&mut self, request: driver_interface::PcidClientRequest, args: &driver_interface::SubdriverArguments) -> driver_interface::PcidClientResponse {
        use driver_interface::*;
        use crate::pci::cap::{MsiCapability, MsixCapability};

        match request {
            PcidClientRequest::RequestConfig => {
                PcidClientResponse::Config(args.clone())
            }
            PcidClientRequest::RequestFeatures => {
                PcidClientResponse::AllFeatures(self.capabilities.iter().filter_map(|(_, capability)| match capability {
                    PciCapability::Msi(msi) => Some((PciFeature::Msi, FeatureStatus::enabled(msi.enabled()))),
                    PciCapability::MsiX(msix) => Some((PciFeature::MsiX, FeatureStatus::enabled(msix.msix_enabled()))),
                    _ => None,
                }).collect())
            }
            PcidClientRequest::EnableFeature(feature) => match feature {
                PciFeature::Msi => {
                    let (offset, capability): (u8, &mut MsiCapability) = match self.capabilities.iter_mut().find_map(|&mut (offset, ref mut capability)| capability.as_msi_mut().map(|cap| (offset, cap))) {
                        Some(tuple) => tuple,
                        None => return PcidClientResponse::Error(PcidServerResponseError::NonexistentFeature(feature)),
                    };
                    unsafe {
                        with_pci_func_raw(&self.state.pci, self.bus_num, self.dev_num, self.func_num, |func| {
                            capability.set_enabled(true);
                            capability.write_message_control(func, offset);
                        });
                    }
                    PcidClientResponse::FeatureEnabled(feature)
                }
                PciFeature::MsiX => {
                    let (offset, capability): (u8, &mut MsixCapability) = match self.capabilities.iter_mut().find_map(|&mut (offset, ref mut capability)| capability.as_msix_mut().map(|cap| (offset, cap))) {
                        Some(tuple) => tuple,
                        None => return PcidClientResponse::Error(PcidServerResponseError::NonexistentFeature(feature)),
                    };
                    unsafe {
                        with_pci_func_raw(&self.state.pci, self.bus_num, self.dev_num, self.func_num, |func| {
                            capability.set_msix_enabled(true);
                            capability.write_a(func, offset);
                        });
                    }
                    PcidClientResponse::FeatureEnabled(feature)
                }
            }
            PcidClientRequest::FeatureStatus(feature) => PcidClientResponse::FeatureStatus(feature, match feature {
                PciFeature::Msi => self.capabilities.iter().find_map(|(_, capability)| if let PciCapability::Msi(msi) = capability {
                    Some(FeatureStatus::enabled(msi.enabled()))
                } else {
                    None
                }).unwrap_or(FeatureStatus::Disabled),
                PciFeature::MsiX => self.capabilities.iter().find_map(|(_, capability)| if let PciCapability::MsiX(msix) = capability {
                    Some(FeatureStatus::enabled(msix.msix_enabled()))
                } else {
                    None
                }).unwrap_or(FeatureStatus::Disabled),
            }),
            PcidClientRequest::FeatureInfo(feature) => PcidClientResponse::FeatureInfo(feature, match feature {
                PciFeature::Msi => if let Some(info) = self.capabilities.iter().find_map(|(_, capability)| capability.as_msi()) {
                    PciFeatureInfo::Msi(*info)
                } else {
                    return PcidClientResponse::Error(PcidServerResponseError::NonexistentFeature(feature));
                }
                PciFeature::MsiX => if let Some(info) = self.capabilities.iter().find_map(|(_, capability)| capability.as_msix()) {
                    PciFeatureInfo::MsiX(*info)
                } else {
                    return PcidClientResponse::Error(PcidServerResponseError::NonexistentFeature(feature));
                }
            }),
        }
    }
    fn handle_spawn(mut self, pcid_to_client_write: Option<usize>, pcid_from_client_read: Option<usize>, args: driver_interface::SubdriverArguments) {
        use driver_interface::*;

        if let (Some(pcid_to_client_fd), Some(pcid_from_client_fd)) = (pcid_to_client_write, pcid_from_client_read) {
            let mut pcid_to_client = unsafe { File::from_raw_fd(pcid_to_client_fd as RawFd) };
            let mut pcid_from_client = unsafe { File::from_raw_fd(pcid_from_client_fd as RawFd) };

            while let Ok(msg) = recv(&mut pcid_from_client) {
                let response = self.respond(msg, &args);
                send(&mut pcid_to_client, &response).unwrap();
            }
        }
    }
}

pub struct State {
    threads: Mutex<Vec<thread::JoinHandle<()>>>,
    pci: Pci,
}

fn handle_parsed_header(state: Arc<State>, config: &Config, bus_num: u8,
                        dev_num: u8, func_num: u8, header: PciHeader) {
    let pci = &state.pci;

    let raw_class: u8 = header.class().into();
    let mut string = format!("PCI {:>02X}/{:>02X}/{:>02X} {:>04X}:{:>04X} {:>02X}.{:>02X}.{:>02X}.{:>02X} {:?}",
                             bus_num, dev_num, func_num, header.vendor_id(), header.device_id(), raw_class,
                             header.subclass(), header.interface(), header.revision(), header.class());
    match header.class() {
        PciClass::Legacy if header.subclass() == 1 => string.push_str("  VGA CTL"),
        PciClass::Storage => match header.subclass() {
            0x01 => {
                string.push_str(" IDE");
            },
            0x06 => if header.interface() == 0 {
                string.push_str(" SATA VND");
            } else if header.interface() == 1 {
                string.push_str(" SATA AHCI");
            },
            _ => ()
        },
        PciClass::SerialBus => match header.subclass() {
            0x03 => match header.interface() {
                0x00 => {
                    string.push_str(" UHCI");
                },
                0x10 => {
                    string.push_str(" OHCI");
                },
                0x20 => {
                    string.push_str(" EHCI");
                },
                0x30 => {
                    string.push_str(" XHCI");
                },
                _ => ()
            },
            _ => ()
        },
        _ => ()
    }

    for (i, bar) in header.bars().iter().enumerate() {
        if !bar.is_none() {
            string.push_str(&format!(" {}={}", i, bar));
        }
    }

    string.push('\n');

    print!("{}", string);

    for driver in config.drivers.iter() {
        if let Some(class) = driver.class {
            if class != raw_class { continue; }
        }

        if let Some(subclass) = driver.subclass {
            if subclass != header.subclass() { continue; }
        }

        if let Some(interface) = driver.interface {
            if interface != header.interface() { continue; }
        }

        if let Some(ref ids) = driver.ids {
            let mut device_found = false;
            for (vendor, devices) in ids {
                let vendor_without_prefix = vendor.trim_start_matches("0x");
                let vendor = i64::from_str_radix(vendor_without_prefix, 16).unwrap() as u16;

                if vendor != header.vendor_id() { continue; }

                for device in devices {
                    if *device == header.device_id() {
                        device_found = true;
                        break;
                    }
                }
            }
            if !device_found { continue; }
        } else {
            if let Some(vendor) = driver.vendor {
                if vendor != header.vendor_id() { continue; }
            }

            if let Some(device) = driver.device {
                if device != header.device_id() { continue; }
            }
        }

        if let Some(ref device_id_range) = driver.device_id_range {
            if header.device_id() < device_id_range.start  ||
               device_id_range.end <= header.device_id() { continue; }
        }

        if let Some(ref args) = driver.command {
            // Enable bus mastering, memory space, and I/O space
            unsafe {
                let mut data = pci.read(bus_num, dev_num, func_num, 0x04);
                data |= 7;
                pci.write(bus_num, dev_num, func_num, 0x04, data);
            }

            // Set IRQ line to 9 if not set
            let mut irq;
            unsafe {
                let mut data = pci.read(bus_num, dev_num, func_num, 0x3C);
                irq = (data & 0xFF) as u8;
                if irq == 0xFF {
                    irq = 9;
                }
                data = (data & 0xFFFFFF00) | irq as u32;
                pci.write(bus_num, dev_num, func_num, 0x3C, data);
            }

            let interrupt_pin = unsafe { pci.read(bus_num, dev_num, func_num, 0x3B) };

            // Find BAR sizes
            let mut bars = [PciBar::None; 6];
            let mut bar_sizes = [0; 6];
            unsafe {
                let count = match header.header_type() {
                    PciHeaderType::GENERAL => 6,
                    PciHeaderType::PCITOPCI => 2,
                    _ => 0,
                };

                for i in 0..count {
                    bars[i] = header.get_bar(i);

                    let offset = 0x10 + (i as u8) * 4;

                    let original = pci.read(bus_num, dev_num, func_num, offset);
                    pci.write(bus_num, dev_num, func_num, offset, 0xFFFFFFFF);

                    let new = pci.read(bus_num, dev_num, func_num, offset);
                    pci.write(bus_num, dev_num, func_num, offset, original);

                    let masked = if new & 1 == 1 {
                        new & 0xFFFFFFFC
                    } else {
                        new & 0xFFFFFFF0
                    };

                    let size = !masked + 1;
                    bar_sizes[i] = if size <= 1 {
                        0
                    } else {
                        size
                    };
                }
            }

            let capabilities = {
                let bus = PciBus {
                    pci,
                    num: bus_num,
                };
                let dev = PciDev {
                    bus: &bus,
                    num: dev_num
                };
                let func = PciFunc {
                    dev: &dev,
                    num: func_num,
                };
                crate::pci::cap::CapabilitiesIter { inner: crate::pci::cap::CapabilityOffsetsIter::new(header.cap_pointer(), &func) }.collect::<Vec<_>>()
            };
            println!("PCI DEVICE CAPABILITIES for {}: {:?}", args.iter().map(|string| string.as_ref()).nth(0).unwrap_or("[unknown]"), capabilities);

            use driver_interface::LegacyInterruptPin;

            let legacy_interrupt_pin = match interrupt_pin {
                0 => None,
                1 => Some(LegacyInterruptPin::IntA),
                2 => Some(LegacyInterruptPin::IntB),
                3 => Some(LegacyInterruptPin::IntC),
                4 => Some(LegacyInterruptPin::IntD),

                other => {
                    println!("pcid: invalid interrupt pin: {}", other);
                    None
                }
            };

            let func = driver_interface::PciFunction {
                bars,
                bar_sizes,
                bus_num,
                dev_num,
                func_num,
                devid: header.device_id(),
                legacy_interrupt_line: irq,
                legacy_interrupt_pin,
                venid: header.vendor_id(),
            };

            let subdriver_args = driver_interface::SubdriverArguments {
                func,
            };

            let mut args = args.iter();
            if let Some(program) = args.next() {
                let mut command = Command::new(program);
                for arg in args {
                    let arg = match arg.as_str() {
                        "$BUS" => format!("{:>02X}", bus_num),
                        "$DEV" => format!("{:>02X}", dev_num),
                        "$FUNC" => format!("{:>02X}", func_num),
                        "$NAME" => format!("pci-{:>02X}.{:>02X}.{:>02X}", bus_num, dev_num, func_num),
                        "$BAR0" => format!("{}", bars[0]),
                        "$BAR1" => format!("{}", bars[1]),
                        "$BAR2" => format!("{}", bars[2]),
                        "$BAR3" => format!("{}", bars[3]),
                        "$BAR4" => format!("{}", bars[4]),
                        "$BAR5" => format!("{}", bars[5]),
                        "$BARSIZE0" => format!("{:>08X}", bar_sizes[0]),
                        "$BARSIZE1" => format!("{:>08X}", bar_sizes[1]),
                        "$BARSIZE2" => format!("{:>08X}", bar_sizes[2]),
                        "$BARSIZE3" => format!("{:>08X}", bar_sizes[3]),
                        "$BARSIZE4" => format!("{:>08X}", bar_sizes[4]),
                        "$BARSIZE5" => format!("{:>08X}", bar_sizes[5]),
                        "$IRQ" => format!("{}", irq),
                        "$VENID" => format!("{:>04X}", header.vendor_id()),
                        "$DEVID" => format!("{:>04X}", header.device_id()),
                        _ => arg.clone()
                    };
                    command.arg(&arg);
                }

                println!("PCID SPAWN {:?}", command);

                let (pcid_to_client_write, pcid_from_client_read, envs) = if driver.channel_name.is_some() {
                    let mut fds1 = [0usize; 2];
                    let mut fds2 = [0usize; 2];

                    syscall::pipe2(&mut fds1, 0).expect("pcid: failed to create pcid->client pipe");
                    syscall::pipe2(&mut fds2, 0).expect("pcid: failed to create client->pcid pipe");

                    let [pcid_to_client_read, pcid_to_client_write] = fds1;
                    let [pcid_from_client_read, pcid_from_client_write] = fds2;

                    (Some(pcid_to_client_write), Some(pcid_from_client_read), vec! [("PCID_TO_CLIENT_FD", format!("{}", pcid_to_client_read)), ("PCID_FROM_CLIENT_FD", format!("{}", pcid_from_client_write))])
                } else {
                    (None, None, vec! [])
                };

                match command.envs(envs).spawn() {
                    Ok(mut child) => {
                        let driver_handler = DriverHandler {
                            bus_num,
                            dev_num,
                            func_num,
                            config: driver.clone(),
                            header,
                            state: Arc::clone(&state),
                            capabilities,
                        };
                        let thread = thread::spawn(move || {
                            driver_handler.handle_spawn(pcid_to_client_write, pcid_from_client_read, subdriver_args);
                        });
                        match child.wait() {
                            Ok(_status) => (),
                            Err(err) => println!("pcid: failed to wait for {:?}: {}", command, err),
                        }
                    }
                    Err(err) => println!("pcid: failed to execute {:?}: {}", command, err)
                }
            }
        }
    }
}

fn main() {
    let mut config = Config::default();

    let mut args = env::args().skip(1);
    if let Some(config_path) = args.next() {
        if metadata(&config_path).unwrap().is_file() {
            if let Ok(mut config_file) = File::open(&config_path) {
                let mut config_data = String::new();
                if let Ok(_) = config_file.read_to_string(&mut config_data) {
                    config = toml::from_str(&config_data).unwrap_or(Config::default());
                }
            }
        } else {
            let paths = read_dir(&config_path).unwrap();

            let mut config_data = String::new();

            for path in paths {
                if let Ok(mut config_file) = File::open(&path.unwrap().path()) {
                    let mut tmp = String::new();
                    if let Ok(_) = config_file.read_to_string(&mut tmp) {
                        config_data.push_str(&tmp);
                    }
                }
            }

            config = toml::from_str(&config_data).unwrap_or(Config::default());
        }
    }

    let state = Arc::new(State {
        pci: Pci::new(),
        threads: Mutex::new(Vec::new()),
    });

    let pci = &state.pci;

    unsafe { iopl(3).unwrap() };

    print!("PCI BS/DV/FN VEND:DEVI CL.SC.IN.RV\n");

    'bus: for bus in pci.buses() {
        'dev: for dev in bus.devs() {
            for func in dev.funcs() {
                let func_num = func.num;
                match PciHeader::from_reader(func) {
                    Ok(header) => {
                        handle_parsed_header(Arc::clone(&state), &config, bus.num, dev.num, func_num, header);
                    }
                    Err(PciHeaderError::NoDevice) => {
                        if func_num == 0 {
                            if dev.num == 0 {
                                // println!("PCI {:>02X}: no bus", bus.num);
                                continue 'bus;
                            } else {
                                // println!("PCI {:>02X}/{:>02X}: no dev", bus.num, dev.num);
                                continue 'dev;
                            }
                        }
                    },
                    Err(PciHeaderError::UnknownHeaderType(id)) => {
                        println!("pcid: unknown header type: {}", id);
                    }
                }
            }
        }
    }

    for thread in state.threads.lock().unwrap().drain(..) {
        thread.join().unwrap();
    }
}
