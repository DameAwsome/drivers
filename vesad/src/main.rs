#![deny(warnings)]
#![feature(alloc)]
#![feature(allocator_api)]
#![feature(asm)]

extern crate alloc;
extern crate orbclient;
extern crate syscall;

use std::env;
use std::fs::File;
use std::io::{Read, Write};
use syscall::{physmap, physunmap, Packet, SchemeMut, EVENT_READ, MAP_WRITE, MAP_WRITE_COMBINE};

use mode_info::VBEModeInfo;
use primitive::fast_set64;
use scheme::DisplayScheme;

pub mod display;
pub mod mode_info;
pub mod primitive;
pub mod scheme;
pub mod screen;

fn main() {
    let mut spec = Vec::new();

    for arg in env::args().skip(1) {
        if arg == "T" {
            spec.push(false);
        } else if arg == "G" {
            spec.push(true);
        } else {
            println!("vesad: unknown screen type: {}", arg);
        }
    }

    let width;
    let height;
    let physbaseptr;

    {
        let mode_info = unsafe { &*(physmap(0x5200, 4096, 0).expect("vesad: failed to map VBE info") as *const VBEModeInfo) };

        width = mode_info.xresolution as usize;
        height = mode_info.yresolution as usize;
        physbaseptr = mode_info.physbaseptr as usize;

        unsafe { let _ = physunmap(mode_info as *const _ as usize); }
    }

    if physbaseptr > 0 {
        // Daemonize
        if unsafe { syscall::clone(0).unwrap() } == 0 {
            let mut socket = File::create(":display").expect("vesad: failed to create display scheme");

            let size = width * height;
            //TODO: Remap on resize
            let largest_size = 8 * 1024 * 1024;
            let onscreen = unsafe { physmap(physbaseptr, largest_size * 4, MAP_WRITE | MAP_WRITE_COMBINE).expect("vesad: failed to map VBE LFB") };
            unsafe { fast_set64(onscreen as *mut u64, 0, size/2) };

            let mut scheme = DisplayScheme::new(width, height, onscreen, &spec);

            syscall::setrens(0, 0).expect("vesad: failed to enter null namespace");

            let mut blocked = Vec::new();
            loop {
                let mut packet = Packet::default();
                socket.read(&mut packet).expect("vesad: failed to read display scheme");

                // If it is a read packet, and there is no data, block it. Otherwise, handle packet
                if packet.a == syscall::number::SYS_READ && packet.d > 0 && scheme.can_read(packet.b).is_none() {
                    blocked.push(packet);
                } else {
                    scheme.handle(&mut packet);
                    socket.write(&packet).expect("vesad: failed to write display scheme");
                }

                // If there are blocked readers, and data is available, handle them
                {
                    let mut i = 0;
                    while i < blocked.len() {
                        if scheme.can_read(blocked[i].b).is_some() {
                            let mut packet = blocked.remove(i);
                            scheme.handle(&mut packet);
                            socket.write(&packet).expect("vesad: failed to write display scheme");
                        } else {
                            i += 1;
                        }
                    }
                }

                for (handle_id, handle) in scheme.handles.iter() {
                    if handle.events & EVENT_READ != 0 {
                        if let Some(count) = scheme.can_read(*handle_id) {
                            if count > 0 {
                                let event_packet = Packet {
                                    id: 0,
                                    pid: 0,
                                    uid: 0,
                                    gid: 0,
                                    a: syscall::number::SYS_FEVENT,
                                    b: *handle_id,
                                    c: EVENT_READ,
                                    d: count
                                };

                                socket.write(&event_packet).expect("vesad: failed to write display event");
                            }
                        }
                    }
                }
            }
        }
    }
}
