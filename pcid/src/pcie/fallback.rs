use std::cell::Cell;
use std::convert::TryFrom;
use std::sync::Mutex;

#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
use syscall::io::{Io as _, Pio};

use log::info;

use crate::pci::{CfgAccess, PciAddress};

pub(crate) struct Pci {
    lock: Mutex<()>,
}

impl Pci {
    pub(crate) fn new() -> Self {
        Self {
            lock: Mutex::new(()),
        }
    }

    fn set_iopl() {
        // The IO privilege level is per-thread, so we need to do the initialization on every thread.
        thread_local! {
            static IOPL_ONCE: Cell<bool> = Cell::new(false);
        }

        IOPL_ONCE.with(|iopl_once| {
            if !iopl_once.replace(true) {
                // make sure that pcid is not granted io port permission unless pcie memory-mapped
                // configuration space is not available.
                info!(
                    "PCI: couldn't find or access PCIe extended configuration, \
                 and thus falling back to PCI 3.0 io ports"
                );
                unsafe {
                    syscall::iopl(3).expect("pcid: failed to set iopl to 3");
                }
            }
        });
    }

    fn address(address: PciAddress, offset: u8) -> u32 {
        // TODO: Find the part of pcid that uses an unaligned offset!
        //
        // assert_eq!(offset & 0xFC, offset, "pci offset is not aligned");
        //
        let offset = offset & 0xFC;

        assert_eq!(
            address.segment(),
            0,
            "usage of multiple segments requires PCIe extended configuration"
        );

        0x80000000
            | (u32::from(address.bus()) << 16)
            | (u32::from(address.device()) << 11)
            | (u32::from(address.function()) << 8)
            | u32::from(offset)
    }
}
#[cfg(any(target_arch = "x86", target_arch = "x86_64"))]
impl CfgAccess for Pci {
    unsafe fn read(&self, address: PciAddress, offset: u16) -> u32 {
        let _guard = self.lock.lock().unwrap();

        Self::set_iopl();

        let offset =
            u8::try_from(offset).expect("offset too large for PCI 3.0 configuration space");
        let address = Self::address(address, offset);

        Pio::<u32>::new(0xCF8).write(address);
        Pio::<u32>::new(0xCFC).read()
    }

    unsafe fn write(&self, address: PciAddress, offset: u16, value: u32) {
        let _guard = self.lock.lock().unwrap();

        Self::set_iopl();

        let offset =
            u8::try_from(offset).expect("offset too large for PCI 3.0 configuration space");
        let address = Self::address(address, offset);

        Pio::<u32>::new(0xCF8).write(address);
        Pio::<u32>::new(0xCFC).write(value);
    }
}
#[cfg(not(any(target_arch = "x86", target_arch = "x86_64")))]
impl CfgAccess for Pci {
    unsafe fn read(&self, addr: PciAddress, offset: u16) -> u32 {
        let _guard = self.lock.lock().unwrap();
        todo!("Pci::CfgAccess::read on this architecture")
    }

    unsafe fn write(&self, addr: PciAddress, offset: u16, value: u32) {
        let _guard = self.lock.lock().unwrap();
        todo!("Pci::CfgAccess::write on this architecture")
    }
}
