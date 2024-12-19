#![no_std]
#![deny(unsafe_code)]
#![feature(strict_provenance)]

extern crate alloc;
use alloc::boxed::Box;
use crate::alloc::string::ToString;
use alloc::sync::Arc;
use aster_virtio::device::block::DEVICE_NAME;
use aster_virtio::device::socket::buffer;
use aster_virtio::device::network::header::VirtioNetHdr;
use ostd::mm;
use ostd::mm::VmIoOnce;
use ostd::mm::{VmReader,DmaDirection,DmaStream,VmWriter,FrameAllocOptions};
use ostd::prelude::println;
use ostd::bus::pci::common_device::PciCommonDevice;
use ostd::bus::pci::cfg_space::Bar;
use ostd::bus::pci::cfg_space::MemoryBar;
use ostd::sync::{Mutex,SpinLock,LocalIrqDisabled};
use ostd::bus::BusProbeError;
use ostd::bus::pci::PCI_BUS;
use ostd::Pod;
use ostd::bus::pci::bus::PciDevice;
use ostd::bus::pci::bus::PciDriver;
use alloc::{vec::Vec};
use ostd::io_mem::IoMem;
use component::{init_component, ComponentInitError};
use ostd::bus::pci::PciDeviceId;
use aster_network::{DmaSegment, RxBuffer, TxBuffer,AnyNetworkDevice,VirtioNetError,EthernetAddr};
use aster_bigtcp::device::DeviceCapabilities;
use core::fmt::Debug;
use alloc::{fmt, slice};
use alloc::collections::linked_list::LinkedList;
use aster_network::dma_pool;
use ostd::bus::pci::cfg_space::AddrLen;
use aster_util::{field_ptr, safe_ptr::SafePtr};
use ostd::mm::{DmaCoherent, HasDaddr, HasPaddr, Paddr, VmIo};
use core::mem;

const OFFSET:usize = 16384;
const TDOFFSET:usize = mem::size_of::<TD>();
const RDOFFSET:usize = mem::size_of::<TD>();
const RD_DD :u8 = 1;
const RDT : usize = 0x2818;
#[init_component]
fn e1000_init() -> Result<(), ComponentInitError> {
    driver_e1000_init();
    Ok(())
}

/// The dma descriptor for transmitting 1
#[derive(Debug, Clone,Pod,Copy)]
#[repr(C, align(16))]
pub struct TD {
addr: u64,
length: usize,
cso: u8,
cmd: u8,
status: u8,
css: u8,
special: u16,
}
/// [E1000 3.2.3]
/// The dma descriptor for receiving
#[derive(Debug, Clone,Pod,Copy)]
#[repr(C, align(16))]
pub struct RD {
addr: u64, /* Address of the descriptor's data buffer */
length: usize, /* Length of data DMAed into data buffer */
csum: u16, /* Packet checksum */
status: u8, /* Descriptor status */
errors: u8, /* Descriptor Errors */
special: u16,
}
//#[derive(Pod)]
#[repr(C)]
pub struct PciDeviceE1000 {
    common_device: PciCommonDevice,
    base: usize,
    mac_address: EthernetAddr,
    header: VirtioNetHdr,
    caps: DeviceCapabilities,
    receive_buffers: DmaCoherent,
    receive_ring: DmaCoherent,
    receive_index: usize,

    transmit_buffers: DmaCoherent,
    transmit_ring: DmaCoherent,
    //transmit_ring_free: usize,
    transmit_index: usize,
    transmit_clean_index: usize,
    //td_dma_coherent:DmaCoherent
    dma_pool_device: Arc<dma_pool::DmaPool>
}

impl fmt::Debug for PciDeviceE1000 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("PciDeviceE1000")
            .field("common_device", &self.common_device)
            .field("base", &self.base)
            .field("mac_address", &self.mac_address)
            .field("cap", &self.caps)
            .field("header", &self.header)
            .field("receive_index", &self.receive_index)
            .field("transmit_index", &self.transmit_index)
            .field("transmit_clean_index", &self.transmit_clean_index)
            .finish()
    }
}
/* 
impl AnyNetworkDevice for PciDeviceE1000 {
    fn mac_addr(&self) -> EthernetAddr {
        self.mac_address
    }
    fn capabilities(&self) -> DeviceCapabilities {
        self.caps
        }
    
    }*/


impl PciDeviceE1000{
    fn mac_addr(&self) -> EthernetAddr {
        self.mac_address
    }
    pub fn new(common_device: PciCommonDevice,mac_address:EthernetAddr) -> Self{
        let dma_pool_new = dma_pool::DmaPool::new(
            mm::PAGE_SIZE,
            10,
            50,
            DmaDirection::Bidirectional,
            false,
        );
        let new_transmission_buffer:Vec<[u8;16384]> = Vec::with_capacity(16);
        let vm_segment = FrameAllocOptions::new(1)
        .is_contiguous(true)
        .alloc_contiguous()
        .unwrap();
        let trans_dma_coherent = DmaCoherent::map(vm_segment.clone(), false).unwrap();
        for i in 0 .. new_transmission_buffer.len(){
            trans_dma_coherent.write_bytes(i * OFFSET, &new_transmission_buffer[i]).unwrap();
        }
        let mut new_transmission_ring:Vec<TD> = Vec::with_capacity(16);
        for  i in 0 .. new_transmission_ring.len(){
            let td = TD{
                addr : (trans_dma_coherent.daddr() + i * OFFSET) as u64,
                length : 0,
                cso : 0,
                cmd : 0,
                status : 0,
                css : 0,
                special : 0,
            };
            new_transmission_ring.push(td);
        } 
        let vm_segment = FrameAllocOptions::new(1)
        .is_contiguous(true)
        .alloc_contiguous()
        .unwrap();
        let td_dma_coherent = DmaCoherent::map(vm_segment.clone(), false).unwrap();
        for i in 0 .. new_transmission_ring.len(){
            td_dma_coherent.write_val(i * TDOFFSET, &new_transmission_ring[i]).unwrap();
        }
        let vm_segment = FrameAllocOptions::new(1)
        .is_contiguous(true)
        .alloc_contiguous()
        .unwrap();
        let new_receive_buffer:Vec<[u8;16384]> = Vec::with_capacity(16);
        let recv_dma_coherent = DmaCoherent::map(vm_segment.clone(), false).unwrap();
        for i in 0 .. new_receive_buffer.len() {
            recv_dma_coherent.write_bytes(i * OFFSET, &new_receive_buffer[i]).unwrap();
        }
        let mut new_receive_ring:Vec<RD> = Vec::with_capacity(16);
        for  i in 0 .. new_receive_ring.len(){
            let rd = RD{
                addr : (recv_dma_coherent.daddr() + i * OFFSET) as u64,
                length : 0,
                status:0,
                csum : 0,
                errors : 0,
                special : 0,
            };
            new_receive_ring.push(rd);
        }
        println!("bytes1:{}",recv_dma_coherent.nbytes());
        let vm_segment = FrameAllocOptions::new(1)
        .is_contiguous(true)
        .alloc_contiguous()
        .unwrap();
        let rd_dma_coherent = DmaCoherent::map(vm_segment.clone(), false).unwrap();
        for i in 0 .. new_receive_buffer.len() {
            rd_dma_coherent.write_val(i * RDOFFSET, &new_receive_ring[i]).unwrap();
        }
        println!("bytes2:{}",rd_dma_coherent.nbytes());
        let device=PciDeviceE1000{
            common_device,
            base: 0,
            mac_address: mac_address,
            header: VirtioNetHdr::default(),
            caps: DeviceCapabilities::default(),
            receive_ring: recv_dma_coherent,
            receive_buffers: rd_dma_coherent,
            receive_index: 0,

            transmit_ring:td_dma_coherent,
            transmit_buffers:trans_dma_coherent,
            transmit_index: 0,
            transmit_clean_index: 0,
            //td_dma_coherent: dma_coherent
            dma_pool_device: dma_pool_new,
        };
        /*aster_network::register_device(
            DEVICE_NAME.to_string(),
            Arc::new(SpinLock::new(device)),
        );
        let pci_device=aster_network::get_device(DEVICE_NAME).unwrap().lock();
        pci_device8*/
        device
    }

   
    /* 
    pub fn receive_packet(&mut self) -> Result<Vec<u8>, VirtioNetError>{
        let buffer = self.receive_buffers[self.receive_index].as_ref();
        let rd = self.receive_ring[self.receive_index].as_ref();
        let mut reader = (&buffer::segment).reader().unwrap();
        let packet = [0u8; rd.length];
        reader.read(&mut VmWriter::from(&mut packet as &mut [u8]));
        self.receive_index = (self.receive_index + 1) % 64;
        Ok(packet)
        //TODO: Implement notify the receive end when the data arrive.
    }*/
    pub fn send_packet(&mut self, packet: &[u8]) -> Result<(), VirtioNetError>{
        if self.transmit_index >= 16 {
            return Err(VirtioNetError::WrongToken);
        }
        let td = TD{
            addr:(self.transmit_buffers.daddr() + self.transmit_index * OFFSET) as u64,
            length:packet.len(),
            cso:0,
            cmd:0,
            status:0,
            css:0,
            special:0,
        };
        self.transmit_ring.write_val(self.transmit_index * TDOFFSET, &td).unwrap();
        self.transmit_buffers.write_bytes(self.transmit_index * OFFSET, packet);
        self.transmit_index = (self.transmit_index + 1) % 16;
        Ok(())
        //TODO: Implement notify the device when the data is ready.
    }
    pub fn recevice_packet(&mut self) -> Result<RxBuffer,VirtioNetError>{
        let mut rd:RD = self.receive_ring.read_val(self.receive_index * RDOFFSET).unwrap();
        if rd.status & RD_DD == RD_DD{
            rd.status = 0;
            let mut buf :Vec<u8> = Vec::with_capacity(rd.length);
            let slice:&mut [u8] = buf.as_mut_slice();
            self.receive_buffers.read_bytes(self.receive_index * RDOFFSET, slice).unwrap();
            let bars=self.common_device.bar_manager();
            let bar=bars.bar(0).unwrap();
            if let Bar::Memory(memory_bar)=&bar{
                memory_bar.io_mem().write_once(RDT, &self.receive_index).unwrap();
             }
            
            let mut rx_buffer = RxBuffer::new(size_of::<VirtioNetHdr>(), &self.dma_pool_device);
            rx_buffer.set_packet_len(rd.length - size_of::<VirtioNetHdr>());
            let mut writer = rx_buffer.get_segment().writer().unwrap();
            let slice = buf.as_slice();
            writer.write(&mut VmReader::from(slice));
            //是否需要写回去
            self.receive_buffers.write_val(self.receive_index * RDOFFSET, &rd).unwrap();
            self.receive_index = (self.receive_index + 1) & 16;
            Ok(rx_buffer)
        }else{
            Err(VirtioNetError::NotReady)
        }
    }

}
// static TX_BUFFER_POOL: SpinLock<LinkedList<DmaStream>, LocalIrqDisabled> =
//     SpinLock::new(LinkedList::new());
    
impl PciDevice for PciDeviceE1000 {
    fn device_id(&self) -> PciDeviceId {
        self.common_device.device_id().clone()
    }
}

impl AnyNetworkDevice for PciDeviceE1000 {
    fn mac_addr(&self) -> EthernetAddr {
        self.mac_address
    }

    fn capabilities(&self) -> DeviceCapabilities {
        self.caps.clone()
    }

    fn can_receive(&self) -> bool {
        let rd :RD = self.receive_ring.read_val(self.receive_index * RDOFFSET).unwrap();
        rd.status == 0
    }

    fn can_send(&self) -> bool {
        let td :TD = self.transmit_ring.read_val(self.transmit_index * TDOFFSET).unwrap();
        td.status == 0
    }
 //加了一个get_segment不知道是否可行
    fn receive(&mut self) -> Result<RxBuffer, VirtioNetError> {
        self.recevice_packet()
    }

    fn send(&mut self, packet: &[u8]) -> Result<(), VirtioNetError> {
        self.send_packet(packet)
    }
 //可能有些问题，组会时再讨论
    fn free_processed_tx_buffers(&mut self) {
        while let mut td= self.transmit_ring.read_val::<TD>(self.transmit_clean_index * TDOFFSET).unwrap(){
            if td.status != 0 {
                td.status = 0;
                self.transmit_ring.write_val(self.transmit_index * TDOFFSET, &td).unwrap();
                self.transmit_clean_index += 1;
            }else {
                break;
            }
        }
    }
}
#[derive(Debug)]
pub struct PciDriverE1000 {
    devices: Mutex<Vec<Arc<PciDeviceE1000>>>,
}
fn flag(memory_bar: &Arc<MemoryBar>,register: usize,flag: u32,value : bool){
    if value {
        memory_bar.io_mem().write_once(register,&(memory_bar.io_mem().read_once::<u32>(register as usize).unwrap() | flag)).unwrap();
    }else {
        memory_bar.io_mem().write_once(register,&(memory_bar.io_mem().read_once::<u32>(register as usize).unwrap() & !flag)).unwrap();
    }
}
impl PciDriver for PciDriverE1000 {
    fn probe(
        &self,
        device: PciCommonDevice
    ) -> Result<Arc<dyn PciDevice>, (BusProbeError, PciCommonDevice)> {
        // 检查设备是否匹配
        if device.device_id().vendor_id != 0x8086 || device.device_id().device_id != 0x100E {
            // 0x8086 是 Intel 的 Vendor ID，0x100E 是 e1000 的 Device ID
            return Err((BusProbeError::DeviceNotMatch, device));
        }
        // 创建 DMA 池
        // let dma_pool_new = dma_pool::DmaPool::new(
        //     mm::PAGE_SIZE,
        //     10,
        //     50,
        //     DmaDirection::Bidirectional,
        //     false
        // );

        // 获取设备的 MAC 地址，假设可以从设备配置空间读取
        const RAL0:usize=0x5400;
        const RAH0: usize= 0x5404;
        const CTRL:usize=0x0;
        const STATUS:usize=0x8;
        const TDT : usize=0x3818;
        const TDLEN : usize=0x3808;
        const TDBAL : usize=0x3800;
        const TDBAH : usize=0x3804;
        const TCTL : usize=0x400;
        const TCTL_EN : u32=1 << 1;
        const TCTL_PSP : u32=1 << 3; 
        let bars=device.bar_manager();
        let bar=bars.bar(0).unwrap();
        let mut mac_low:u32=0;
        let mut mac_high:u32=0;
        let mut ctl:u32=0;
        let mut status:u32=0;
        // 创建 PciDeviceE1000 的实例
        
        if let Bar::Memory(memory_bar)=&bar{
            ctl=memory_bar.io_mem().read_once(CTRL).unwrap();
            status=memory_bar.io_mem().read_once(STATUS).unwrap();
            mac_low=memory_bar.io_mem().read_once(RAL0).unwrap();
            mac_high=memory_bar.io_mem().read_once(RAH0).unwrap();
         }
         let pci_device = Arc::new(PciDeviceE1000 ::new(device, aster_network::EthernetAddr([0, 0, 0, 0, 0, 0])));
        // 将设备添加到驱动的设备列表
        // println!("{:?}",pci_device);
        self.devices.lock().push(pci_device.clone());
        if let Bar::Memory(new_memory_bar) =&bar{
            new_memory_bar.io_mem().write_once(TDT ,&pci_device.transmit_index).unwrap();
            new_memory_bar.io_mem().write_once(TDLEN , &(TDOFFSET * 16)).unwrap();
            new_memory_bar.io_mem().write_once(TDBAH , &(((pci_device.transmit_ring.paddr() as u64) >> 32) as u32)).unwrap();
            new_memory_bar.io_mem().write_once(TDBAL as usize, &(pci_device.transmit_ring.paddr() as u32)).unwrap();
            flag(&new_memory_bar, TCTL as usize, TCTL_EN, true);
            flag(&new_memory_bar, TCTL as usize, TCTL_PSP, true);
        }
         println!("Read value: {:?}", ctl);
         println!("Read value: {:?}", status);
         println!("Read value: {:x?}", mac_low);
         aster_network::register_device(
            DEVICE_NAME.to_string(),
            Arc::new(SpinLock::new(pci_device)),
        );
        // 返回创建的设备
        Ok(pci_device.clone())
    }}


pub fn driver_e1000_init() {
    let driver_a = Arc::new(PciDriverE1000 {
        //修改Mutex
        devices: Mutex::new(Vec::new()),
    });
    PCI_BUS.lock().register_driver(driver_a);
}
