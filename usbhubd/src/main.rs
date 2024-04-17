use std::collections::VecDeque;
use std::env;
use std::fs::File;
use std::io::{Read, Write};

use redox_log::{OutputBuilder, RedoxLogger};
use xhcid_interface::{plain, usb, ConfigureEndpointsReq, DevDesc, DeviceReqData, EndpDirection, EndpointTy, PortReqRecipient, PortReqTy, XhciClientHandle};

fn setup_logging() -> Option<&'static RedoxLogger> {
    let mut logger = RedoxLogger::new().with_output(
        OutputBuilder::stderr()
            .with_filter(log::LevelFilter::Info) // limit global output to important info
            .with_ansi_escape_codes()
            .flush_on_newline(true)
            .build(),
    );

    #[cfg(target_os = "redox")]
    match OutputBuilder::in_redox_logging_scheme("usb", "device", "hub.log") {
        Ok(b) => {
            logger = logger.with_output(
                // TODO: Add a configuration file for this
                b.with_filter(log::LevelFilter::Info)
                    .flush_on_newline(true)
                    .build(),
            )
        }
        Err(error) => eprintln!("Failed to create hub.log: {}", error),
    }

    #[cfg(target_os = "redox")]
    match OutputBuilder::in_redox_logging_scheme("usb", "device", "hub.ansi.log") {
        Ok(b) => {
            logger = logger.with_output(
                b.with_filter(log::LevelFilter::Info)
                    .with_ansi_escape_codes()
                    .flush_on_newline(true)
                    .build(),
            )
        }
        Err(error) => eprintln!("Failed to create hub.ansi.log: {}", error),
    }

    match logger.enable() {
        Ok(logger_ref) => {
            eprintln!("usbhubd: enabled logger");
            Some(logger_ref)
        }
        Err(error) => {
            eprintln!("usbhubd: failed to set default logger: {}", error);
            None
        }
    }
}

fn main() {
    let _logger_ref = setup_logging();

    let mut args = env::args().skip(1);

    const USAGE: &'static str = "usbhubd <scheme> <port> <interface>";

    let scheme = args.next().expect(USAGE);
    let port = args
        .next()
        .expect(USAGE)
        .parse::<usize>()
        .expect("Expected integer as input of port");
    let interface_num = args
        .next()
        .expect(USAGE)
        .parse::<u8>()
        .expect("Expected integer as input of interface");

    log::info!(
        "USB HUB driver spawned with scheme `{}`, port {}, interface {}",
        scheme,
        port,
        interface_num
    );

    let handle = XhciClientHandle::new(scheme, port);
    let desc: DevDesc = handle
        .get_standard_descs()
        .expect("Failed to get standard descriptors");
    log::info!("{:X?}", desc);

    let (conf_desc, conf_num, if_desc) = desc
        .config_descs
        .iter()
        .enumerate()
        .find_map(|(conf_num, conf_desc)| {
            let if_desc = conf_desc.interface_descs.iter().find_map(|if_desc| {
                if if_desc.number == interface_num {
                    Some(if_desc.clone())
                } else {
                    None
                }
            })?;
            Some((
                conf_desc.clone(),
                conf_num,
                if_desc,
            ))
        })
        .expect("Failed to find suitable configuration");

    /*TODO
    handle
        .configure_endpoints(&ConfigureEndpointsReq {
            config_desc: conf_num as u8,
            interface_desc: Some(interface_num),
            alternate_setting: Some(if_desc.alternate_setting),
        })
        .expect("Failed to configure endpoints");
    */

    let mut hub_desc = usb::HubDescriptor::default();
    handle
        .device_request(
            PortReqTy::Class,
            PortReqRecipient::Device,
            0x6,
            0,
            //TODO: should this be an index into interface_descs?
            interface_num as u16,
            DeviceReqData::In(unsafe { plain::as_mut_bytes(&mut hub_desc) }),
        )
        .expect("Failed to retrieve hub descriptor");

    log::info!("{:X?}", hub_desc);
}
