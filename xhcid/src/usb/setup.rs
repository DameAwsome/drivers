use super::DescriptorKind;

#[repr(packed)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Setup {
    pub kind: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}

#[repr(u8)]
pub enum ReqDirection {
    HostToDevice = 0,
    DeviceToHost = 1,
}

#[repr(u8)]
pub enum ReqType {
    /// Standard device requests, such as SET_ADDRESS and SET_CONFIGURATION. These aren't directly
    /// accessible using the API, but are sent from xhcid when required.
    Standard = 0,

    /// Class specific requests that are directly accessible from the API.
    Class = 1,

    /// Vendor specific requests that are accessible using the API.
    Vendor = 2,

    /// Reserved
    Reserved = 3,
}

#[repr(u8)]
pub enum ReqRecipient {
    Device = 0,
    Interface = 1,
    Endpoint = 2,
    Other = 3,
    // 4..=30 are reserved
    VendorSpecific = 31,
}

pub const USB_SETUP_DIR_BIT: u8 = 1 << 7;
pub const USB_SETUP_DIR_SHIFT: u8 = 7;
pub const USB_SETUP_REQ_TY_MASK: u8 = 0x60;
pub const USB_SETUP_REQ_TY_SHIFT: u8 = 5;
pub const USB_SETUP_RECIPIENT_MASK: u8 = 0x1F;
pub const USB_SETUP_RECIPIENT_SHIFT: u8 = 0;

impl Setup {
    pub fn direction(&self) -> ReqDirection {
        if self.kind & USB_SETUP_DIR_BIT == 0 {
            ReqDirection::HostToDevice
        } else {
            ReqDirection::DeviceToHost
        }
    }
    pub const fn req_ty(&self) -> u8 {
        (self.kind & USB_SETUP_REQ_TY_MASK) >> USB_SETUP_REQ_TY_SHIFT
    }

    pub const fn req_recipient(&self) -> u8 {
        (self.kind & USB_SETUP_RECIPIENT_MASK) >> USB_SETUP_RECIPIENT_SHIFT
    }
    pub fn is_allowed_from_api(&self) -> bool {
        self.req_ty() == ReqType::Class as u8 || self.req_ty() == ReqType::Vendor as u8
    }

    pub const fn get_status() -> Self {
        Self {
            kind: 0b1000_0000,
            request: 0x00,
            value: 0,
            index: 0,
            length: 2,
        }
    }

    pub const fn clear_feature(feature: u16) -> Self {
        Self {
            kind: 0b0000_0000,
            request: 0x01,
            value: feature,
            index: 0,
            length: 0,
        }
    }

    pub const fn set_feature(feature: u16) -> Self {
        Self {
            kind: 0b0000_0000,
            request: 0x03,
            value: feature,
            index: 0,
            length: 0,
        }
    }

    pub const fn set_address(address: u16) -> Self {
        Self {
            kind: 0b0000_0000,
            request: 0x05,
            value: address,
            index: 0,
            length: 0,
        }
    }

    pub const fn get_descriptor(
        kind: DescriptorKind,
        index: u8,
        language: u16,
        length: u16,
    ) -> Self {
        Self {
            kind: 0b1000_0000,
            request: 0x06,
            value: ((kind as u16) << 8) | (index as u16),
            index: language,
            length: length,
        }
    }

    pub const fn set_descriptor(kind: u8, index: u8, language: u16, length: u16) -> Self {
        Self {
            kind: 0b0000_0000,
            request: 0x07,
            value: ((kind as u16) << 8) | (index as u16),
            index: language,
            length: length,
        }
    }

    pub const fn get_configuration() -> Self {
        Self {
            kind: 0b1000_0000,
            request: 0x08,
            value: 0,
            index: 0,
            length: 1,
        }
    }

    pub const fn set_configuration(value: u16) -> Self {
        Self {
            kind: 0b0000_0000,
            request: 0x09,
            value: value,
            index: 0,
            length: 0,
        }
    }
}
