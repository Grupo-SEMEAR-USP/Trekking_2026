#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_control__msg__I2cData() -> *const std::ffi::c_void;
}

#[link(name = "robot_control__rosidl_generator_c")]
extern "C" {
    fn robot_control__msg__I2cData__init(msg: *mut I2cData) -> bool;
    fn robot_control__msg__I2cData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<I2cData>, size: usize) -> bool;
    fn robot_control__msg__I2cData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<I2cData>);
    fn robot_control__msg__I2cData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<I2cData>, out_seq: *mut rosidl_runtime_rs::Sequence<I2cData>) -> bool;
}

// Corresponds to robot_control__msg__I2cData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// I2cData.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct I2cData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: u32,

}



impl Default for I2cData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_control__msg__I2cData__init(&mut msg as *mut _) {
        panic!("Call to robot_control__msg__I2cData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for I2cData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__I2cData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__I2cData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__I2cData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for I2cData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for I2cData where Self: Sized {
  const TYPE_NAME: &'static str = "robot_control/msg/I2cData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_control__msg__I2cData() }
  }
}


#[link(name = "robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_control__msg__UARTData() -> *const std::ffi::c_void;
}

#[link(name = "robot_control__rosidl_generator_c")]
extern "C" {
    fn robot_control__msg__UARTData__init(msg: *mut UARTData) -> bool;
    fn robot_control__msg__UARTData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<UARTData>, size: usize) -> bool;
    fn robot_control__msg__UARTData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<UARTData>);
    fn robot_control__msg__UARTData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<UARTData>, out_seq: *mut rosidl_runtime_rs::Sequence<UARTData>) -> bool;
}

// Corresponds to robot_control__msg__UARTData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// UARTData.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UARTData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: u32,

}



impl Default for UARTData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_control__msg__UARTData__init(&mut msg as *mut _) {
        panic!("Call to robot_control__msg__UARTData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for UARTData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__UARTData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__UARTData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__UARTData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for UARTData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for UARTData where Self: Sized {
  const TYPE_NAME: &'static str = "robot_control/msg/UARTData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_control__msg__UARTData() }
  }
}


#[link(name = "robot_control__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_control__msg__VelocityData() -> *const std::ffi::c_void;
}

#[link(name = "robot_control__rosidl_generator_c")]
extern "C" {
    fn robot_control__msg__VelocityData__init(msg: *mut VelocityData) -> bool;
    fn robot_control__msg__VelocityData__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<VelocityData>, size: usize) -> bool;
    fn robot_control__msg__VelocityData__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<VelocityData>);
    fn robot_control__msg__VelocityData__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<VelocityData>, out_seq: *mut rosidl_runtime_rs::Sequence<VelocityData>) -> bool;
}

// Corresponds to robot_control__msg__VelocityData
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]

/// VelocityData.msg

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct VelocityData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_speed_left: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_speed_right: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub servo_angle: f32,

}



impl Default for VelocityData {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_control__msg__VelocityData__init(&mut msg as *mut _) {
        panic!("Call to robot_control__msg__VelocityData__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for VelocityData {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__VelocityData__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__VelocityData__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_control__msg__VelocityData__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for VelocityData {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for VelocityData where Self: Sized {
  const TYPE_NAME: &'static str = "robot_control/msg/VelocityData";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_control__msg__VelocityData() }
  }
}


