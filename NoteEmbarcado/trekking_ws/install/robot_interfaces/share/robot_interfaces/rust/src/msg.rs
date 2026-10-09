#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to robot_interfaces__msg__UARTData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct UARTData {

    // This member is not documented.
    #[allow(missing_docs)]
    pub x: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub y: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub z: i32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timestamp: u32,

}



impl Default for UARTData {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::UARTData::default())
  }
}

impl rosidl_runtime_rs::Message for UARTData {
  type RmwMsg = super::msg::rmw::UARTData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        x: msg.x,
        y: msg.y,
        z: msg.z,
        timestamp: msg.timestamp,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      x: msg.x,
      y: msg.y,
      z: msg.z,
      timestamp: msg.timestamp,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      x: msg.x,
      y: msg.y,
      z: msg.z,
      timestamp: msg.timestamp,
    }
  }
}


// Corresponds to robot_interfaces__msg__VelocityData

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::VelocityData::default())
  }
}

impl rosidl_runtime_rs::Message for VelocityData {
  type RmwMsg = super::msg::rmw::VelocityData;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        angular_speed_left: msg.angular_speed_left,
        angular_speed_right: msg.angular_speed_right,
        servo_angle: msg.servo_angle,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      angular_speed_left: msg.angular_speed_left,
      angular_speed_right: msg.angular_speed_right,
      servo_angle: msg.servo_angle,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      angular_speed_left: msg.angular_speed_left,
      angular_speed_right: msg.angular_speed_right,
      servo_angle: msg.servo_angle,
    }
  }
}


