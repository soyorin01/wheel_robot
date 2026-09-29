#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to motor_temperature_pkg__msg__MotorTemperatures

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorTemperatures {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: Vec<std::string::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub temperature_c: Vec<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub warning_threshold_c: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub overheated_index: Vec<u8>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub overheated: bool,

}



impl Default for MotorTemperatures {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::MotorTemperatures::default())
  }
}

impl rosidl_runtime_rs::Message for MotorTemperatures {
  type RmwMsg = super::msg::rmw::MotorTemperatures;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        name: msg.name
          .into_iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        temperature_c: msg.temperature_c.into(),
        warning_threshold_c: msg.warning_threshold_c,
        overheated_index: msg.overheated_index.into(),
        overheated: msg.overheated,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        name: msg.name
          .iter()
          .map(|elem| elem.as_str().into())
          .collect(),
        temperature_c: msg.temperature_c.as_slice().into(),
      warning_threshold_c: msg.warning_threshold_c,
        overheated_index: msg.overheated_index.as_slice().into(),
      overheated: msg.overheated,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      name: msg.name
          .into_iter()
          .map(|elem| elem.to_string())
          .collect(),
      temperature_c: msg.temperature_c
          .into_iter()
          .collect(),
      warning_threshold_c: msg.warning_threshold_c,
      overheated_index: msg.overheated_index
          .into_iter()
          .collect(),
      overheated: msg.overheated,
    }
  }
}


