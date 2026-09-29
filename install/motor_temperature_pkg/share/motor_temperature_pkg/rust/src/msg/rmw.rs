#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "motor_temperature_pkg__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__motor_temperature_pkg__msg__MotorTemperatures() -> *const std::ffi::c_void;
}

#[link(name = "motor_temperature_pkg__rosidl_generator_c")]
extern "C" {
    fn motor_temperature_pkg__msg__MotorTemperatures__init(msg: *mut MotorTemperatures) -> bool;
    fn motor_temperature_pkg__msg__MotorTemperatures__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<MotorTemperatures>, size: usize) -> bool;
    fn motor_temperature_pkg__msg__MotorTemperatures__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<MotorTemperatures>);
    fn motor_temperature_pkg__msg__MotorTemperatures__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<MotorTemperatures>, out_seq: *mut rosidl_runtime_rs::Sequence<MotorTemperatures>) -> bool;
}

// Corresponds to motor_temperature_pkg__msg__MotorTemperatures
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct MotorTemperatures {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub name: rosidl_runtime_rs::Sequence<rosidl_runtime_rs::String>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub temperature_c: rosidl_runtime_rs::Sequence<f32>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub warning_threshold_c: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub overheated_index: rosidl_runtime_rs::Sequence<u8>,


    // This member is not documented.
    #[allow(missing_docs)]
    pub overheated: bool,

}



impl Default for MotorTemperatures {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !motor_temperature_pkg__msg__MotorTemperatures__init(&mut msg as *mut _) {
        panic!("Call to motor_temperature_pkg__msg__MotorTemperatures__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for MotorTemperatures {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_temperature_pkg__msg__MotorTemperatures__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_temperature_pkg__msg__MotorTemperatures__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { motor_temperature_pkg__msg__MotorTemperatures__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for MotorTemperatures {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for MotorTemperatures where Self: Sized {
  const TYPE_NAME: &'static str = "motor_temperature_pkg/msg/MotorTemperatures";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__motor_temperature_pkg__msg__MotorTemperatures() }
  }
}


