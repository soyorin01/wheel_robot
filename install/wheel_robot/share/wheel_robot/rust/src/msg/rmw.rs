#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};


#[link(name = "wheel_robot__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wheel_robot__msg__ObstacleStatus() -> *const std::ffi::c_void;
}

#[link(name = "wheel_robot__rosidl_generator_c")]
extern "C" {
    fn wheel_robot__msg__ObstacleStatus__init(msg: *mut ObstacleStatus) -> bool;
    fn wheel_robot__msg__ObstacleStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ObstacleStatus>, size: usize) -> bool;
    fn wheel_robot__msg__ObstacleStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ObstacleStatus>);
    fn wheel_robot__msg__ObstacleStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ObstacleStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<ObstacleStatus>) -> bool;
}

// Corresponds to wheel_robot__msg__ObstacleStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub status: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_status: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub center_status: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_status: u8,


    // This member is not documented.
    #[allow(missing_docs)]
    pub left_distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub center_distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub right_distance: f32,

}

impl ObstacleStatus {

    // This constant is not documented.
    #[allow(missing_docs)]
    pub const UNKNOWN: u8 = 0;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CLEAR: u8 = 1;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const CAUTION: u8 = 2;


    // This constant is not documented.
    #[allow(missing_docs)]
    pub const STOP: u8 = 3;

}


impl Default for ObstacleStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wheel_robot__msg__ObstacleStatus__init(&mut msg as *mut _) {
        panic!("Call to wheel_robot__msg__ObstacleStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ObstacleStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__ObstacleStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__ObstacleStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__ObstacleStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ObstacleStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ObstacleStatus where Self: Sized {
  const TYPE_NAME: &'static str = "wheel_robot/msg/ObstacleStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wheel_robot__msg__ObstacleStatus() }
  }
}


#[link(name = "wheel_robot__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wheel_robot__msg__StepApproachStatus() -> *const std::ffi::c_void;
}

#[link(name = "wheel_robot__rosidl_generator_c")]
extern "C" {
    fn wheel_robot__msg__StepApproachStatus__init(msg: *mut StepApproachStatus) -> bool;
    fn wheel_robot__msg__StepApproachStatus__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StepApproachStatus>, size: usize) -> bool;
    fn wheel_robot__msg__StepApproachStatus__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StepApproachStatus>);
    fn wheel_robot__msg__StepApproachStatus__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StepApproachStatus>, out_seq: *mut rosidl_runtime_rs::Sequence<StepApproachStatus>) -> bool;
}

// Corresponds to wheel_robot__msg__StepApproachStatus
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StepApproachStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub locked: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub step_detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detected_distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub height: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub stop_distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_travel: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub traveled: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub remaining: f32,

}



impl Default for StepApproachStatus {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wheel_robot__msg__StepApproachStatus__init(&mut msg as *mut _) {
        panic!("Call to wheel_robot__msg__StepApproachStatus__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StepApproachStatus {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__StepApproachStatus__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__StepApproachStatus__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__StepApproachStatus__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StepApproachStatus {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StepApproachStatus where Self: Sized {
  const TYPE_NAME: &'static str = "wheel_robot/msg/StepApproachStatus";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wheel_robot__msg__StepApproachStatus() }
  }
}


#[link(name = "wheel_robot__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__wheel_robot__msg__StepDetection() -> *const std::ffi::c_void;
}

#[link(name = "wheel_robot__rosidl_generator_c")]
extern "C" {
    fn wheel_robot__msg__StepDetection__init(msg: *mut StepDetection) -> bool;
    fn wheel_robot__msg__StepDetection__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StepDetection>, size: usize) -> bool;
    fn wheel_robot__msg__StepDetection__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StepDetection>);
    fn wheel_robot__msg__StepDetection__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StepDetection>, out_seq: *mut rosidl_runtime_rs::Sequence<StepDetection>) -> bool;
}

// Corresponds to wheel_robot__msg__StepDetection
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StepDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::rmw::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub detected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub distance: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub height: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confidence: f32,

    /// 台阶前沿在相机地面坐标系中的姿态。
    /// edge_angle_rad=0 表示台阶横边与相机 x 轴平行，即车头正对台阶。
    /// 正值表示拟合直线 z=a*x+b 的 a>0；lateral_offset_m 正值表示台阶中心在车体右侧。
    pub pose_valid: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub edge_angle_rad: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub lateral_offset_m: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub confirm_frames: u32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub required_confirm_frames: u32,

}



impl Default for StepDetection {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !wheel_robot__msg__StepDetection__init(&mut msg as *mut _) {
        panic!("Call to wheel_robot__msg__StepDetection__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StepDetection {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__StepDetection__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__StepDetection__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { wheel_robot__msg__StepDetection__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StepDetection {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StepDetection where Self: Sized {
  const TYPE_NAME: &'static str = "wheel_robot/msg/StepDetection";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__wheel_robot__msg__StepDetection() }
  }
}


