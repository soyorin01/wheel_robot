#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to wheel_robot__msg__ObstacleStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ObstacleStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ObstacleStatus::default())
  }
}

impl rosidl_runtime_rs::Message for ObstacleStatus {
  type RmwMsg = super::msg::rmw::ObstacleStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        status: msg.status,
        left_status: msg.left_status,
        center_status: msg.center_status,
        right_status: msg.right_status,
        left_distance: msg.left_distance,
        center_distance: msg.center_distance,
        right_distance: msg.right_distance,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      status: msg.status,
      left_status: msg.left_status,
      center_status: msg.center_status,
      right_status: msg.right_status,
      left_distance: msg.left_distance,
      center_distance: msg.center_distance,
      right_distance: msg.right_distance,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      status: msg.status,
      left_status: msg.left_status,
      center_status: msg.center_status,
      right_status: msg.right_status,
      left_distance: msg.left_distance,
      center_distance: msg.center_distance,
      right_distance: msg.right_distance,
    }
  }
}


// Corresponds to wheel_robot__msg__StepApproachStatus

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StepApproachStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StepApproachStatus::default())
  }
}

impl rosidl_runtime_rs::Message for StepApproachStatus {
  type RmwMsg = super::msg::rmw::StepApproachStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        state: msg.state.as_str().into(),
        locked: msg.locked,
        step_detected: msg.step_detected,
        detected_distance: msg.detected_distance,
        height: msg.height,
        confidence: msg.confidence,
        stop_distance: msg.stop_distance,
        target_travel: msg.target_travel,
        traveled: msg.traveled,
        remaining: msg.remaining,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
        state: msg.state.as_str().into(),
      locked: msg.locked,
      step_detected: msg.step_detected,
      detected_distance: msg.detected_distance,
      height: msg.height,
      confidence: msg.confidence,
      stop_distance: msg.stop_distance,
      target_travel: msg.target_travel,
      traveled: msg.traveled,
      remaining: msg.remaining,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      state: msg.state.to_string(),
      locked: msg.locked,
      step_detected: msg.step_detected,
      detected_distance: msg.detected_distance,
      height: msg.height,
      confidence: msg.confidence,
      stop_distance: msg.stop_distance,
      target_travel: msg.target_travel,
      traveled: msg.traveled,
      remaining: msg.remaining,
    }
  }
}


// Corresponds to wheel_robot__msg__StepDetection

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StepDetection {

    // This member is not documented.
    #[allow(missing_docs)]
    pub header: std_msgs::msg::Header,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::StepDetection::default())
  }
}

impl rosidl_runtime_rs::Message for StepDetection {
  type RmwMsg = super::msg::rmw::StepDetection;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Owned(msg.header)).into_owned(),
        detected: msg.detected,
        distance: msg.distance,
        height: msg.height,
        confidence: msg.confidence,
        pose_valid: msg.pose_valid,
        edge_angle_rad: msg.edge_angle_rad,
        lateral_offset_m: msg.lateral_offset_m,
        confirm_frames: msg.confirm_frames,
        required_confirm_frames: msg.required_confirm_frames,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        header: std_msgs::msg::Header::into_rmw_message(std::borrow::Cow::Borrowed(&msg.header)).into_owned(),
      detected: msg.detected,
      distance: msg.distance,
      height: msg.height,
      confidence: msg.confidence,
      pose_valid: msg.pose_valid,
      edge_angle_rad: msg.edge_angle_rad,
      lateral_offset_m: msg.lateral_offset_m,
      confirm_frames: msg.confirm_frames,
      required_confirm_frames: msg.required_confirm_frames,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      header: std_msgs::msg::Header::from_rmw_message(msg.header),
      detected: msg.detected,
      distance: msg.distance,
      height: msg.height,
      confidence: msg.confidence,
      pose_valid: msg.pose_valid,
      edge_angle_rad: msg.edge_angle_rad,
      lateral_offset_m: msg.lateral_offset_m,
      confirm_frames: msg.confirm_frames,
      required_confirm_frames: msg.required_confirm_frames,
    }
  }
}


