#!/usr/bin/env python3
"""轮足机器人比赛程序：巡线、环岛、人行横道、黄线计圈、5cm台阶跳跃。"""

from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import math
import signal
import threading
import time
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray
from wheel_robot.msg import StepDetection

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:
    Image = None
    ImageDraw = None
    ImageFont = None


# ============================================================================
# 程序默认值：现场调车请改 config/icn.yaml，不要直接改这里。
# ============================================================================


class RuntimeParams:
    ENABLE_MOTION = True
    SHOW_DEBUG = True
    RECORD_RAW_VIDEO = True
    RECORD_DEBUG_VIDEO = True
class CameraParams:
    BACKEND = "v4l2"
    ID = 0
    DEVICE = "/dev/video0"
    WIDTH = 640
    HEIGHT = 480
    FPS = 30.0
    PIXEL_FORMAT = "YUYV"
    ROTATION_DEGREES = 0
    MIRROR_MODE = "none"
    CONTROL_RATE = 15.0
class DriveParams:
    FORWARD_SPEED = 0.25          # 全局巡线速度；想更快/更慢主要调它
    MIN_FORWARD_SPEED = 0.10      # 弯道最低速度
    MAX_TURN_SPEED = 0.70         # 巡线最大转弯角速度
    CURVE_SLOWDOWN = 0.72         # 弯道降速强度；弯道冲出去就加大
    LINEAR_ACCEL = 0.38           # 起步/减速平滑
    ANGULAR_ACCEL = 2.50
    BIG_TURN_LOSS_SECONDS = 1.20  # 90度大弯丢线后继续找线多久；停太早就加大
    BIG_TURN_LOSS_SPEED = 0.075   # 大弯丢线找线前进速度；冲出赛道就减小
    BIG_TURN_LOSS_TURN = 0.34     # 大弯丢线找线转向；转不过来就加大


class RoundaboutParams:
    FORWARD_SPEED = 0.19          # 环岛固定左转前进速度
    LEFT_TURN_SPEED = 0.34        # 环岛没进去：加大；转太猛/压边：减小
    ARM_AFTER_START_DISTANCE = 1.00  # 每圈先走这么远才允许识别环岛；误触发就加大
    FIRST_LAP_APPROACH_DISTANCE = 1.20   # 第一圈入环：太早转加大，太晚转减小
    SECOND_LAP_APPROACH_DISTANCE = 1.20  # 第二圈入环：太早转加大，太晚转减小
    SECOND_LAP_USE_FIRST_POSITION = False # 第二圈也必须看见环岛线；不再靠里程位置提前进环
    SECOND_LAP_ADVANCE = 0.00
    ENTRY_TURN_DISTANCE = 1.00    # 固定左转保持距离；没进去加大，绕过头减小
    ENTRY_MAX_SECONDS = 7.0       # 固定左转最多多久；绕多了就减小
    FOLLOW_MAX_SECONDS = 28.0     # 进去了没出来：减小；太早出环又回环岛：加大
    EXIT_COMMIT_SECONDS = 1.20    # 出口确认后保护时间；出环后又拐回去就加大
    EXIT_SPEED = 0.105
    BLIND_RECOVERY_SPEED = 0.10
    BLIND_RECOVERY_LEFT_TURN = 0.16
    BLUE_GUARD_SPEED = 0.045
    BLUE_GUARD_TURN_SPEED = 0.30


class CrosswalkParams:
    DETECT_DELAY_AFTER_ROUNDABOUT = 1.50
    CONFIRM_FRAMES = 2
    APPROACH_DISTANCE = 0.30      # 识别人行道后继续走的距离；压线/越过就减小，停太早就加大
    STOP_SECONDS = 5.0


class YellowLineParams:
    TOTAL_LAPS = 2
    CONFIRM_FRAMES = 2
    DETECT_DELAY_AFTER_CROSSWALK = 0.45
    ACCEPT_WHILE_SEEK_CROSSWALK = False # 必须先完成“人行道停车”，再允许黄线换圈/结束
    FINISH_PASS_DISTANCE = 1.50   # 第二圈看到黄线后继续冲过的距离；停太早加大

class StepParams:
    MANUAL_CONFIRM_BEFORE_JUMP = True  # 调参保护：到起跳点先停车，按回车才跳
    APPROACH_SPEED = 0.50         # 台阶冲刺速度；单独测试稳定就别改
    JUMP_FORWARD_SPEED = 0.50
    JUMP_MIN_HEIGHT = 0.15        # 跳跃低点
    JUMP_MAX_HEIGHT = 0.33        # 跳不上去：适当加大；弹太高/落地不稳：减小
    SPRINT_DISTANCE = 0.61       # 固定冲刺总距离；跳早加大，跳晚减小
    JUMP_TRIGGER_LEAD = 0.055     # 提前触发量；跳早减小，跳晚加大
    START_DEPTH = 0.70            # 深度到这个距离开始冲刺；一般不动
    ARM_MIN_SEEK_DISTANCE = 1.25  # 台阶太早接管就加大；一直不接管就减小
    ARM_MAX_SEEK_DISTANCE = 3.20
    SECOND_LAP_SPEED_CAP = 0.16   # 第二圈台阶前普通巡线限速
    POST_REACQUIRE_SECONDS = 3.8   # 跳完偏左/偏右后，允许慢速找线多久
    POST_REACQUIRE_DISTANCE = 0.38 # 跳完找线最多往前蹭多远，避免一直盲走
    POST_REACQUIRE_SPEED = 0.055   # 跳完找线速度
    POST_REACQUIRE_TURN = -0.24    # 跳完偏左找不回线：负数向右找；偏右则改正数


# ------------------------------ 程序内部，不用看 ------------------------------
# 这些旧名字仍由主逻辑使用；启动时会被 config/icn.yaml 覆盖。
CAMERA_BACKEND = CameraParams.BACKEND
CAMERA_ID = CameraParams.ID
CAMERA_DEVICE = CameraParams.DEVICE
IMAGE_WIDTH = CameraParams.WIDTH
IMAGE_HEIGHT = CameraParams.HEIGHT
CAPTURE_FPS = CameraParams.FPS
PIXEL_FORMAT = CameraParams.PIXEL_FORMAT
ROTATION_DEGREES = CameraParams.ROTATION_DEGREES
MIRROR_MODE = CameraParams.MIRROR_MODE
CONTROL_RATE = CameraParams.CONTROL_RATE
IMAGE_TIMEOUT_SECONDS = 0.35
CAMERA_RETRY_SECONDS = 1.0
SHOW_DEBUG = RuntimeParams.SHOW_DEBUG
PROCESS_WIDTH = 320
PROCESS_HEIGHT = 240
BINARY_PREVIEW_WIDTH = 160
BINARY_PREVIEW_HEIGHT = 120
BINARY_PREVIEW_MARGIN = 8
ENABLE_MOTION = RuntimeParams.ENABLE_MOTION
RECORD_RAW_VIDEO = RuntimeParams.RECORD_RAW_VIDEO
RECORD_DEBUG_VIDEO = RuntimeParams.RECORD_DEBUG_VIDEO
DEBUG_VIDEO_CODEC = "MJPG"

FORWARD_SPEED = DriveParams.FORWARD_SPEED
MINIMUM_FORWARD_SPEED = DriveParams.MIN_FORWARD_SPEED
MAX_ANGULAR_SPEED = DriveParams.MAX_TURN_SPEED
CURVE_SLOWDOWN = DriveParams.CURVE_SLOWDOWN
LINEAR_DIRECTION_SIGN = 1.0
STEERING_SIGN = -1.0
STEERING_KP = 1.08
STEERING_KD = 0.018
ERROR_FILTER_ALPHA = 0.48
MAX_ERROR_DERIVATIVE = 3.0
MAX_LINEAR_ACCELERATION = DriveParams.LINEAR_ACCEL
MAX_ANGULAR_ACCELERATION = DriveParams.ANGULAR_ACCEL

SEEK_DISTANCE_MIN_COUNT_SPEED = 0.08

SHORT_LOSS_SECONDS = 0.45
SEARCH_LOSS_SECONDS = 0.80
LOSS_FORWARD_SPEED = 0.035
LOSS_SEARCH_SPEED = 0.0
LOSS_MIN_ANGULAR = 0.0
LOSS_MAX_ANGULAR = 0.26
LOSS_ANGULAR_DECAY = 0.40
BIG_TURN_LOSS_SECONDS = DriveParams.BIG_TURN_LOSS_SECONDS
BIG_TURN_LOSS_SPEED = DriveParams.BIG_TURN_LOSS_SPEED
BIG_TURN_LOSS_TURN = DriveParams.BIG_TURN_LOSS_TURN
BIG_TURN_LOSS_MIN_RECENT_TURN = 0.20
BIG_TURN_LOSS_HINT_KEEP_SECONDS = 1.40

ROUNDABOUT_FORWARD_SPEED = RoundaboutParams.FORWARD_SPEED
ROUNDABOUT_ENTRY_LEFT_ANGULAR = RoundaboutParams.LEFT_TURN_SPEED
ROUNDABOUT_ENTRY_TURN_DISTANCE = RoundaboutParams.ENTRY_TURN_DISTANCE
ROUNDABOUT_SECOND_LAP_USE_FIRST_POSITION = RoundaboutParams.SECOND_LAP_USE_FIRST_POSITION
ROUNDABOUT_SECOND_LAP_ADVANCE_M = RoundaboutParams.SECOND_LAP_ADVANCE
ROUNDABOUT_ENTRY_FALLBACK_SECONDS = RoundaboutParams.ENTRY_MAX_SECONDS
ROUNDABOUT_MIN_SECONDS = 8.0
ROUNDABOUT_MIN_RELATIVE_YAW = 4.00
ROUNDABOUT_MIN_RELATIVE_DISTANCE = 1.20
ROUNDABOUT_ARM_FALLBACK_SECONDS = 9.0
ROUNDABOUT_RETURN_GATE_MIN_SCORE = 0.72
ROUNDABOUT_RETURN_GATE_CONFIRM_FRAMES = 3
ROUNDABOUT_FORCE_EXIT_DISTANCE = 1.65
ROUNDABOUT_FORCE_EXIT_SECONDS = 11.8
ROUNDABOUT_GATE_STRONG_SCORE = 0.82
ROUNDABOUT_GATE_WEAK_SCORE = 0.78
ROUNDABOUT_GATE_EVIDENCE_TRIGGER = 2.40
ROUNDABOUT_GATE_EVIDENCE_MAX = 5.00
ROUNDABOUT_GATE_EVIDENCE_STRONG_GAIN = 1.00
ROUNDABOUT_GATE_EVIDENCE_WEAK_GAIN = 0.35
ROUNDABOUT_GATE_EVIDENCE_DECAY = 0.30
ROUNDABOUT_GATE_CONFIRM_DELAY_ALLOWANCE_M = 0.08
ROUNDABOUT_GATE_CONFIRM_DELAY_MAX_COMP_M = 0.15
ROUNDABOUT_RETURN_EVIDENCE_TRIGGER = 2.20
ROUNDABOUT_RETURN_EVIDENCE_GAIN = 1.00
ROUNDABOUT_RETURN_EVIDENCE_DECAY = 0.25
ROUNDABOUT_EXIT_SPEED = RoundaboutParams.EXIT_SPEED
ROUNDABOUT_EXIT_MIN_ANGULAR = -0.03
ROUNDABOUT_EXIT_MAX_ANGULAR = 0.22
ROUNDABOUT_EXIT_DIRECTION_LOCK_SECONDS = 0.95
ROUNDABOUT_EXIT_LOCK_MIN_ANGULAR = -0.01
ROUNDABOUT_EXIT_MIN_SECONDS = 0.55
ROUNDABOUT_EXIT_MAX_SECONDS = 2.60
ROUNDABOUT_EXIT_MIN_DISTANCE = 0.24
ROUNDABOUT_EXIT_FORCE_DISTANCE = 0.36
ROUNDABOUT_EXIT_STABLE_CONFIRM_FRAMES = 3
ROUNDABOUT_EXIT_SOLID_SCORE = 0.48
ROUNDABOUT_FIRST_LAP_APPROACH_DISTANCE = RoundaboutParams.FIRST_LAP_APPROACH_DISTANCE
ROUNDABOUT_SECOND_LAP_APPROACH_DISTANCE = RoundaboutParams.SECOND_LAP_APPROACH_DISTANCE
ROUNDABOUT_APPROACH_MIN_SECONDS = 0.45
ROUNDABOUT_APPROACH_FALLBACK_SECONDS = 1.25
ROUNDABOUT_GATE_DETECT_DELAY = 1.0
ROUNDABOUT_GATE_MIN_TRAVEL = RoundaboutParams.ARM_AFTER_START_DISTANCE
ROUNDABOUT_GATE_NO_ODOM_FALLBACK_SECONDS = 5.0
ROUNDABOUT_GATE_MIN_CHAIN = 5
ROUNDABOUT_GATE_MIN_Y_SPAN_RATIO = 0.22
ROUNDABOUT_GATE_MAX_GAP_CV = 0.90
ROUNDABOUT_MAX_SECONDS = 14.0
ROUNDABOUT_EXIT_SOLID_MIN_HEIGHT_RATIO = 0.30
ROUNDABOUT_EXIT_SOLID_MAX_WIDTH_RATIO = 0.24
ROUNDABOUT_EXIT_SOLID_MIN_AREA = 120
ROUNDABOUT_EXIT_SOLID_TOP_MAX_RATIO = 0.62
ROUNDABOUT_EXIT_SOLID_BOTTOM_MIN_RATIO = 0.76
SHOW_EXIT_SOLID_BOX = False
ROUNDABOUT_EXIT_GUARD_SECONDS = 1.20
ROUNDABOUT_EXIT_GUARD_MIN_ANGULAR = -0.03
ROUNDABOUT_EXIT_GUARD_SPEED = 0.15
ROUNDABOUT_FOLLOW_SECONDS = RoundaboutParams.FOLLOW_MAX_SECONDS
ROUNDABOUT_EXIT_COMMIT_SECONDS = RoundaboutParams.EXIT_COMMIT_SECONDS
ROUNDABOUT_BLIND_RECOVERY_SPEED = RoundaboutParams.BLIND_RECOVERY_SPEED
ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR = RoundaboutParams.BLIND_RECOVERY_LEFT_TURN
ROUNDABOUT_BLUE_GUARD_ENABLED = True
ROUNDABOUT_BLUE_GUARD_FORWARD_Y_TOP = 0.45
ROUNDABOUT_BLUE_GUARD_FORWARD_Y_BOTTOM = 0.78
ROUNDABOUT_BLUE_GUARD_MIN_BLUE_FRACTION = 0.10
ROUNDABOUT_BLUE_GUARD_WHITE_Y_TOP = 0.55
ROUNDABOUT_BLUE_GUARD_WHITE_Y_BOTTOM = 0.92
ROUNDABOUT_BLUE_GUARD_WHITE_S_MAX = 70
ROUNDABOUT_BLUE_GUARD_WHITE_V_MIN = 125
ROUNDABOUT_BLUE_GUARD_MIN_WHITE_SIDE = 0.32
ROUNDABOUT_BLUE_GUARD_MIN_WHITE_IMBALANCE = 0.12
ROUNDABOUT_BLUE_GUARD_SPEED = RoundaboutParams.BLUE_GUARD_SPEED
ROUNDABOUT_BLUE_GUARD_ANGULAR = RoundaboutParams.BLUE_GUARD_TURN_SPEED
ROUNDABOUT_FOLLOW_MIN_ANGULAR = -0.04
ROUNDABOUT_REJECT_RIGHT_EDGE_RESCUE = True

CROSSWALK_APPROACH_DISTANCE = CrosswalkParams.APPROACH_DISTANCE
CROSSWALK_APPROACH_MIN_SECONDS = 0.50
CROSSWALK_APPROACH_FALLBACK_SECONDS = 4.5
CROSSWALK_STOP_SECONDS = CrosswalkParams.STOP_SECONDS
CROSSWALK_CONFIRM_FRAMES = CrosswalkParams.CONFIRM_FRAMES
CROSSWALK_DETECT_DELAY_AFTER_ROUNDABOUT = CrosswalkParams.DETECT_DELAY_AFTER_ROUNDABOUT

YELLOW_FINISH_CONFIRM_FRAMES = YellowLineParams.CONFIRM_FRAMES
FINISH_DETECT_DELAY_AFTER_CROSSWALK = YellowLineParams.DETECT_DELAY_AFTER_CROSSWALK
YELLOW_ACCEPT_WHILE_SEEK_CROSSWALK = YellowLineParams.ACCEPT_WHILE_SEEK_CROSSWALK
TOTAL_LAPS = YellowLineParams.TOTAL_LAPS
FINISH_PASS_DISTANCE = YellowLineParams.FINISH_PASS_DISTANCE
FINISH_PASS_MIN_SECONDS = 0.60
FINISH_PASS_FALLBACK_SECONDS = 2.5

STEP_ENABLED = True
STEP_DETECTION_TOPIC = "/step_detector/detection"
STEP_DETECTION_TIMEOUT_SECONDS = 0.50
STEP_ODOM_TIMEOUT_SECONDS = 0.50
STEP_ALIGN_MAX_HEADING_DELTA = 0.18
STEP_ALIGN_MAX_NEAR_ABS = 0.75
STEP_ALIGN_MIN_CONFIDENCE = 0.42
STEP_ALIGN_MIN_VALID_BANDS = 3
STEP_ALIGN_CONFIRM_FRAMES = 2
STEP_ALIGN_FORCE_LOCK_DISTANCE_M = 0.45
STEP_ALIGN_FORCE_MAX_HEADING_DELTA = 0.22
STEP_ARM_MAX_HEADING_DELTA = 0.24
STEP_ARM_MIN_CONFIDENCE = 0.48
STEP_ARM_MIN_VALID_BANDS = 3
STEP_IMU_TOPIC = "/imu/data"
STEP_IMU_TIMEOUT_SECONDS = 0.40
STEP_ALIGN_HEADING_KP = 1.35
STEP_ALIGN_CENTER_KP = 0.05
STEP_ALIGN_MAX_ANGULAR = 0.34
STEP_ALIGN_FORWARD_SPEED = 0.045
STEP_ALIGN_NEAR_STEP_HOLD_MARGIN = 0.12
STEP_SPRINT_CALIBRATION_MODE = False
STEP_CALIB_MAX_NEAR_MID_DELTA = 0.08
STEP_CALIB_MAX_MID_FAR_DELTA = 0.09
STEP_CALIB_MAX_NEAR_FAR_DELTA = 0.14
STEP_CALIB_CENTER_ERROR_MAX = 0.08
STEP_CALIB_CONFIRM_FRAMES = 4
STEP_CALIB_TARGET_DEPTH_M = StepParams.START_DEPTH
STEP_FIXED_SPRINT_ENABLED = True
STEP_FIXED_SPRINT_START_DEPTH_M = StepParams.START_DEPTH
STEP_FIXED_SPRINT_DISTANCE_M = StepParams.SPRINT_DISTANCE
STEP_FIXED_START_TOLERANCE_M = 0.015
STEP_FIXED_START_CONFIRM_FRAMES = 3
STEP_FIXED_START_REVERSE_TO_M = 0.76
STEP_ALIGN_SLOWDOWN_DEPTH_M = 0.82
STEP_ALIGN_CREEP_DEPTH_M = 0.74
STEP_ALIGN_SLOW_SPEED_M_S = 0.030
STEP_ALIGN_CREEP_SPEED_M_S = 0.015
STEP_FIXED_TAKEOVER_CONFIRM_FRAMES = 3
STEP_FIXED_TAKEOVER_MAX_DISTANCE_M = 1.05
STEP_FIXED_TAKEOVER_MIN_HEIGHT_M = 0.025
STEP_FIXED_TAKEOVER_MAX_HEIGHT_M = 0.085
STEP_FIXED_TAKEOVER_MIN_CONFIDENCE = 0.42
STEP_RACE_CANDIDATE_MIN_CONFIDENCE = 0.42
STEP_RACE_CANDIDATE_MIN_HEIGHT_M = 0.025
STEP_RACE_CANDIDATE_MAX_HEIGHT_M = 0.085
STEP_RACE_CANDIDATE_MAX_DISTANCE_M = 1.20
STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER = 2.40
STEP_RACE_CANDIDATE_EVIDENCE_MAX = 6.00
STEP_RACE_CANDIDATE_EVIDENCE_GAIN = 1.00
STEP_RACE_CANDIDATE_EVIDENCE_DECAY = 0.60
STEP_SOFT_ARM_MAX_HEADING_DELTA = 0.32
STEP_SOFT_ARM_MIN_CONFIDENCE = 0.34
STEP_SOFT_ARM_MIN_VALID_BANDS = 2
SECOND_LAP_PRESTEP_SLOW_ENABLED = True
SECOND_LAP_PRESTEP_SLOW_START_M = 1.20
SECOND_LAP_PRESTEP_SPEED_CAP = StepParams.SECOND_LAP_SPEED_CAP
STEP_FIXED_RELEASE_IF_LOST_ABOVE_M = 0.85
STEP_FIXED_RELEASE_LOST_SECONDS = 1.00
STEP_FIXED_REVERSE_RECOVERY_ENABLED = True
STEP_FIXED_REVERSE_SPEED_M_S = 0.045
STEP_FIXED_DEPTH_FILTER_SAMPLES = 5
STEP_FIXED_DEPTH_FILTER_MAX_AGE_SECONDS = 0.80
STEP_FIXED_TOO_CLOSE_CONFIRM_FRAMES = 3
STEP_FIXED_NEAR_LOSS_GRACE_SECONDS = 0.80
STEP_FIXED_REVERSE_REACQUIRE_DISTANCE_M = 0.72
STEP_FIXED_REVERSE_REACQUIRE_FRAMES = 2
STEP_FIXED_REVERSE_MIN_TARGET_M = 0.04
STEP_FIXED_REVERSE_MAX_TARGET_M = 0.14
STEP_FIXED_REVERSE_EXTRA_MARGIN_M = 0.015
STEP_FIXED_REVERSE_MAX_DISTANCE_M = 0.24
STEP_FIXED_REVERSE_MAX_SECONDS = 6.0
STEP_FIXED_YAW_HOLD_KP = 1.20
STEP_FIXED_YAW_HOLD_MAX_ANGULAR = 0.18
STEP_CALIB_CENTER_KP = 0.60
STEP_CALIB_FINAL_CENTER_ERROR_MAX = 0.08
STEP_CALIB_FINAL_NEAR_MID_DELTA_MAX = 0.08
STEP_CALIB_FINAL_CONFIRM_FRAMES = 4
STEP_CALIB_FINAL_ROTATE_KP = 1.55
STEP_CALIB_FINAL_CENTER_KP = 0.42
STEP_CALIB_FINAL_MAX_ANGULAR = 0.30
STEP_STOP_DISTANCE_M = 0.15
STEP_POSITION_TOLERANCE_M = 0.015
STEP_MANUAL_CONFIRM_BEFORE_JUMP = StepParams.MANUAL_CONFIRM_BEFORE_JUMP
STEP_APPROACH_SPEED_M_S = StepParams.APPROACH_SPEED
STEP_JUMP_TRIGGER_LEAD_M = StepParams.JUMP_TRIGGER_LEAD
STEP_TRIGGER_DISTANCE_SOURCE = "commanded"
STEP_MAX_APPROACH_SECONDS = 12.0
STEP_LOCK_HOLD_SECONDS = 0.0
STEP_USE_LOW_JUMP_PROFILE = True
STEP_JUMP_MIN_HEIGHT_M = StepParams.JUMP_MIN_HEIGHT
STEP_JUMP_MAX_HEIGHT_M = StepParams.JUMP_MAX_HEIGHT
STEP_STAND_HEIGHT_M = 0.25
STEP_JUMP_FORWARD_SPEED_M_S = StepParams.JUMP_FORWARD_SPEED
STEP_LANDING_FORWARD_SECONDS = 2.0
STEP_LANDING_SPEED_SCALE = 0.60
STEP_JUMP_PULSE_SECONDS = 0.20
STEP_RECOVER_SECONDS = 3.0
STEP_POST_REACQUIRE_ENABLED = True
STEP_POST_REACQUIRE_SECONDS = StepParams.POST_REACQUIRE_SECONDS
STEP_POST_REACQUIRE_MAX_DISTANCE_M = StepParams.POST_REACQUIRE_DISTANCE
STEP_POST_REACQUIRE_SPEED_M_S = StepParams.POST_REACQUIRE_SPEED
STEP_POST_REACQUIRE_TURN_RAD_S = StepParams.POST_REACQUIRE_TURN
STEP_TRIGGER_PHASES = ("SEEK_ROUNDABOUT", "SEEK_CROSSWALK", "SEEK_FINISH")
STEP_ARM_SEEK_WINDOW_ENABLED = True
STEP_ARM_MIN_SEEK_DISTANCE_M = StepParams.ARM_MIN_SEEK_DISTANCE
STEP_ARM_MAX_SEEK_DISTANCE_M = StepParams.ARM_MAX_SEEK_DISTANCE
STEP_ARM_MAX_MID_FAR_DELTA = 0.18
STEP_ARM_MAX_NEAR_FAR_DELTA = 0.30
STEP_ARM_MAX_CENTER_ERROR = 0.18
STEP_ARM_MAX_LAST_ANGULAR = 0.22
STEP_FORCE_ALIGN_ON_CONFIRMED_DISTANCE_M = 1.05
STEP_DEPTH_POSE_JUMP_POINT_DISTANCE_M = 0.90
STEP_DEPTH_ALIGN_MAX_EDGE_ANGLE_RAD = 0.14
STEP_DEPTH_ALIGN_MAX_LATERAL_OFFSET_M = 0.08
STEP_DEPTH_ALIGN_CONFIRM_FRAMES = 3
STEP_DEPTH_ALIGN_EDGE_KP = 1.35
STEP_DEPTH_ALIGN_LATERAL_KP = 1.10
STEP_DEPTH_ALIGN_MAX_ANGULAR = 0.30
STEP_SOFT_ARM_MAX_MID_FAR_DELTA = 0.26
STEP_SOFT_ARM_MAX_NEAR_FAR_DELTA = 0.42
STEP_SOFT_ARM_MAX_CENTER_ERROR = 0.24
STEP_SOFT_ARM_MAX_LAST_ANGULAR = 0.30

# ------------------------------ 内部识别参数 --------------------------------
# 下面是图像识别阈值，巡线已稳定时不要现场调。
BLUE_H_MIN = 76
BLUE_H_MAX = 146
BLUE_S_MIN = 32
BLUE_V_MIN = 12
BLUE_GUARD_DILATE_PIXELS = 7

ADAPTIVE_MAX_V = 145
ADAPTIVE_BLOCK_SIZE = 41
ADAPTIVE_C = 8.0
CLAHE_CLIP_LIMIT = 2.0

STRONG_BLACK_V_MAX = 78
MASK_GROW_ITERATIONS = 10

MASK_COLOR_REJECT_S = 120
COLOR_REJECT_S = 145
COLOR_REJECT_MIN_V = 48

TARGET_X_OFFSET_RATIO = 0.00
CURVE_TARGET_X_OFFSET_RATIO = -0.09
CURVE_OFFSET_REDUCTION_START = 0.10
CURVE_OFFSET_REDUCTION_FULL = 0.32

CURVE_CENTER_RIDING_ENABLED = True
CURVE_CENTER_SHAPE_THRESHOLD = 0.12
CURVE_CENTER_NEAR_WEIGHT = 0.96
CURVE_CENTER_MID_WEIGHT = 0.04
CURVE_CENTER_HEADING_GAIN = 0.02
CURVE_OPPOSITE_NEAR_WEIGHT = 0.98
LINE_EDGE_RESCUE_SPEED_CAP = 0.085
LINE_EDGE_RESCUE_MAX_ANGULAR = 0.42
LINE_HARD_TURN_SPEED_CAP = 0.105
LINE_HARD_TURN_MAX_ANGULAR = 0.56

CURVE_CENTER_SPEED_CAP = 0.17
CURVE_CENTER_STRONG_SPEED_CAP = 0.145
CURVE_CENTER_STRONG_SHAPE = 0.28

LOOKAHEAD_Y_RATIOS: Sequence[float] = (0.93, 0.84, 0.75, 0.66, 0.57, 0.48, 0.39, 0.30)
LOOKAHEAD_BAND_HALF_HEIGHT = 6
SEARCH_LEFT_RATIO = 0.035
SEARCH_RIGHT_RATIO = 0.965
MIN_RUN_WIDTH_RATIO = 0.006
MAX_RUN_WIDTH_RATIO = 0.19
BAND_OCCUPANCY_THRESHOLD = 0.24

CANDIDATE_ABSOLUTE_V_MAX = 122
CANDIDATE_MIN_CONTRAST = 9

NORMAL_MIN_VALID_BANDS = 2
NORMAL_MIN_CONFIDENCE = 0.26

PATH_MAX_STEP_RATIO = 0.24
PATH_MAX_PREDICT_ERROR_RATIO = 0.20
PATH_MAX_MISSED_BANDS = 2
PATH_TEMPORAL_WEIGHT = 0.16
HARD_TURN_NEAR_THRESHOLD = 0.52
HARD_TURN_OPPOSITE_REJECT = 0.24
PATH_DARKNESS_WEIGHT = 0.55
PATH_TEMPORAL_WEIGHT_ROUNDABOUT = 0.62
PATH_ROUNDABOUT_MAX_FRAME_JUMP_RATIO = 0.19
PATH_TEMPORAL_SMOOTH_ALPHA_NORMAL = 0.78
PATH_TEMPORAL_SMOOTH_ALPHA_ROUNDABOUT = 0.56

NEAR_WEIGHT = 0.54
MID_WEIGHT = 0.29
FAR_WEIGHT = 0.17
HEADING_GAIN = 0.24

EVENT_DARK_V_MAX = 75

YELLOW_H_MIN = 14
YELLOW_H_MAX = 42
YELLOW_S_MIN = 70
YELLOW_V_MIN = 75

STATUS_LOG_PERIOD = 0.8


def _read_param(node: Node, name: str, default):
    node.declare_parameter(name, default)
    return node.get_parameter(name).value


def _read_bool_param(node: Node, name: str, default: bool) -> bool:
    value = _read_param(node, name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def apply_ros_parameters(node: Node) -> None:
    """Load field-tuning parameters from ROS YAML before the main logic starts."""
    global CAMERA_BACKEND, CAMERA_ID, CAMERA_DEVICE, IMAGE_WIDTH, IMAGE_HEIGHT
    global CAPTURE_FPS, PIXEL_FORMAT, ROTATION_DEGREES, MIRROR_MODE, CONTROL_RATE
    global SHOW_DEBUG, ENABLE_MOTION, RECORD_RAW_VIDEO, RECORD_DEBUG_VIDEO
    global FORWARD_SPEED, MINIMUM_FORWARD_SPEED, MAX_ANGULAR_SPEED, CURVE_SLOWDOWN
    global MAX_LINEAR_ACCELERATION, MAX_ANGULAR_ACCELERATION
    global BIG_TURN_LOSS_SECONDS, BIG_TURN_LOSS_SPEED, BIG_TURN_LOSS_TURN
    global ROUNDABOUT_FORWARD_SPEED, ROUNDABOUT_ENTRY_LEFT_ANGULAR
    global ROUNDABOUT_ENTRY_TURN_DISTANCE, ROUNDABOUT_SECOND_LAP_USE_FIRST_POSITION
    global ROUNDABOUT_SECOND_LAP_ADVANCE_M, ROUNDABOUT_ENTRY_FALLBACK_SECONDS
    global ROUNDABOUT_EXIT_SPEED, ROUNDABOUT_FIRST_LAP_APPROACH_DISTANCE
    global ROUNDABOUT_SECOND_LAP_APPROACH_DISTANCE, ROUNDABOUT_GATE_MIN_TRAVEL
    global ROUNDABOUT_FOLLOW_SECONDS, ROUNDABOUT_EXIT_COMMIT_SECONDS
    global ROUNDABOUT_BLIND_RECOVERY_SPEED, ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR
    global ROUNDABOUT_BLUE_GUARD_SPEED, ROUNDABOUT_BLUE_GUARD_ANGULAR
    global CROSSWALK_APPROACH_DISTANCE, CROSSWALK_STOP_SECONDS
    global CROSSWALK_CONFIRM_FRAMES, CROSSWALK_DETECT_DELAY_AFTER_ROUNDABOUT
    global YELLOW_FINISH_CONFIRM_FRAMES, FINISH_DETECT_DELAY_AFTER_CROSSWALK
    global YELLOW_ACCEPT_WHILE_SEEK_CROSSWALK, TOTAL_LAPS, FINISH_PASS_DISTANCE
    global STEP_MANUAL_CONFIRM_BEFORE_JUMP, STEP_APPROACH_SPEED_M_S
    global STEP_JUMP_MIN_HEIGHT_M, STEP_JUMP_MAX_HEIGHT_M
    global STEP_JUMP_FORWARD_SPEED_M_S, STEP_FIXED_SPRINT_DISTANCE_M
    global STEP_JUMP_TRIGGER_LEAD_M, STEP_CALIB_TARGET_DEPTH_M
    global STEP_FIXED_SPRINT_START_DEPTH_M, STEP_ARM_MIN_SEEK_DISTANCE_M
    global STEP_ARM_MAX_SEEK_DISTANCE_M, SECOND_LAP_PRESTEP_SPEED_CAP
    global STEP_POST_REACQUIRE_SECONDS, STEP_POST_REACQUIRE_MAX_DISTANCE_M
    global STEP_POST_REACQUIRE_SPEED_M_S, STEP_POST_REACQUIRE_TURN_RAD_S

    ENABLE_MOTION = _read_bool_param(node, "runtime.enable_motion", ENABLE_MOTION)
    SHOW_DEBUG = _read_bool_param(node, "runtime.show_debug", SHOW_DEBUG)
    RECORD_RAW_VIDEO = _read_bool_param(
        node, "runtime.record_raw_video", RECORD_RAW_VIDEO
    )
    RECORD_DEBUG_VIDEO = _read_bool_param(
        node, "runtime.record_debug_video", RECORD_DEBUG_VIDEO
    )

    CAMERA_BACKEND = str(_read_param(node, "camera.backend", CAMERA_BACKEND))
    CAMERA_ID = int(_read_param(node, "camera.id", CAMERA_ID))
    CAMERA_DEVICE = str(_read_param(node, "camera.device", CAMERA_DEVICE))
    IMAGE_WIDTH = int(_read_param(node, "camera.width", IMAGE_WIDTH))
    IMAGE_HEIGHT = int(_read_param(node, "camera.height", IMAGE_HEIGHT))
    CAPTURE_FPS = float(_read_param(node, "camera.fps", CAPTURE_FPS))
    PIXEL_FORMAT = str(_read_param(node, "camera.pixel_format", PIXEL_FORMAT))
    ROTATION_DEGREES = int(
        _read_param(node, "camera.rotation_degrees", ROTATION_DEGREES)
    )
    MIRROR_MODE = str(_read_param(node, "camera.mirror_mode", MIRROR_MODE))
    CONTROL_RATE = float(_read_param(node, "camera.control_rate", CONTROL_RATE))

    FORWARD_SPEED = float(_read_param(node, "drive.forward_speed", FORWARD_SPEED))
    MINIMUM_FORWARD_SPEED = float(
        _read_param(node, "drive.min_forward_speed", MINIMUM_FORWARD_SPEED)
    )
    MAX_ANGULAR_SPEED = float(
        _read_param(node, "drive.max_turn_speed", MAX_ANGULAR_SPEED)
    )
    CURVE_SLOWDOWN = float(_read_param(node, "drive.curve_slowdown", CURVE_SLOWDOWN))
    MAX_LINEAR_ACCELERATION = float(
        _read_param(node, "drive.linear_accel", MAX_LINEAR_ACCELERATION)
    )
    MAX_ANGULAR_ACCELERATION = float(
        _read_param(node, "drive.angular_accel", MAX_ANGULAR_ACCELERATION)
    )
    BIG_TURN_LOSS_SECONDS = float(
        _read_param(node, "drive.big_turn_loss_seconds", BIG_TURN_LOSS_SECONDS)
    )
    BIG_TURN_LOSS_SPEED = float(
        _read_param(node, "drive.big_turn_loss_speed", BIG_TURN_LOSS_SPEED)
    )
    BIG_TURN_LOSS_TURN = float(
        _read_param(node, "drive.big_turn_loss_turn", BIG_TURN_LOSS_TURN)
    )

    ROUNDABOUT_FORWARD_SPEED = float(
        _read_param(node, "roundabout.forward_speed", ROUNDABOUT_FORWARD_SPEED)
    )
    ROUNDABOUT_ENTRY_LEFT_ANGULAR = float(
        _read_param(
            node, "roundabout.left_turn_speed", ROUNDABOUT_ENTRY_LEFT_ANGULAR
        )
    )
    ROUNDABOUT_GATE_MIN_TRAVEL = float(
        _read_param(
            node, "roundabout.arm_after_start_distance", ROUNDABOUT_GATE_MIN_TRAVEL
        )
    )
    ROUNDABOUT_FIRST_LAP_APPROACH_DISTANCE = float(
        _read_param(
            node,
            "roundabout.first_lap_approach_distance",
            ROUNDABOUT_FIRST_LAP_APPROACH_DISTANCE,
        )
    )
    ROUNDABOUT_SECOND_LAP_APPROACH_DISTANCE = float(
        _read_param(
            node,
            "roundabout.second_lap_approach_distance",
            ROUNDABOUT_SECOND_LAP_APPROACH_DISTANCE,
        )
    )
    ROUNDABOUT_SECOND_LAP_USE_FIRST_POSITION = _read_bool_param(
        node,
        "roundabout.second_lap_use_first_position",
        ROUNDABOUT_SECOND_LAP_USE_FIRST_POSITION,
    )
    ROUNDABOUT_SECOND_LAP_ADVANCE_M = float(
        _read_param(
            node, "roundabout.second_lap_advance", ROUNDABOUT_SECOND_LAP_ADVANCE_M
        )
    )
    ROUNDABOUT_ENTRY_TURN_DISTANCE = float(
        _read_param(
            node, "roundabout.entry_turn_distance", ROUNDABOUT_ENTRY_TURN_DISTANCE
        )
    )
    ROUNDABOUT_ENTRY_FALLBACK_SECONDS = float(
        _read_param(
            node, "roundabout.entry_max_seconds", ROUNDABOUT_ENTRY_FALLBACK_SECONDS
        )
    )
    ROUNDABOUT_FOLLOW_SECONDS = float(
        _read_param(node, "roundabout.follow_max_seconds", ROUNDABOUT_FOLLOW_SECONDS)
    )
    ROUNDABOUT_EXIT_COMMIT_SECONDS = float(
        _read_param(
            node, "roundabout.exit_commit_seconds", ROUNDABOUT_EXIT_COMMIT_SECONDS
        )
    )
    ROUNDABOUT_EXIT_SPEED = float(
        _read_param(node, "roundabout.exit_speed", ROUNDABOUT_EXIT_SPEED)
    )
    ROUNDABOUT_BLIND_RECOVERY_SPEED = float(
        _read_param(
            node, "roundabout.blind_recovery_speed", ROUNDABOUT_BLIND_RECOVERY_SPEED
        )
    )
    ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR = float(
        _read_param(
            node,
            "roundabout.blind_recovery_left_turn",
            ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR,
        )
    )
    ROUNDABOUT_BLUE_GUARD_SPEED = float(
        _read_param(node, "roundabout.blue_guard_speed", ROUNDABOUT_BLUE_GUARD_SPEED)
    )
    ROUNDABOUT_BLUE_GUARD_ANGULAR = float(
        _read_param(
            node, "roundabout.blue_guard_turn_speed", ROUNDABOUT_BLUE_GUARD_ANGULAR
        )
    )

    CROSSWALK_CONFIRM_FRAMES = int(
        _read_param(node, "crosswalk.confirm_frames", CROSSWALK_CONFIRM_FRAMES)
    )
    CROSSWALK_APPROACH_DISTANCE = float(
        _read_param(node, "crosswalk.approach_distance", CROSSWALK_APPROACH_DISTANCE)
    )
    CROSSWALK_STOP_SECONDS = float(
        _read_param(node, "crosswalk.stop_seconds", CROSSWALK_STOP_SECONDS)
    )
    CROSSWALK_DETECT_DELAY_AFTER_ROUNDABOUT = float(
        _read_param(
            node,
            "crosswalk.detect_delay_after_roundabout",
            CROSSWALK_DETECT_DELAY_AFTER_ROUNDABOUT,
        )
    )

    YELLOW_FINISH_CONFIRM_FRAMES = int(
        _read_param(node, "yellow_line.confirm_frames", YELLOW_FINISH_CONFIRM_FRAMES)
    )
    FINISH_DETECT_DELAY_AFTER_CROSSWALK = float(
        _read_param(
            node,
            "yellow_line.detect_delay_after_crosswalk",
            FINISH_DETECT_DELAY_AFTER_CROSSWALK,
        )
    )
    YELLOW_ACCEPT_WHILE_SEEK_CROSSWALK = _read_bool_param(
        node,
        "yellow_line.accept_while_seek_crosswalk",
        YELLOW_ACCEPT_WHILE_SEEK_CROSSWALK,
    )
    TOTAL_LAPS = int(_read_param(node, "yellow_line.total_laps", TOTAL_LAPS))
    FINISH_PASS_DISTANCE = float(
        _read_param(node, "yellow_line.finish_pass_distance", FINISH_PASS_DISTANCE)
    )

    STEP_MANUAL_CONFIRM_BEFORE_JUMP = _read_bool_param(
        node, "step.manual_confirm_before_jump", STEP_MANUAL_CONFIRM_BEFORE_JUMP
    )
    STEP_APPROACH_SPEED_M_S = float(
        _read_param(node, "step.approach_speed", STEP_APPROACH_SPEED_M_S)
    )
    STEP_JUMP_FORWARD_SPEED_M_S = float(
        _read_param(node, "step.jump_forward_speed", STEP_JUMP_FORWARD_SPEED_M_S)
    )
    STEP_JUMP_MIN_HEIGHT_M = float(
        _read_param(node, "step.jump_min_height", STEP_JUMP_MIN_HEIGHT_M)
    )
    STEP_JUMP_MAX_HEIGHT_M = float(
        _read_param(node, "step.jump_max_height", STEP_JUMP_MAX_HEIGHT_M)
    )
    STEP_FIXED_SPRINT_DISTANCE_M = float(
        _read_param(node, "step.sprint_distance", STEP_FIXED_SPRINT_DISTANCE_M)
    )
    STEP_JUMP_TRIGGER_LEAD_M = float(
        _read_param(node, "step.jump_trigger_lead", STEP_JUMP_TRIGGER_LEAD_M)
    )
    step_start_depth = float(
        _read_param(node, "step.start_depth", STEP_FIXED_SPRINT_START_DEPTH_M)
    )
    STEP_CALIB_TARGET_DEPTH_M = step_start_depth
    STEP_FIXED_SPRINT_START_DEPTH_M = step_start_depth
    STEP_ARM_MIN_SEEK_DISTANCE_M = float(
        _read_param(node, "step.arm_min_seek_distance", STEP_ARM_MIN_SEEK_DISTANCE_M)
    )
    STEP_ARM_MAX_SEEK_DISTANCE_M = float(
        _read_param(node, "step.arm_max_seek_distance", STEP_ARM_MAX_SEEK_DISTANCE_M)
    )
    SECOND_LAP_PRESTEP_SPEED_CAP = float(
        _read_param(node, "step.second_lap_speed_cap", SECOND_LAP_PRESTEP_SPEED_CAP)
    )
    STEP_POST_REACQUIRE_SECONDS = float(
        _read_param(node, "step.post_reacquire_seconds", STEP_POST_REACQUIRE_SECONDS)
    )
    STEP_POST_REACQUIRE_MAX_DISTANCE_M = float(
        _read_param(
            node, "step.post_reacquire_distance", STEP_POST_REACQUIRE_MAX_DISTANCE_M
        )
    )
    STEP_POST_REACQUIRE_SPEED_M_S = float(
        _read_param(node, "step.post_reacquire_speed", STEP_POST_REACQUIRE_SPEED_M_S)
    )
    STEP_POST_REACQUIRE_TURN_RAD_S = float(
        _read_param(node, "step.post_reacquire_turn", STEP_POST_REACQUIRE_TURN_RAD_S)
    )


@dataclass
class VisionResult:
    line_valid: bool
    error: float
    confidence: float
    near_error: float
    mid_error: float
    far_error: float
    valid_bands: int
    solid_score: float
    roundabout_gate: bool
    roundabout_gate_score: float
    roundabout_exit_solid: bool
    roundabout_exit_score: float
    crosswalk: bool
    crosswalk_score: float
    yellow_finish: bool
    yellow_score: float
    debug_frame: np.ndarray
    reason: str


def clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


_FONT_CACHE = {}


PHASE_TEXT = {
    "SEEK_ROUNDABOUT": "找环岛/台阶",
    "ROUNDABOUT_APPROACH": "接近环岛",
    "ROUNDABOUT": "进环岛",
    "ROUNDABOUT_FOLLOW": "环岛内",
    "ROUNDABOUT_EXIT": "出环岛",
    "SEEK_CROSSWALK": "找人行道",
    "CROSSWALK_APPROACH": "接近人行道",
    "CROSSWALK_STOP": "人行道停车",
    "SEEK_FINISH": "找终点黄线",
    "FINISH_PASS": "冲过终点",
    "FINISHED": "完成",
}

STEP_TEXT = {
    "IDLE": "未接管",
    "LOCKED": "已锁定",
    "APPROACH": "冲刺中",
    "JUMP_CONFIRM": "等回车",
    "JUMP": "跳跃中",
    "LAND": "落地前行",
    "RECOVER": "恢复中",
    "STOPPED": "已停车",
    "CALIB_HOLD": "标定停车",
}


def phase_text(phase: str) -> str:
    return PHASE_TEXT.get(phase, phase)


def step_text(state: str) -> str:
    return STEP_TEXT.get(state, state)


def _font(size: int):
    if ImageFont is None:
        return None
    if size in _FONT_CACHE:
        return _FONT_CACHE[size]
    for path in (
        "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    ):
        if Path(path).exists():
            _FONT_CACHE[size] = ImageFont.truetype(path, size)
            return _FONT_CACHE[size]
    _FONT_CACHE[size] = ImageFont.load_default()
    return _FONT_CACHE[size]


def draw_overlay(
    frame: np.ndarray,
    lines: Sequence[Tuple[str, Tuple[int, int, int]]],
    *,
    origin: Tuple[int, int] = (10, 10),
    font_size: int = 19,
    line_height: int = 25,
) -> np.ndarray:
    """Draw a compact Chinese overlay. Colors are BGR."""
    if Image is None or ImageDraw is None or ImageFont is None:
        x, y = origin
        for text, color in lines:
            cv2.putText(
                frame,
                text,
                (x, y + font_size),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                1,
                cv2.LINE_AA,
            )
            y += line_height
        return frame

    x, y = origin
    width = frame.shape[1]
    height = min(frame.shape[0], y + line_height * len(lines) + 10)
    pil = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil)
    draw.rectangle((0, 0, width, height), fill=(0, 0, 0))
    font = _font(font_size)
    for index, (text, bgr) in enumerate(lines):
        b, g, r = bgr
        draw.text((x, y + index * line_height), text, font=font, fill=(r, g, b))
    return cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class CourseVision:
    """一次完成二值巡线、环岛入口、人行横道、黄色终点的视觉处理。"""

    def __init__(self) -> None:
        self._clahe = cv2.createCLAHE(
            clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=(8, 8)
        )
        self._previous_band_x: List[Optional[float]] = [
            None for _ in LOOKAHEAD_Y_RATIOS
        ]
        self._last_good_band_x: List[Optional[float]] = [
            None for _ in LOOKAHEAD_Y_RATIOS
        ]
        self._effective_target_offset_ratio = TARGET_X_OFFSET_RATIO

        # V1.45 环岛边界安全层调试量。
        self._roundabout_blue_guard_active = False
        self._roundabout_blue_guard_turn_sign = 0.0   # +1=左转, -1=右转
        self._roundabout_blue_guard_blue_fraction = 0.0
        self._roundabout_blue_guard_white_left = 0.0
        self._roundabout_blue_guard_white_right = 0.0

    def reset_tracking(self) -> None:
        self._previous_band_x = [None for _ in LOOKAHEAD_Y_RATIOS]

    @staticmethod
    def _runs(binary_row: np.ndarray) -> List[Tuple[int, int]]:
        padded = np.pad((binary_row > 0).astype(np.int8), (1, 1))
        diff = np.diff(padded)
        starts = np.flatnonzero(diff == 1)
        ends = np.flatnonzero(diff == -1)
        return list(zip(starts.tolist(), ends.tolist()))

    def _make_dark_mask(
        self, frame: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """V1.19：强黑核心 + 弱候选连通生长。

        旧版的主要问题是 adaptive 阈值会把白底上的阴影、褶皱也变成白色候选；
        同时白蓝交界处的暗边缘可能落在蓝色 HSV 阈值之外。

        新流程：
        1) 先找蓝色区域并向白区扩一圈保护带，直接排除白蓝交界；
        2) 自适应阈值只生成“弱候选”；
        3) V 足够低的真实黑色生成“强黑核心”；
        4) 只保留能从强黑核心沿弱候选连续生长到的区域。
        这样阴影即使被 adaptive 选中，只要没有真正黑核心，就不会进入主二值图。
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        blue_core = (
            (hue >= BLUE_H_MIN)
            & (hue <= BLUE_H_MAX)
            & (sat >= BLUE_S_MIN)
            & (val >= BLUE_V_MIN)
        ).astype(np.uint8) * 255
        blue_core = cv2.morphologyEx(
            blue_core,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), dtype=np.uint8),
            iterations=1,
        )
        guard_k = 2 * BLUE_GUARD_DILATE_PIXELS + 1
        blue = cv2.dilate(
            blue_core,
            np.ones((guard_k, guard_k), dtype=np.uint8),
            iterations=1,
        )

        normalized = self._clahe.apply(val)
        adaptive = cv2.adaptiveThreshold(
            normalized,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            ADAPTIVE_BLOCK_SIZE,
            ADAPTIVE_C,
        )

        weak = (
            (adaptive > 0)
            & (val <= ADAPTIVE_MAX_V)
            & ((sat <= MASK_COLOR_REJECT_S) | (val <= COLOR_REJECT_MIN_V))
        ).astype(np.uint8) * 255
        weak[blue > 0] = 0

        # 真实黑胶带的“种子”。低亮度时饱和度不可靠，因此这里只看 V，
        # 但仍然排除已经确认的蓝色保护区。
        strong = (val <= STRONG_BLACK_V_MAX).astype(np.uint8) * 255
        strong[blue > 0] = 0
        strong = cv2.morphologyEx(
            strong, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
        )
        strong = cv2.morphologyEx(
            strong, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1
        )

        # 双阈值滞后连接：只有与强黑核心连接的弱候选才留下。
        allowed = cv2.bitwise_or(weak, strong)
        dark = strong.copy()
        grow_kernel = np.ones((3, 3), dtype=np.uint8)
        for _ in range(MASK_GROW_ITERATIONS):
            grown = cv2.dilate(dark, grow_kernel, iterations=1)
            new_dark = cv2.bitwise_and(grown, allowed)
            if np.array_equal(new_dark, dark):
                break
            dark = new_dark

        dark = cv2.morphologyEx(
            dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
        )
        dark = cv2.morphologyEx(
            dark, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=1
        )

        # 环岛虚线/斑马线继续使用严格强黑事件图，不用 adaptive 阴影候选。
        event_dark = (val <= EVENT_DARK_V_MAX).astype(np.uint8) * 255
        event_dark[blue > 0] = 0
        event_dark = cv2.morphologyEx(
            event_dark, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1
        )
        event_dark = cv2.morphologyEx(
            event_dark, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), iterations=1
        )
        return dark, event_dark, hsv, blue

    def _update_roundabout_blue_guard(
        self,
        hsv: np.ndarray,
        roundabout: bool,
    ) -> None:
        """判断车头是否正在逼近蓝色赛道边界，并判断白色赛道位于哪一侧。

        这里不把蓝边当成巡线目标，只作为安全护栏。
        典型危险画面：前方蓝带横穿视野，白色赛道明显集中在一侧，
        而另一侧已经是红/棕色赛道外区域。
        """
        self._roundabout_blue_guard_active = False
        self._roundabout_blue_guard_turn_sign = 0.0
        self._roundabout_blue_guard_blue_fraction = 0.0
        self._roundabout_blue_guard_white_left = 0.0
        self._roundabout_blue_guard_white_right = 0.0

        if not (ROUNDABOUT_BLUE_GUARD_ENABLED and roundabout):
            return

        h, w = hsv.shape[:2]
        hue = hsv[:, :, 0]
        sat = hsv[:, :, 1]
        val = hsv[:, :, 2]

        # 使用未膨胀的真实蓝色核心，避免蓝色保护带把面积人为放大。
        blue_core = (
            (hue >= BLUE_H_MIN)
            & (hue <= BLUE_H_MAX)
            & (sat >= BLUE_S_MIN)
            & (val >= BLUE_V_MIN)
        )

        fy0 = int(ROUNDABOUT_BLUE_GUARD_FORWARD_Y_TOP * h)
        fy1 = int(ROUNDABOUT_BLUE_GUARD_FORWARD_Y_BOTTOM * h)
        fx0 = int(0.08 * w)
        fx1 = int(0.92 * w)
        forward_blue = blue_core[fy0:fy1, fx0:fx1]
        blue_fraction = (
            float(np.mean(forward_blue))
            if forward_blue.size
            else 0.0
        )

        white = (
            (sat <= ROUNDABOUT_BLUE_GUARD_WHITE_S_MAX)
            & (val >= ROUNDABOUT_BLUE_GUARD_WHITE_V_MIN)
        )
        wy0 = int(ROUNDABOUT_BLUE_GUARD_WHITE_Y_TOP * h)
        wy1 = int(ROUNDABOUT_BLUE_GUARD_WHITE_Y_BOTTOM * h)
        white_roi = white[wy0:wy1, :]
        half = w // 2

        if white_roi.size:
            white_left = float(np.mean(white_roi[:, :half]))
            white_right = float(np.mean(white_roi[:, half:]))
        else:
            white_left = 0.0
            white_right = 0.0

        self._roundabout_blue_guard_blue_fraction = blue_fraction
        self._roundabout_blue_guard_white_left = white_left
        self._roundabout_blue_guard_white_right = white_right

        white_best = max(white_left, white_right)
        white_gap = abs(white_left - white_right)

        # 必须同时满足“前方蓝边明显 + 白色赛道明显偏在一侧”，
        # 避免正常环岛中仅仅看到侧边蓝色就误触发。
        if (
            blue_fraction >= ROUNDABOUT_BLUE_GUARD_MIN_BLUE_FRACTION
            and white_best >= ROUNDABOUT_BLUE_GUARD_MIN_WHITE_SIDE
            and white_gap >= ROUNDABOUT_BLUE_GUARD_MIN_WHITE_IMBALANCE
        ):
            self._roundabout_blue_guard_active = True
            self._roundabout_blue_guard_turn_sign = (
                1.0 if white_left > white_right else -1.0
            )

    def _candidate_valid(
        self,
        hsv: np.ndarray,
        x0: int,
        x1: int,
        y0: int,
        y1: int,
    ) -> bool:
        h, w = hsv.shape[:2]
        x0 = max(0, min(w - 1, x0))
        x1 = max(x0 + 1, min(w, x1))
        y0 = max(0, min(h - 1, y0))
        y1 = max(y0 + 1, min(h, y1))
        candidate = hsv[y0:y1, x0:x1]
        if candidate.size == 0:
            return False

        med_v = float(np.median(candidate[:, :, 2]))
        med_s = float(np.median(candidate[:, :, 1]))
        strong_fraction = float(
            np.mean(candidate[:, :, 2] <= STRONG_BLACK_V_MAX)
        )
        if med_v > COLOR_REJECT_MIN_V and med_s > COLOR_REJECT_S:
            return False
        # 阴影往往只是“相对暗”，却没有足够的真正黑核心。
        if strong_fraction < 0.06 and med_v > 105:
            return False

        pad = max(7, min(26, 2 * (x1 - x0)))
        left = hsv[y0:y1, max(0, x0 - pad):x0]
        right = hsv[y0:y1, x1:min(w, x1 + pad)]
        flank_values = []
        if left.size:
            flank_values.append(float(np.median(left[:, :, 2])))
        if right.size:
            flank_values.append(float(np.median(right[:, :, 2])))
        flank_v = float(np.mean(flank_values)) if flank_values else med_v
        contrast = flank_v - med_v

        # 白蓝交界常表现成一条很暗的细边；只要候选一侧明显是蓝区，就不把它当赛道黑线。
        def blue_fraction(pixels: np.ndarray) -> float:
            if pixels.size == 0:
                return 0.0
            ph = pixels[:, :, 0]
            ps = pixels[:, :, 1]
            pv = pixels[:, :, 2]
            blue_pixels = (
                (ph >= BLUE_H_MIN)
                & (ph <= BLUE_H_MAX)
                & (ps >= BLUE_S_MIN)
                & (pv >= BLUE_V_MIN)
            )
            return float(np.mean(blue_pixels))

        if blue_fraction(left) >= 0.20 or blue_fraction(right) >= 0.20:
            return False

        return med_v <= CANDIDATE_ABSOLUTE_V_MAX or contrast >= CANDIDATE_MIN_CONTRAST

    def _extract_line(
        self,
        frame: np.ndarray,
        dark: np.ndarray,
        hsv: np.ndarray,
        roundabout: bool,
        use_dynamic_offset: bool = True,
    ) -> Tuple[
        bool,
        float,
        float,
        float,
        float,
        int,
        float,
        List[Optional[float]],
        str,
    ]:
        """从机器人近场开始向远场连续追踪黑线。

        V1.2 的关键变化：每一个更远的候选必须与已经确认的近场路径连续。
        一旦连续丢失若干层就停止向远处搜索，绝不把另一侧的孤立黑块拼进来。
        这专门解决第一个 90° 弯中“左下真线 + 右上假目标”被拟合在一起的问题。
        """
        h, w = dark.shape
        x_left = int(w * SEARCH_LEFT_RATIO)
        x_right = int(w * SEARCH_RIGHT_RATIO)
        min_width = max(2, int(w * MIN_RUN_WIDTH_RATIO))
        max_width = max(min_width + 1, int(w * MAX_RUN_WIDTH_RATIO))

        selected: List[Optional[float]] = [None for _ in LOOKAHEAD_Y_RATIOS]
        selected_widths: List[Optional[float]] = [None for _ in LOOKAHEAD_Y_RATIOS]
        chosen_indices: List[int] = []
        miss_streak = 0

        for i, ratio in enumerate(LOOKAHEAD_Y_RATIOS):
            y = int(ratio * h)
            y0 = max(0, y - LOOKAHEAD_BAND_HALF_HEIGHT)
            y1 = min(h, y + LOOKAHEAD_BAND_HALF_HEIGHT + 1)
            band = dark[y0:y1, x_left:x_right]
            if band.size == 0:
                continue

            occupancy = np.mean(band > 0, axis=0)
            binary = (occupancy >= BAND_OCCUPANCY_THRESHOLD).astype(np.uint8)
            candidates: List[Tuple[float, float, float]] = []
            for start, end in self._runs(binary):
                width = end - start
                if not min_width <= width <= max_width:
                    continue
                gx0 = x_left + start
                gx1 = x_left + end
                if not self._candidate_valid(hsv, gx0, gx1, y0, y1):
                    continue
                candidate_v = float(np.median(hsv[y0:y1, gx0:gx1, 2]))
                candidates.append((0.5 * (gx0 + gx1 - 1), float(width), candidate_v))

            temporal = self._previous_band_x[i]
            if temporal is None:
                temporal = self._last_good_band_x[i]

            if not chosen_indices:
                if not candidates:
                    # 最靠近车体的前两层允许缺失，但不能一路跳到很远才随便认目标。
                    miss_streak += 1
                    if miss_streak > 2:
                        break
                    continue
                # 第一锚点：时间信息仅作弱约束。急弯时真实黑线可以快速从中心移到边缘。
                reference = temporal if temporal is not None else 0.5 * w
                def anchor_score(item: Tuple[float, float, float]) -> float:
                    center, width, candidate_v = item
                    temporal_cost = abs(center - reference) / max(1.0, 0.5 * w)
                    expected_width = max(4.0, 0.022 * w * (0.70 + 0.80 * ratio))
                    width_cost = abs(width - expected_width) / max(6.0, expected_width)
                    darkness_cost = clamp(candidate_v / 135.0, 0.0, 1.0)
                    # 急弯时黑胶带在单个横带里会显得很宽，线宽只能作弱约束；
                    # 地板接缝通常亮得多，因此优先选择真正更黑的候选。
                    return (
                        PATH_DARKNESS_WEIGHT * darkness_cost
                        + PATH_TEMPORAL_WEIGHT * temporal_cost
                        + 0.025 * width_cost
                    )
                if roundabout and temporal is not None:
                    temporal_candidates = [
                        item for item in candidates
                        if abs(item[0] - temporal)
                        <= PATH_ROUNDABOUT_MAX_FRAME_JUMP_RATIO * w
                    ]
                    if temporal_candidates:
                        candidates = temporal_candidates
                center, width, _candidate_v = min(candidates, key=anchor_score)
                selected[i] = center
                selected_widths[i] = width
                chosen_indices.append(i)
                miss_streak = 0
                continue

            last_i = chosen_indices[-1]
            last_x = float(selected[last_i])
            gap = max(1, i - last_i)
            if len(chosen_indices) >= 2:
                prev_i = chosen_indices[-2]
                prev_x = float(selected[prev_i])
                slope_per_band = (last_x - prev_x) / max(1, last_i - prev_i)
                predicted = last_x + slope_per_band * gap
            else:
                predicted = last_x

            # 允许随跳过的 band 略微放宽，但绝不允许一次跨到画面另一侧。
            max_step = PATH_MAX_STEP_RATIO * w * (1.0 + 0.32 * (gap - 1))
            max_predict_error = PATH_MAX_PREDICT_ERROR_RATIO * w * (1.0 + 0.28 * (gap - 1))
            compatible: List[Tuple[float, float, float]] = []
            for center, width, candidate_v in candidates:
                if abs(center - last_x) > max_step:
                    continue
                if len(chosen_indices) >= 2 and abs(center - predicted) > max_predict_error:
                    continue
                compatible.append((center, width, candidate_v))

            if not compatible:
                miss_streak += 1
                if miss_streak > PATH_MAX_MISSED_BANDS:
                    break
                continue

            def continuation_score(item: Tuple[float, float, float]) -> float:
                center, width, candidate_v = item
                spatial_cost = abs(center - predicted) / max(1.0, max_predict_error)
                last_cost = abs(center - last_x) / max(1.0, max_step)
                temporal_cost = 0.0
                if temporal is not None:
                    temporal_cost = abs(center - temporal) / max(1.0, 0.40 * w)
                expected_width = max(4.0, 0.022 * w * (0.70 + 0.80 * ratio))
                width_cost = abs(width - expected_width) / max(6.0, expected_width)
                darkness_cost = clamp(candidate_v / 135.0, 0.0, 1.0)
                return (
                    0.48 * spatial_cost
                    + 0.24 * last_cost
                    + 0.14 * darkness_cost
                    + (PATH_TEMPORAL_WEIGHT_ROUNDABOUT if roundabout else PATH_TEMPORAL_WEIGHT) * temporal_cost
                    + 0.025 * width_cost
                )

            center, width, _candidate_v = min(compatible, key=continuation_score)
            selected[i] = center
            selected_widths[i] = width
            chosen_indices.append(i)
            miss_streak = 0

        # V1.7：对每个前瞻层做帧间连续性处理。尤其在环岛中，如果某个虚线块
        # 突然消失、另一块出现在画面另一侧，宁可这一层暂时不用，也不允许绿色轨迹瞬移。
        for i, value in enumerate(selected):
            if value is None:
                continue
            temporal = self._previous_band_x[i]
            if temporal is None:
                temporal = self._last_good_band_x[i]
            if temporal is None:
                continue
            delta = float(value) - float(temporal)
            if roundabout and abs(delta) > PATH_ROUNDABOUT_MAX_FRAME_JUMP_RATIO * w:
                selected[i] = None
                continue
            if abs(delta) <= 0.32 * w:
                smooth_alpha = (
                    PATH_TEMPORAL_SMOOTH_ALPHA_ROUNDABOUT
                    if roundabout
                    else PATH_TEMPORAL_SMOOTH_ALPHA_NORMAL
                )
                selected[i] = (
                    smooth_alpha * float(value)
                    + (1.0 - smooth_alpha) * float(temporal)
                )

        valid_indices = [i for i, value in enumerate(selected) if value is not None]
        valid_count = len(valid_indices)
        minimum_bands = 1 if roundabout else NORMAL_MIN_VALID_BANDS
        # 90° 急弯最极端时，黑线可能只剩画面底部一层。若唯一点就在近场且明显靠边，
        # 允许作为“单点边缘救线”，继续朝该侧大角度转，而不是立刻丢线。
        edge_single_rescue = False
        if not roundabout and valid_count == 1:
            only_i = valid_indices[0]
            only_x = float(selected[only_i])
            only_e = (only_x - 0.5 * w) / (0.5 * w)
            edge_single_rescue = only_i <= 2 and abs(only_e) >= 0.55
        if valid_count < minimum_bands and not edge_single_rescue:
            self._previous_band_x = selected
            return (
                False, 0.0, 0.0, 0.0, 0.0, 0.0,
                valid_count, 0.0, selected, "line_too_few_bands",
            )

        # 急弯安全过滤：近场已经明显在一侧时，远场若突然跑到另一侧，视为假目标。
        near_candidates = [i for i in valid_indices if i <= 2]
        if near_candidates:
            near_anchor_i = near_candidates[0]
            near_anchor_e = (float(selected[near_anchor_i]) - 0.5 * w) / (0.5 * w)
            if abs(near_anchor_e) >= HARD_TURN_NEAR_THRESHOLD:
                sign = 1.0 if near_anchor_e > 0 else -1.0
                cut = False
                for i in valid_indices:
                    if cut:
                        selected[i] = None
                        continue
                    e = (float(selected[i]) - 0.5 * w) / (0.5 * w)
                    if i > near_anchor_i and e * sign < -HARD_TURN_OPPOSITE_REJECT:
                        selected[i] = None
                        cut = True
                valid_indices = [i for i, value in enumerate(selected) if value is not None]
                valid_count = len(valid_indices)

        if valid_count < minimum_bands and not edge_single_rescue:
            self._previous_band_x = selected
            return (
                False, 0.0, 0.0, 0.0, 0.0, 0.0,
                valid_count, 0.0, selected, "line_rejected_outliers",
            )

        ys = np.array([LOOKAHEAD_Y_RATIOS[i] for i in valid_indices], dtype=float)
        xs = np.array([selected[i] for i in valid_indices], dtype=float)
        degree = 2 if valid_count >= 4 else 1 if valid_count >= 2 else 0
        poly = None
        if degree > 0:
            # 越靠近机器人权重越高；远场只负责预判，不能推翻近场事实。
            weights = np.array([1.55 - 0.70 * (1.0 - y) for y in ys], dtype=float)
            try:
                coeff = np.polyfit(ys, xs, degree, w=weights)
                poly = np.poly1d(coeff)
            except (ValueError, np.linalg.LinAlgError):
                poly = None

        min_y = float(np.min(ys))
        max_y = float(np.max(ys))

        def estimated_x(target_ratio: float) -> float:
            nearest = min(valid_indices, key=lambda idx: abs(LOOKAHEAD_Y_RATIOS[idx] - target_ratio))
            measured = float(selected[nearest])
            # 不做远距离外推：没有真实远场点时宁可使用最近测量，也不凭拟合“猜”到另一边。
            if poly is None or target_ratio < min_y - 0.035 or target_ratio > max_y + 0.035:
                return measured
            fitted = float(poly(target_ratio))
            fitted = clamp(fitted, measured - 0.20 * w, measured + 0.20 * w)
            return clamp(fitted, 0.02 * w, 0.98 * w)

        near_x = estimated_x(0.86)
        mid_x = estimated_x(0.60)
        far_x = estimated_x(0.38)

        # V1.24：曲率只由 near/mid/far 的相对形状计算，与 TARGET 偏置无关。
        # 直线时保持用户标定好的 +0.12；弯得越急，平滑减到 +0.07。
        near_center_e = (near_x - 0.5 * w) / (0.5 * w)
        mid_center_e = (mid_x - 0.5 * w) / (0.5 * w)
        far_center_e = (far_x - 0.5 * w) / (0.5 * w)
        curve_shape = max(
            abs(mid_center_e - near_center_e),
            0.80 * abs(far_center_e - mid_center_e),
            0.65 * abs(far_center_e - near_center_e),
        )
        curve_blend = clamp(
            (curve_shape - CURVE_OFFSET_REDUCTION_START)
            / max(1e-6, CURVE_OFFSET_REDUCTION_FULL - CURVE_OFFSET_REDUCTION_START),
            0.0,
            1.0,
        )
        # V1.31：动态镜头偏移只允许在“本圈台阶尚未完成”时生效。
        # 台阶完成后 use_dynamic_offset=False，后半圈固定使用 0 偏移，
        # 避免 +0.12/-0.12 再影响环岛；下一圈会自动重新打开。
        if use_dynamic_offset and not roundabout:
            # V1.54：只在台阶前普通曲线做很小的外侧补偿。
            # 与旧版0.24画面宽度的大偏移完全不同：现在最大只移动0.04画面宽度。
            effective_offset = (
                TARGET_X_OFFSET_RATIO * (1.0 - curve_blend)
                + CURVE_TARGET_X_OFFSET_RATIO * curve_blend
            )
        else:
            # 台阶完成后的 S 弯、以及 ROUNDABOUT/FOLLOW/EXIT 都固定真正中心0.00。
            effective_offset = 0.0
        self._effective_target_offset_ratio = effective_offset
        target_x = 0.5 * w + effective_offset * w

        def norm_x(x: float) -> float:
            return clamp((x - target_x) / (0.5 * w), -1.0, 1.0)

        near_e = norm_x(near_x)
        mid_e = norm_x(mid_x)
        far_e = norm_x(far_x)

        # V1.3：控制的第一目标是“车身重新骑到黑线中心”，而不是提前切弯。
        # V1.2 把 far-near 当成较强的航向项；在 90° / S 弯里经常出现
        # near 在右、far 已经跑到左边，此时远场会把近场纠偏抵消，导致车辆切内弯，
        # 出现单侧轮胎压黑线甚至贴边界。现在转向主要由 near + mid 决定，
        # far 只用于提前降速，不再直接把方向“拉过去”。
        near_mid_opposite = (near_e * mid_e < -0.015)
        local_heading = mid_e - near_e

        if near_mid_opposite:
            # S弯/弯道入口处，远场经常已经看见下一段弯，近场仍决定车轮当前位置。
            # 这里优先把近处黑线带回车体中心，避免提前切弯导致单侧轮压线。
            error = (
                CURVE_OPPOSITE_NEAR_WEIGHT * near_e
                + (1.0 - CURVE_OPPOSITE_NEAR_WEIGHT) * mid_e
            )
            reason = "line_recenter_transition"
        elif (
            CURVE_CENTER_RIDING_ENABLED
            and (not roundabout)
            and curve_shape >= CURVE_CENTER_SHAPE_THRESHOLD
        ):
            # V1.51：曲线时第一目标是“最近处黑线必须继续从车体中心下方通过”。
            # 旧逻辑/旧offset会为了追远处弯线而提前切内弯，导致一侧轮胎压线。
            # 这里90%看near，只用少量mid+heading告诉车辆弯向。
            error = (
                CURVE_CENTER_NEAR_WEIGHT * near_e
                + CURVE_CENTER_MID_WEIGHT * mid_e
                + CURVE_CENTER_HEADING_GAIN * local_heading
            )
            reason = "line_curve_center_riding"
        elif abs(near_e) >= HARD_TURN_NEAR_THRESHOLD:
            error = 0.90 * near_e + 0.10 * mid_e
            reason = "line_hard_turn_centering"
        else:
            error = 0.78 * near_e + 0.22 * mid_e + 0.06 * local_heading
            reason = "line_center_tracking"

        error = clamp(error, -1.0, 1.0)

        coverage = valid_count / float(len(LOOKAHEAD_Y_RATIOS))
        adjacent_quality: List[float] = []
        last_i: Optional[int] = None
        last_x: Optional[float] = None
        for i, value in enumerate(selected):
            if value is None:
                continue
            if last_i is not None and last_x is not None:
                gap = max(1, i - last_i)
                allowed = PATH_MAX_STEP_RATIO * w * (1.0 + 0.32 * (gap - 1))
                adjacent_quality.append(max(0.0, 1.0 - abs(float(value) - last_x) / max(1.0, allowed)))
            last_i = i
            last_x = float(value)
        continuity = float(np.mean(adjacent_quality)) if adjacent_quality else 0.50
        confidence = clamp(0.70 * coverage + 0.30 * continuity, 0.0, 1.0)
        if edge_single_rescue:
            confidence = max(confidence, 0.31)
            reason = "line_edge_single_rescue"

        # 实线分数仍使用连通块纵向覆盖。
        y0 = int(0.27 * h)
        y1 = int(0.985 * h)
        roi = dark[y0:y1, x_left:x_right]
        count, _labels, stats, _ = cv2.connectedComponentsWithStats(roi)
        max_vertical_span = 0
        for idx in range(1, count):
            _x, _y, cw, ch, area = stats[idx]
            if area < 20 or cw > int(0.30 * w):
                continue
            max_vertical_span = max(max_vertical_span, int(ch))
        span_score = clamp(max_vertical_span / max(1.0, 0.40 * h), 0.0, 1.0)
        solid_score = clamp(0.55 * span_score + 0.45 * coverage, 0.0, 1.0)

        if not roundabout and confidence < NORMAL_MIN_CONFIDENCE and not edge_single_rescue:
            valid = False
            reason = "line_low_confidence"
        else:
            valid = True

        self._previous_band_x = selected
        if valid:
            for i, value in enumerate(selected):
                if value is not None:
                    self._last_good_band_x[i] = value

        return (
            valid, error, confidence, near_e, mid_e, far_e,
            valid_count, solid_score, selected, reason,
        )

    def _small_neutral_components(
        self,
        dark: np.ndarray,
        hsv: np.ndarray,
        y_top_ratio: float,
        y_bottom_ratio: float,
    ) -> List[Tuple[float, float, int, int, int]]:
        h, w = dark.shape
        x0 = int(0.05 * w)
        x1 = int(0.95 * w)
        y0 = int(y_top_ratio * h)
        y1 = int(y_bottom_ratio * h)
        roi = dark[y0:y1, x0:x1]
        n, labels, stats, centers = cv2.connectedComponentsWithStats(roi)
        out: List[Tuple[float, float, int, int, int]] = []
        for idx in range(1, n):
            x, y, cw, ch, area = [int(v) for v in stats[idx]]
            if not (30 <= area <= 3200 and 4 <= cw <= 90 and 4 <= ch <= 100):
                continue
            rectangularity = area / float(max(1, cw * ch))
            if rectangularity < 0.28:
                continue
            gx0, gy0 = x0 + x, y0 + y
            gx1, gy1 = gx0 + cw, gy0 + ch
            pad = max(6, min(18, int(0.35 * max(cw, ch))))
            px0, px1 = max(0, gx0 - pad), min(w, gx1 + pad)
            py0, py1 = max(0, gy0 - pad), min(h, gy1 + pad)
            patch = hsv[py0:py1, px0:px1]
            if patch.size == 0:
                continue
            bright_neutral = (
                (patch[:, :, 2] >= 92) & (patch[:, :, 1] <= 185)
            )
            if float(np.mean(bright_neutral)) < 0.16:
                continue
            cx, cy = centers[idx]
            out.append((float(cx + x0), float(cy + y0), cw, ch, area))
        return out

    def _detect_roundabout_gate(
        self, dark: np.ndarray, hsv: np.ndarray
    ) -> Tuple[bool, float, List[Tuple[float, float, int, int, int]]]:
        """检测真正的连续虚线链，尽量排除地板接缝和反光碎块。

        旧版只要 4 个小黑块能被宽松地串起来就触发，车库地板纹理也可能满足。
        现在增加：最小块数、最小纵向跨度、最小相邻纵向间距以及间距规律性。
        """
        h, w = dark.shape
        comps = self._small_neutral_components(dark, hsv, 0.22, 0.80)
        if len(comps) < ROUNDABOUT_GATE_MIN_CHAIN:
            # 少量零散黑块不画橙框，避免把普通地板纹理看成“环岛候选”。
            return False, 0.0, []

        order = sorted(range(len(comps)), key=lambda i: comps[i][1], reverse=True)
        best_len = [1 for _ in comps]
        predecessor: List[Optional[int]] = [None for _ in comps]
        min_dy = 0.020 * h
        max_dy = 0.18 * h

        for pos_i, i in enumerate(order):
            xi, yi, *_ = comps[i]
            for j in order[:pos_i]:
                xj, yj, *_ = comps[j]
                dy = yj - yi
                if dy < min_dy or dy > max_dy:
                    continue
                # 真虚线可以有弯曲，但相邻块不会在横向突然跳很远。
                allowed_dx = 0.065 * w + 0.42 * dy
                if abs(xi - xj) > allowed_dx:
                    continue
                proposal = best_len[j] + 1
                if proposal > best_len[i]:
                    best_len[i] = proposal
                    predecessor[i] = j

        best_idx = int(np.argmax(best_len))
        chain_indices: List[int] = []
        cursor: Optional[int] = best_idx
        while cursor is not None:
            chain_indices.append(cursor)
            cursor = predecessor[cursor]

        chain = [comps[i] for i in chain_indices]
        chain_len = len(chain)
        if chain_len < ROUNDABOUT_GATE_MIN_CHAIN:
            score = clamp(chain_len / float(ROUNDABOUT_GATE_MIN_CHAIN), 0.0, 1.0)
            return False, score, chain

        ys = np.array([item[1] for item in chain], dtype=float)
        xs = np.array([item[0] for item in chain], dtype=float)
        y_span_ratio = float(np.ptp(ys) / max(1.0, h))

        sorted_y = np.sort(ys)
        gaps = np.diff(sorted_y)
        if len(gaps) >= 2 and float(np.mean(gaps)) > 1e-6:
            gap_cv = float(np.std(gaps) / np.mean(gaps))
        else:
            gap_cv = 99.0

        # 防止随机散点刚好被串成很长的链。真正环岛虚线在局部应是一条路径。
        if chain_len >= 3:
            coeff = np.polyfit(ys, xs, 1)
            predicted_x = np.polyval(coeff, ys)
            line_rmse_ratio = float(
                np.sqrt(np.mean((xs - predicted_x) ** 2)) / max(1.0, w)
            )
        else:
            line_rmse_ratio = 1.0

        span_score = clamp(y_span_ratio / 0.32, 0.0, 1.0)
        length_score = clamp(chain_len / 7.0, 0.0, 1.0)
        regularity_score = clamp(1.0 - gap_cv / 1.30, 0.0, 1.0)
        shape_score = clamp(1.0 - line_rmse_ratio / 0.10, 0.0, 1.0)
        score = clamp(
            0.34 * length_score
            + 0.30 * span_score
            + 0.18 * regularity_score
            + 0.18 * shape_score,
            0.0,
            1.0,
        )

        visible = (
            chain_len >= ROUNDABOUT_GATE_MIN_CHAIN
            and y_span_ratio >= ROUNDABOUT_GATE_MIN_Y_SPAN_RATIO
            and gap_cv <= ROUNDABOUT_GATE_MAX_GAP_CV
            and line_rmse_ratio <= 0.085
        )
        # 第三个返回值只给真正被串成同一条虚线链的块，用于橙框调试。
        return visible, score, chain

    def _detect_roundabout_exit_solid(
        self, event_dark: np.ndarray
    ) -> Tuple[bool, float, Optional[Tuple[int, int, int, int]]]:
        """只认真正连续的实线出口，不把环岛虚线块当作出口。

        使用严格 event_dark；要求同一个连通域从较远处一直延伸到近场。
        这比旧的 solid_score 严格得多，后者会被多个独立虚线块的 band coverage 抬高。
        """
        h, w = event_dark.shape
        x0, x1 = int(0.04 * w), int(0.96 * w)
        y0, y1 = int(0.18 * h), int(0.99 * h)
        roi = event_dark[y0:y1, x0:x1]
        n, _labels, stats, _centers = cv2.connectedComponentsWithStats(roi)
        best_score = 0.0
        best_box: Optional[Tuple[int, int, int, int]] = None
        visible = False
        for idx in range(1, n):
            x, y, cw, ch, area = [int(v) for v in stats[idx]]
            if area < ROUNDABOUT_EXIT_SOLID_MIN_AREA:
                continue
            if cw > int(ROUNDABOUT_EXIT_SOLID_MAX_WIDTH_RATIO * w):
                continue
            gx, gy = x0 + x, y0 + y
            top_ratio = gy / float(h)
            bottom_ratio = (gy + ch) / float(h)
            height_ratio = ch / float(h)
            height_score = clamp(
                height_ratio / max(1e-6, ROUNDABOUT_EXIT_SOLID_MIN_HEIGHT_RATIO),
                0.0, 1.0,
            )
            reach_score = 0.5 * clamp(
                (ROUNDABOUT_EXIT_SOLID_TOP_MAX_RATIO - top_ratio + 0.10) / 0.10,
                0.0, 1.0,
            ) + 0.5 * clamp(
                (bottom_ratio - ROUNDABOUT_EXIT_SOLID_BOTTOM_MIN_RATIO + 0.10) / 0.10,
                0.0, 1.0,
            )
            area_score = clamp(area / 420.0, 0.0, 1.0)
            score = clamp(0.55 * height_score + 0.30 * reach_score + 0.15 * area_score, 0.0, 1.0)
            if score > best_score:
                best_score = score
                best_box = (gx, gy, cw, ch)
            if (
                height_ratio >= ROUNDABOUT_EXIT_SOLID_MIN_HEIGHT_RATIO
                and top_ratio <= ROUNDABOUT_EXIT_SOLID_TOP_MAX_RATIO
                and bottom_ratio >= ROUNDABOUT_EXIT_SOLID_BOTTOM_MIN_RATIO
            ):
                visible = True
        return visible, best_score, best_box

    def _detect_crosswalk(self, dark: np.ndarray) -> Tuple[bool, float]:
        """在人行横道较远时就找同一横排上的多个黑条，尽量停在线前。"""
        h, w = dark.shape
        x0 = int(0.07 * w)
        x1 = int(0.93 * w)
        y0 = int(0.24 * h)
        y1 = int(0.66 * h)
        max_runs = 0
        good_rows = 0
        for y in range(y0, y1, 3):
            strip_y0 = max(0, y - 2)
            strip_y1 = min(h, y + 3)
            occupancy = np.mean(dark[strip_y0:strip_y1, x0:x1] > 0, axis=0)
            binary = (occupancy >= 0.38).astype(np.uint8)
            runs = []
            for start, end in self._runs(binary):
                width = end - start
                if 5 <= width <= int(0.10 * w):
                    runs.append((start, end))
            if len(runs) >= 6:
                centers = [0.5 * (a + b) for a, b in runs]
                span = max(centers) - min(centers)
                if span >= 0.36 * (x1 - x0):
                    good_rows += 1
                    max_runs = max(max_runs, len(runs))
        score = clamp(0.55 * (max_runs / 9.0) + 0.45 * (good_rows / 10.0), 0.0, 1.0)
        return (max_runs >= 7 and good_rows >= 5), score

    def _detect_yellow_finish(
        self, hsv: np.ndarray
    ) -> Tuple[bool, float, np.ndarray]:
        h, w = hsv.shape[:2]
        yellow = cv2.inRange(
            hsv,
            np.array((YELLOW_H_MIN, YELLOW_S_MIN, YELLOW_V_MIN), np.uint8),
            np.array((YELLOW_H_MAX, 255, 255), np.uint8),
        )
        yellow = cv2.morphologyEx(
            yellow, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=1
        )
        x0, x1 = int(0.07 * w), int(0.93 * w)
        y0, y1 = int(0.20 * h), int(0.84 * h)
        best_fraction = 0.0
        consecutive = 0
        best_consecutive = 0
        for y in range(y0, y1):
            row = yellow[y, x0:x1] > 0
            fraction = float(np.mean(row))
            best_fraction = max(best_fraction, fraction)
            if fraction >= 0.30:
                consecutive += 1
                best_consecutive = max(best_consecutive, consecutive)
            else:
                consecutive = 0
        score = clamp(0.65 * (best_fraction / 0.55) + 0.35 * (best_consecutive / 8.0), 0.0, 1.0)
        visible = best_fraction >= 0.30 and best_consecutive >= 3
        return visible, score, yellow

    def process(
        self,
        frame: np.ndarray,
        roundabout: bool = False,
        use_dynamic_offset: bool = True,
        show_yellow_debug: bool = True,
    ) -> VisionResult:
        # 保留摄像头 640x480 的视场和 RAW 录像，只在算法内部缩放到 320x240。
        original_h, original_w = frame.shape[:2]
        if original_w != PROCESS_WIDTH or original_h != PROCESS_HEIGHT:
            work = cv2.resize(
                frame, (PROCESS_WIDTH, PROCESS_HEIGHT), interpolation=cv2.INTER_AREA
            )
        else:
            work = frame

        dark, event_dark, hsv, _blue = self._make_dark_mask(work)
        self._update_roundabout_blue_guard(hsv, roundabout)
        (
            line_valid,
            error,
            confidence,
            near_e,
            mid_e,
            far_e,
            valid_bands,
            solid_score,
            band_x,
            reason,
        ) = self._extract_line(
            work,
            dark,
            hsv,
            roundabout,
            use_dynamic_offset=use_dynamic_offset,
        )

        gate, gate_score, components = self._detect_roundabout_gate(event_dark, hsv)
        # V1.8 不再做视觉出口判断；出环运动按 icn.py 的 0.80m 局部位移结束。
        exit_solid, exit_score, exit_box = False, 0.0, None
        crosswalk, crosswalk_score = self._detect_crosswalk(event_dark)
        yellow, yellow_score, yellow_mask = self._detect_yellow_finish(hsv)

        debug = work.copy()
        h, w = work.shape[:2]
        overlay = np.zeros_like(debug)
        overlay[:, :, 2] = dark
        debug = cv2.addWeighted(debug, 0.86, overlay, 0.18, 0.0)

        # 多前瞻点：绿色为被连续路径接受的点。没有绿色点的远场不会参与拟合。
        previous_pt: Optional[Tuple[int, int]] = None
        for ratio, x in zip(LOOKAHEAD_Y_RATIOS, band_x):
            y = int(ratio * h)
            cv2.line(debug, (0, y), (w - 1, y), (80, 80, 80), 1)
            if x is not None:
                pt = (int(x), y)
                cv2.circle(debug, pt, 4, (0, 255, 0), -1)
                if previous_pt is not None:
                    cv2.line(debug, previous_pt, pt, (0, 255, 0), 2)
                previous_pt = pt
        debug_target_x = int(0.5 * w + self._effective_target_offset_ratio * w)
        cv2.line(debug, (debug_target_x, int(0.27 * h)), (debug_target_x, h - 1), (255, 255, 0), 1)

        for cx, cy, cw, ch, _area in components:
            x0 = int(cx - 0.5 * cw)
            y0 = int(cy - 0.5 * ch)
            x1 = int(cx + 0.5 * cw)
            y1 = int(cy + 0.5 * ch)
            cv2.rectangle(debug, (x0, y0), (x1, y1), (0, 165, 255), 1)

        # V1.8 默认关闭紫框：它只是旧版“连续实线出口”的兜底诊断框，不参与主要出环判断。
        # V1.8 已完全取消视觉出口判断；紫框不参与任何控制。
        if SHOW_EXIT_SOLID_BOX and roundabout and exit_solid and exit_box is not None:
            ex, ey, ew, eh = exit_box
            cv2.rectangle(
                debug, (ex, ey), (ex + ew, ey + eh), (255, 0, 255), 2,
            )

        # 黄色框只是“终点黄线诊断框”，从来不是车辆跟踪目标。
        # V1.45：环岛/台阶阶段完全不画它，避免赛道外棕色区域被HSV误认为黄色后
        # 产生大矩形造成误解；终点阶段也只画明显“横向长条”的候选。
        if show_yellow_debug and yellow:
            contours, _ = cv2.findContours(
                yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            for contour in contours:
                x, y, cw, ch = cv2.boundingRect(contour)
                horizontal_strip = (
                    cw >= int(0.30 * w)
                    and ch <= int(0.18 * h)
                    and cw >= 2.8 * max(1, ch)
                )
                if horizontal_strip and int(0.18 * h) <= y <= int(0.88 * h):
                    cv2.rectangle(
                        debug, (x, y), (x + cw, y + ch),
                        (0, 255, 255), 2,
                    )

        # DEBUG 恢复为原始分辨率，现场窗口和录像仍保持 640x480，便于观察。
        if debug.shape[1] != original_w or debug.shape[0] != original_h:
            debug = cv2.resize(debug, (original_w, original_h), interpolation=cv2.INTER_NEAREST)

        # 右下角二值图预览：这是“主巡线 dark mask”，不是额外算法结果。
        # 白色像素=程序认为可能属于黑胶带的候选；黑色=非候选。
        preview = cv2.resize(
            dark,
            (BINARY_PREVIEW_WIDTH, BINARY_PREVIEW_HEIGHT),
            interpolation=cv2.INTER_NEAREST,
        )
        preview_bgr = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)
        ph, pw = preview_bgr.shape[:2]
        px = max(0, original_w - pw - BINARY_PREVIEW_MARGIN)
        py = max(0, original_h - ph - BINARY_PREVIEW_MARGIN)
        debug[py:py + ph, px:px + pw] = preview_bgr
        cv2.rectangle(debug, (px, py), (px + pw - 1, py + ph - 1), (0, 255, 255), 2)
        cv2.rectangle(debug, (px, py), (px + pw - 1, py + 20), (0, 0, 0), -1)
        cv2.putText(
            debug, "BINARY white=black-line candidate", (px + 4, py + 14),
            cv2.FONT_HERSHEY_SIMPLEX, 0.34, (0, 255, 255), 1, cv2.LINE_AA,
        )

        return VisionResult(
            line_valid=line_valid,
            error=error,
            confidence=confidence,
            near_error=near_e,
            mid_error=mid_e,
            far_error=far_e,
            valid_bands=valid_bands,
            solid_score=solid_score,
            roundabout_gate=gate,
            roundabout_gate_score=gate_score,
            roundabout_exit_solid=exit_solid,
            roundabout_exit_score=exit_score,
            crosswalk=crosswalk,
            crosswalk_score=crosswalk_score,
            yellow_finish=yellow,
            yellow_score=yellow_score,
            debug_frame=debug,
            reason=reason,
        )


class WheelLegRaceNode(Node):
    NODE_NAME = "wheelleg_vision_race"

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)
        apply_ros_parameters(self)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.cmd_posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self.cmd_jump_pub = self.create_publisher(Bool, "/cmd_jump", 10)
        self.cmd_jump_profile_pub = self.create_publisher(
            Float32MultiArray, "/cmd_jump_profile", 10
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Imu, STEP_IMU_TOPIC, self._on_imu, 20)
        self.create_subscription(
            StepDetection, STEP_DETECTION_TOPIC, self._on_step_detection, 10
        )

        self.vision = CourseVision()
        self._frame_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._stop_capture = threading.Event()
        self._capture: Optional[cv2.VideoCapture] = None
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_time = 0.0
        self._frame_sequence = 0
        self._last_processed_sequence = -1
        self._camera_healthy = False

        self._odom_lock = threading.Lock()
        self._odom_pose: Optional[Tuple[float, float, float]] = None
        self._last_odom_time = 0.0

        # 车体自带 IMU：使用真实四元数 yaw + gyro_z。
        self._imu_yaw: Optional[float] = None
        self._imu_yaw_rate = 0.0
        self._last_imu_time = 0.0

        # 台阶检测由独立 step_detector_node.py 发布；这里不重复做深度 RANSAC。
        self._latest_step_detection: Optional[StepDetection] = None
        self._latest_step_detection_time = 0.0

        # V1.48：不再完全依赖 detector 的“严格连续5帧 detected=True”。
        # 这里对每一条可信 candidate 做证据积分，允许第二圈短暂/抖动候选被抓住。
        self._step_race_candidate_evidence = 0.0
        self._step_race_candidate_plausible = False

        # 台阶控制状态。IDLE 时完全不干涉原来的循迹控制。
        self._step_state = "IDLE"
        self._step_done_this_lap = False

        # V1.32：冲刺起点标定。达到“可冲刺”条件后记录一次并停车保持。
        self._step_calib_printed = False
        self._step_calib_pose: Optional[Tuple[float, float, float]] = None
        self._step_calib_depth_distance = float("nan")
        self._step_calib_lap_travel = float("nan")
        self._step_calib_align_frames = 0
        self._step_calib_alignment_latched = False
        self._step_calib_at_target_depth = False
        self._step_calib_final_frames = 0
        # V1.37：只要台阶标定接管过一次，就不允许因为深度瞬时丢失退回普通巡线。
        self._step_calib_takeover_latched = False
        self._step_calib_takeover_logged = False

        # V1.38：正式固定冲刺也使用“接管锁存”，避免靠近过程中深度瞬时丢失后
        # 恢复普通巡线继续前冲。
        self._step_fixed_takeover_latched = False
        self._step_fixed_takeover_logged = False
        self._step_fixed_candidate_frames = 0
        self._step_race_candidate_evidence = 0.0
        self._step_race_candidate_plausible = False
        self._step_fixed_last_valid_distance = float("nan")
        self._step_fixed_lost_since = 0.0
        self._step_fixed_takeover_yaw: Optional[float] = None
        self._step_fixed_reverse_active = False
        self._step_fixed_reverse_start_pose: Optional[Tuple[float, float, float]] = None
        self._step_fixed_reverse_traveled = 0.0
        self._step_fixed_reverse_target_yaw: Optional[float] = None
        self._step_fixed_reverse_start_time = 0.0
        self._step_fixed_reverse_reacquire_frames = 0
        self._step_fixed_reverse_target_distance = 0.0

        # V1.49：深度短时滤波 + “过近”连续确认，避免单帧0.65m触发倒车。
        self._step_depth_history = deque(maxlen=STEP_FIXED_DEPTH_FILTER_SAMPLES)
        self._step_fixed_too_close_frames = 0

        # 固定冲刺起点0.70m窗口连续确认。
        self._step_fixed_start_confirm_frames = 0

        # 台阶候选已经确认但车身尚未摆正时，仍由视觉循迹控制。
        self._step_waiting_alignment = False
        self._step_align_frames = 0
        self._step_align_last_reason = "waiting_detection"

        self._step_detected_distance = float("nan")
        self._step_locked_height = float("nan")
        self._step_locked_confidence = 0.0
        self._step_lock_pose: Optional[Tuple[float, float, float]] = None
        self._step_lock_time = 0.0
        self._step_target_yaw: Optional[float] = None
        self._step_yaw_source = "none"
        self._step_yaw_error = 0.0
        self._step_target_travel = 0.0
        self._step_odom_traveled = 0.0
        self._step_commanded_traveled = 0.0
        self._step_trigger_traveled = 0.0
        self._step_remaining = 0.0
        self._step_approach_start_time = 0.0
        self._step_last_approach_time = 0.0
        self._step_deadline = 0.0
        self._step_stop_reason = "等待台阶"
        self._step_last_mode = "step_idle"
        self._step_jump_confirm_event = threading.Event()
        self._step_jump_confirm_prompted = False
        self._step_jump_confirm_thread: Optional[threading.Thread] = None
        self._step_jump_confirm_request_time = 0.0
        self._post_step_reacquire_active = False
        self._post_step_reacquire_until = 0.0
        self._post_step_reacquire_start_pose: Optional[Tuple[float, float, float]] = None

        # 赛道状态：每圈只按视觉事件推进，不按全局坐标推进。
        self.phase = "SEEK_ROUNDABOUT"
        self.phase_enter_time = time.monotonic()
        self.lap_count = 0
        self._roundabout_gate_frames = 0
        self._roundabout_exit_frames = 0
        self._roundabout_gate_evidence = 0.0
        self._roundabout_return_evidence = 0.0  # V1.8 保留字段仅兼容旧调试，不参与出环
        self._first_roundabout_gate_seek: Optional[float] = None
        self._roundabout_gate_first_seen_seek: Optional[float] = None
        self._roundabout_approach_compensation = 0.0
        self._roundabout_gate_source = "none"
        self._crosswalk_frames = 0
        self._yellow_frames = 0
        self._stop_until = 0.0
        self._roundabout_exit_guard_until = 0.0

        # V1.19：人行横道和终点线都采用“视觉触发 -> 局部相对位移 -> 动作”，
        # 不使用全局 /odom x/y 区域。
        self._crosswalk_approach_start_pose: Optional[Tuple[float, float, float]] = None
        self._crosswalk_approach_distance = 0.0
        self._finish_pass_start_pose: Optional[Tuple[float, float, float]] = None
        self._finish_pass_distance = 0.0
        # V1.19：不再维护额外 rb_recover 标志。
        # 是否允许“丢线默认左转”完全由 phase == ROUNDABOUT_FOLLOW 决定。

        # “寻找环岛”也只使用每圈开始后的短程相对里程。
        # 第一帧有效图像到来时再真正开始计时，避免摄像头启动耗时吃掉保护延时。
        self._race_clock_started = False
        self._seek_last_odom: Optional[Tuple[float, float, float]] = None
        self._seek_distance = 0.0

        # V1.7：视觉第一次确认远处环岛后，再建立一次“进环接近段”的局部零点。
        self._approach_start_pose: Optional[Tuple[float, float, float]] = None
        self._approach_distance = 0.0

        # V1.8：完全参考 icn.py。正式进环时只保存一个局部起点，
        # 之后 ring_entry_disp = hypot(x-x0, y-y0)，绝不把每帧抖动累计起来。
        self._rb_entry_pose: Optional[Tuple[float, float, float]] = None
        self._rb_entry_displacement = 0.0

        # 控制器。
        self._previous_error = 0.0
        self._previous_control_time = 0.0
        self._have_previous_error = False
        self._last_valid_line_time = 0.0
        self._last_good_angular = 0.0
        self._last_curve_turn_angular = 0.0
        self._last_curve_turn_time = 0.0
        self._last_command = (0.0, 0.0)
        self._last_motion_update_time = time.monotonic()
        self._last_status_log = 0.0

        self._exit_requested = False
        self._closing = False
        self._debug_window_ready = False
        self._debug_window_sized = False
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None
        self._video_failed = False
        self._raw_writer: Optional[cv2.VideoWriter] = None
        self._raw_path: Optional[Path] = None
        self._raw_failed = False
        self._race_log_file = None
        self._race_log_writer: Optional[csv.DictWriter] = None
        self._race_log_path: Optional[Path] = None
        self._race_log_failed = False
        self._last_step_gate_log = 0.0

        if SHOW_DEBUG:
            try:
                cv2.namedWindow("WheelLeg Race Vision", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("WheelLeg Race Vision", IMAGE_WIDTH, IMAGE_HEIGHT)
                self._debug_window_ready = True
            except cv2.error as error:
                self.get_logger().warn(f"无法创建调试窗口，继续运行: {error}")

        self._publish_zero()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="race_camera", daemon=True
        )
        self._capture_thread.start()
        self._timer = self.create_timer(1.0 / CONTROL_RATE, self._control_timer)

        self.get_logger().warn(
            "比赛程序已启动："
            f"巡线速度{FORWARD_SPEED:.2f}，最大转向{MAX_ANGULAR_SPEED:.2f}；"
            f"环岛速度{ROUNDABOUT_FORWARD_SPEED:.2f}，左转{ROUNDABOUT_ENTRY_LEFT_ANGULAR:.2f}；"
            f"台阶冲刺{STEP_APPROACH_SPEED_M_S:.2f}，距离{STEP_FIXED_SPRINT_DISTANCE_M:.3f}，"
            f"提前量{STEP_JUMP_TRIGGER_LEAD_M:.3f}，跳高{STEP_JUMP_MIN_HEIGHT_M:.2f}->{STEP_JUMP_MAX_HEIGHT_M:.2f}；"
            f"手动确认跳跃={int(STEP_MANUAL_CONFIRM_BEFORE_JUMP)}"
        )

    # ------------------------------ camera ---------------------------------
    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        if CAMERA_BACKEND == "v4l2":
            capture = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(CAMERA_ID)
        if not capture.isOpened():
            capture.release()
            return None
        if CAMERA_BACKEND == "v4l2":
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*PIXEL_FORMAT))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        self.get_logger().info(
            f"摄像头连接: {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
            f"{capture.get(cv2.CAP_PROP_FPS):.1f}fps"
        )
        return capture

    @staticmethod
    def _transform_frame(frame: np.ndarray) -> np.ndarray:
        if ROTATION_DEGREES == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif ROTATION_DEGREES == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif ROTATION_DEGREES == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if MIRROR_MODE == "horizontal":
            frame = cv2.flip(frame, 1)
        elif MIRROR_MODE == "vertical":
            frame = cv2.flip(frame, 0)
        elif MIRROR_MODE == "both":
            frame = cv2.flip(frame, -1)
        return frame

    def _capture_loop(self) -> None:
        while not self._stop_capture.is_set():
            with self._capture_lock:
                capture = self._capture
            if capture is None or not capture.isOpened():
                capture = self._open_camera()
                with self._capture_lock:
                    self._capture = capture
                if capture is None:
                    with self._frame_lock:
                        self._camera_healthy = False
                        self._latest_frame = None
                    self._stop_capture.wait(CAMERA_RETRY_SECONDS)
                    continue
            ok, frame = capture.read()
            if not ok or frame is None:
                with self._frame_lock:
                    self._camera_healthy = False
                    self._latest_frame = None
                capture.release()
                with self._capture_lock:
                    self._capture = None
                self._stop_capture.wait(CAMERA_RETRY_SECONDS)
                continue
            frame = self._transform_frame(frame)
            with self._frame_lock:
                self._latest_frame = frame
                self._latest_frame_time = time.monotonic()
                self._frame_sequence += 1
                self._camera_healthy = True

    # ------------------------------- odom ----------------------------------
    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(float(q.x), float(q.y), float(q.z), float(q.w))
        with self._odom_lock:
            self._odom_pose = (float(p.x), float(p.y), yaw)
            self._last_odom_time = time.monotonic()

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._imu_yaw = quaternion_to_yaw(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        self._imu_yaw_rate = float(msg.angular_velocity.z)
        self._last_imu_time = time.monotonic()

    def _on_step_detection(self, msg: StepDetection) -> None:
        self._latest_step_detection = msg
        self._latest_step_detection_time = time.monotonic()

        # detector 即使 detected=False，只要内部已经找到 candidate，
        # distance/height/confidence 仍然会被发布出来。
        # V1.48 利用这些信息做“带衰减证据积分”，避免第二圈因为少一两帧
        # 就把真实台阶完全当成没看到。
        d = float(msg.distance)
        h = float(msg.height)
        c = float(msg.confidence)
        plausible = (
            math.isfinite(d)
            and math.isfinite(h)
            and d <= STEP_RACE_CANDIDATE_MAX_DISTANCE_M
            and STEP_RACE_CANDIDATE_MIN_HEIGHT_M
                <= h <= STEP_RACE_CANDIDATE_MAX_HEIGHT_M
            and c >= STEP_RACE_CANDIDATE_MIN_CONFIDENCE
        )
        self._step_race_candidate_plausible = plausible
        if plausible:
            self._step_depth_history.append(
                (self._latest_step_detection_time, d)
            )
            self._step_race_candidate_evidence = min(
                STEP_RACE_CANDIDATE_EVIDENCE_MAX,
                self._step_race_candidate_evidence
                + STEP_RACE_CANDIDATE_EVIDENCE_GAIN,
            )
        else:
            self._step_race_candidate_evidence = max(
                0.0,
                self._step_race_candidate_evidence
                - STEP_RACE_CANDIDATE_EVIDENCE_DECAY,
            )

    def _imu_fresh(self, now: float) -> bool:
        return (
            self._imu_yaw is not None
            and self._last_imu_time > 0.0
            and now - self._last_imu_time <= STEP_IMU_TIMEOUT_SECONDS
        )

    def _step_heading_snapshot(self, now: float) -> Tuple[Optional[float], str]:
        # V1.24：短程冲刺固定使用 /odom yaw。
        odom = self._odom_snapshot()
        if odom is not None and now - self._last_odom_time <= STEP_ODOM_TIMEOUT_SECONDS:
            return float(odom[2]), "odom"
        return None, "none"

    def _odom_snapshot(self) -> Optional[Tuple[float, float, float]]:
        with self._odom_lock:
            return self._odom_pose

    def _reset_seek_roundabout_progress(self) -> None:
        """每圈开始时重新建立寻找环岛的局部里程零点。"""
        self._seek_distance = 0.0
        self._seek_last_odom = self._odom_snapshot()

    def _update_seek_roundabout_progress(self) -> None:
        if self.phase != "SEEK_ROUNDABOUT":
            return
        current = self._odom_snapshot()
        if current is None:
            return
        if self._seek_last_odom is None:
            self._seek_last_odom = current
            return
        x, y, _yaw = current
        lx, ly, _lyaw = self._seek_last_odom
        segment = math.hypot(x - lx, y - ly)
        commanded_forward = self._last_command[0] >= SEEK_DISTANCE_MIN_COUNT_SPEED
        # 只在程序确实命令车辆前进时累计；停车时的 /odom 漂移只刷新参考点，不算路程。
        if commanded_forward and segment <= 0.20:
            self._seek_distance += segment
        self._seek_last_odom = current

    def _reset_roundabout_approach_progress(self) -> None:
        """视觉确认环岛后，以确认瞬间为局部零点。

        V1.8 的 app 是逐帧 sum(hypot(dx,dy))，轮足底盘转向/打滑/odom 抖动
        都会把数值越累越大；V1.11 接入 V1.9 的做法，始终计算当前位置
        到“视觉确认环岛那一刻”的欧氏位移。
        """
        self._approach_distance = 0.0
        self._approach_start_pose = self._odom_snapshot()

    def _update_roundabout_approach_progress(self) -> None:
        if self.phase != "ROUNDABOUT_APPROACH":
            return
        current = self._odom_snapshot()
        if current is None:
            return
        if self._approach_start_pose is None:
            self._approach_start_pose = current
            return
        x, y, _yaw = current
        x0, y0, _yaw0 = self._approach_start_pose
        displacement = math.hypot(x - x0, y - y0)
        # 赛道尺度很小；过大的单次结果视为 odom 跳变，不拿它触发进环。
        if displacement <= 1.50:
            self._approach_distance = displacement

    def _reset_roundabout_progress(self) -> None:
        """正式进环瞬间建立局部零点；计算方式与 icn.py 一致。"""
        self._rb_entry_displacement = 0.0
        self._rb_entry_pose = self._odom_snapshot()

    def _update_roundabout_progress(self) -> None:
        """计算当前位置到进环起点的欧氏位移，不累计每帧路程。

        这样机器人原地转向时，即使 /odom 每帧有轻微抖动，也不会像
        ``sum(hypot(dx,dy))`` 那样越积越大。
        """
        if self.phase != "ROUNDABOUT":
            return
        current = self._odom_snapshot()
        if current is None:
            return
        if self._rb_entry_pose is None:
            self._rb_entry_pose = current
            return
        x, y, _yaw = current
        x0, y0, _yaw0 = self._rb_entry_pose
        displacement = math.hypot(x - x0, y - y0)
        # 极端 odom 突跳不让它直接结束环岛。赛道尺度很小，2m 已明显异常。
        if displacement <= 2.0:
            self._rb_entry_displacement = displacement

    def _reset_crosswalk_approach_progress(self) -> None:
        self._crosswalk_approach_distance = 0.0
        self._crosswalk_approach_start_pose = self._odom_snapshot()

    def _update_crosswalk_approach_progress(self) -> None:
        if self.phase != "CROSSWALK_APPROACH":
            return
        current = self._odom_snapshot()
        if current is None:
            return
        if self._crosswalk_approach_start_pose is None:
            self._crosswalk_approach_start_pose = current
            return
        x, y, _yaw = current
        x0, y0, _yaw0 = self._crosswalk_approach_start_pose
        distance = math.hypot(x - x0, y - y0)
        if distance <= 2.0:
            self._crosswalk_approach_distance = distance

    def _reset_finish_pass_progress(self) -> None:
        self._finish_pass_distance = 0.0
        self._finish_pass_start_pose = self._odom_snapshot()

    def _update_finish_pass_progress(self) -> None:
        if self.phase != "FINISH_PASS":
            return
        current = self._odom_snapshot()
        if current is None:
            return
        if self._finish_pass_start_pose is None:
            self._finish_pass_start_pose = current
            return
        x, y, _yaw = current
        x0, y0, _yaw0 = self._finish_pass_start_pose
        distance = math.hypot(x - x0, y - y0)
        if distance <= 2.0:
            self._finish_pass_distance = distance

    # ------------------------------ step jump -------------------------------
    @staticmethod
    def _step_state_active_name(state: str) -> bool:
        return state in (
            "LOCKED",
            "APPROACH",
            "JUMP_CONFIRM",
            "JUMP",
            "LAND",
            "RECOVER",
            "STOPPED",
            "CALIB_HOLD",
        )

    def _step_active(self) -> bool:
        return self._step_state_active_name(self._step_state)

    def _set_step_state(self, state: str, reason: str) -> None:
        if self._step_state == state:
            return
        old = self._step_state
        self._step_state = state
        self._step_stop_reason = reason
        self.get_logger().warn(
            f"台阶状态：{step_text(old)} -> {step_text(state)}：{reason}"
        )

    def _step_odom_fresh(self, now: float) -> bool:
        return (
            self._odom_snapshot() is not None
            and self._last_odom_time > 0.0
            and now - self._last_odom_time <= STEP_ODOM_TIMEOUT_SECONDS
        )

    def _step_detection_fresh(self, now: float) -> bool:
        return (
            self._latest_step_detection is not None
            and now - self._latest_step_detection_time
            <= STEP_DETECTION_TIMEOUT_SECONDS
        )

    def _step_arm_window_ready(self) -> Tuple[bool, str]:
        """限制台阶接管发生在每圈固定赛道路段，避免2号点弯中早抢控制。"""
        if (
            not STEP_ARM_SEEK_WINDOW_ENABLED
            or self.phase != "SEEK_ROUNDABOUT"
        ):
            return True, "zone_ok"

        seek_distance = float(self._seek_distance)
        if seek_distance < STEP_ARM_MIN_SEEK_DISTANCE_M:
            return (
                False,
                f"step_zone_before_min seek={seek_distance:.2f}<"
                f"{STEP_ARM_MIN_SEEK_DISTANCE_M:.2f}",
            )
        if (
            STEP_ARM_MAX_SEEK_DISTANCE_M > 0.0
            and seek_distance > STEP_ARM_MAX_SEEK_DISTANCE_M
        ):
            return (
                False,
                f"step_zone_after_max seek={seek_distance:.2f}>"
                f"{STEP_ARM_MAX_SEEK_DISTANCE_M:.2f}",
            )
        return True, f"zone_ok seek={seek_distance:.2f}"

    def _filtered_step_distance(self, now: float) -> float:
        """返回最近可信台阶距离的中值，抑制0.79->0.65这种单帧跳变。"""
        values = [
            float(d)
            for ts, d in self._step_depth_history
            if now - float(ts) <= STEP_FIXED_DEPTH_FILTER_MAX_AGE_SECONDS
            and math.isfinite(float(d))
        ]
        if not values:
            return float("nan")
        return float(np.median(np.asarray(values, dtype=float)))

    def _start_fixed_reverse_recovery(
        self,
        now: float,
        reason: str,
        current_distance: float,
    ) -> bool:
        """启动低速倒车，把台阶重新放回统一冲刺起点之外。"""
        pose = self._odom_snapshot()
        if pose is None:
            self._step_align_last_reason = "fixed_reverse_odom_missing"
            return False

        self._step_fixed_reverse_active = True
        self._step_fixed_reverse_start_pose = pose
        self._step_fixed_reverse_traveled = 0.0
        self._step_fixed_reverse_target_yaw = float(pose[2])
        self._step_fixed_reverse_start_time = now
        self._step_fixed_reverse_reacquire_frames = 0
        self._step_fixed_start_confirm_frames = 0
        self._step_fixed_too_close_frames = 0

        if math.isfinite(current_distance):
            needed = (
                STEP_FIXED_REVERSE_REACQUIRE_DISTANCE_M
                - float(current_distance)
                + STEP_FIXED_REVERSE_EXTRA_MARGIN_M
            )
        else:
            needed = STEP_FIXED_REVERSE_MIN_TARGET_M
        self._step_fixed_reverse_target_distance = clamp(
            needed,
            STEP_FIXED_REVERSE_MIN_TARGET_M,
            STEP_FIXED_REVERSE_MAX_TARGET_M,
        )
        self._step_waiting_alignment = True
        self._step_align_last_reason = (
            f"fixed_reverse_start {reason} d={current_distance:.3f}"
        )
        self.get_logger().warn(
            "STEP_START_NORMALIZE_REVERSE: "
            f"reason={reason} current_d={current_distance:.3f}m；"
            f"当前需要后退，开始以{STEP_FIXED_REVERSE_SPEED_M_S:.3f}m/s倒车；"
            f"本次目标只后退约{self._step_fixed_reverse_target_distance:.3f}m，"
            f"不再强行追求{STEP_FIXED_START_REVERSE_TO_M:.2f}m"
        )
        return False

    def _fixed_reverse_travel(self) -> float:
        """返回从开始倒车起，沿开始朝向向后的正距离。"""
        current = self._odom_snapshot()
        start = self._step_fixed_reverse_start_pose
        if current is None or start is None:
            return 0.0
        x, y, _yaw = current
        x0, y0, yaw0 = start
        forward_projection = (
            (x - x0) * math.cos(yaw0)
            + (y - y0) * math.sin(yaw0)
        )
        return max(0.0, -forward_projection)

    def _fixed_yaw_hold_angular(self) -> float:
        current = self._odom_snapshot()
        target = self._step_fixed_reverse_target_yaw
        if target is None:
            target = self._step_fixed_takeover_yaw
        if current is None or target is None:
            return 0.0
        yaw_error = wrap_pi(float(target) - float(current[2]))
        self._step_yaw_error = yaw_error
        return clamp(
            STEP_FIXED_YAW_HOLD_KP * yaw_error,
            -STEP_FIXED_YAW_HOLD_MAX_ANGULAR,
            STEP_FIXED_YAW_HOLD_MAX_ANGULAR,
        )

    def _lock_fixed_sprint_from_here(
        self,
        now: float,
        distance_for_log: float,
        reason: str,
    ) -> bool:
        pose = self._odom_snapshot()
        if pose is None:
            self._step_align_last_reason = "fixed_sprint_odom_missing"
            return False

        self._step_detected_distance = float(distance_for_log)
        self._step_lock_pose = pose
        self._step_lock_time = now

        target_yaw, yaw_source = self._step_heading_snapshot(now)
        self._step_target_yaw = target_yaw
        self._step_yaw_source = yaw_source
        self._step_yaw_error = 0.0

        self._step_target_travel = STEP_FIXED_SPRINT_DISTANCE_M
        self._step_odom_traveled = 0.0
        self._step_commanded_traveled = 0.0
        self._step_trigger_traveled = 0.0
        self._step_remaining = STEP_FIXED_SPRINT_DISTANCE_M
        self._step_approach_start_time = 0.0
        self._step_last_approach_time = 0.0

        self._step_fixed_bridge_active = False
        self._step_waiting_alignment = False
        self._step_align_frames = 0
        self._step_align_last_reason = reason

        self._set_step_state(
            "LOCKED",
            f"到达固定冲刺起点：{reason}；准备直线冲刺{STEP_FIXED_SPRINT_DISTANCE_M:.2f}m后跳跃",
        )
        self.get_logger().warn(
            "FIXED_SPRINT_LOCK: source=%s depth_ref=%.3fm sprint=%.3fm v=%.2fm/s w=0"
            % (
                reason,
                distance_for_log,
                STEP_FIXED_SPRINT_DISTANCE_M,
                STEP_APPROACH_SPEED_M_S,
            )
        )
        return True

    def _lock_depth_approach_from_here(
        self,
        now: float,
        distance: float,
        detection: StepDetection,
        reason: str,
    ) -> bool:
        pose = self._odom_snapshot()
        if pose is None:
            self._step_align_last_reason = "depth_lock_odom_missing"
            return False

        target_travel = max(0.0, float(distance) - STEP_STOP_DISTANCE_M)

        self._step_detected_distance = float(distance)
        self._step_locked_height = float(detection.height)
        self._step_locked_confidence = float(detection.confidence)
        self._step_lock_pose = pose
        self._step_lock_time = now

        target_yaw, yaw_source = self._step_heading_snapshot(now)
        self._step_target_yaw = target_yaw
        self._step_yaw_source = yaw_source
        self._step_yaw_error = 0.0

        self._step_target_travel = target_travel
        self._step_odom_traveled = 0.0
        self._step_commanded_traveled = 0.0
        self._step_trigger_traveled = 0.0
        self._step_remaining = target_travel
        self._step_approach_start_time = 0.0
        self._step_last_approach_time = 0.0

        self._step_fixed_takeover_latched = True
        self._step_fixed_last_valid_distance = float(distance)
        self._step_fixed_bridge_active = False
        self._step_waiting_alignment = False
        self._step_align_frames = 0
        self._step_fixed_candidate_frames = 0
        self._step_fixed_start_confirm_frames = 0
        self._step_fixed_too_close_frames = 0
        self._step_align_last_reason = reason

        self._set_step_state(
            "LOCKED",
            f"台阶姿态已摆正，按深度锁定：还需前进{target_travel:.2f}m",
        )
        self.get_logger().warn(
            "DEPTH_POSE_LOCK: reason=%s depth=%.3fm stop=%.3fm "
            "target=%.3fm h=%.3fm conf=%.2f；台阶前沿已摆正，不再等待黑线"
            % (
                reason,
                distance,
                STEP_STOP_DISTANCE_M,
                target_travel,
                float(detection.height),
                float(detection.confidence),
            )
        )
        return True

    def _release_false_fixed_takeover_if_needed(self, now: float) -> bool:
        """远处疑似台阶接管后若检测持续丢失，自动释放误接管。

        返回 True 表示本周期已释放，应恢复普通巡线。
        """
        if not (STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched):
            return False

        if self._step_fixed_lost_since <= 0.0:
            self._step_fixed_lost_since = now
            return False

        lost_for = now - self._step_fixed_lost_since
        last_d = self._step_fixed_last_valid_distance
        far_takeover = math.isfinite(last_d) and last_d > STEP_FIXED_RELEASE_IF_LOST_ABOVE_M

        if far_takeover and lost_for >= STEP_FIXED_RELEASE_LOST_SECONDS:
            self.get_logger().warn(
                "STEP_FIXED_FALSE_TAKEOVER_RELEASE: "
                f"上次有效距离={last_d:.3f}m，深度已丢失{lost_for:.2f}s；"
                "判断为远处误触发，释放台阶接管并恢复普通巡线"
            )
            self._step_fixed_takeover_latched = False
            self._step_fixed_takeover_logged = False
            self._step_fixed_candidate_frames = 0
            self._step_fixed_last_valid_distance = float("nan")
            self._step_fixed_lost_since = 0.0
            self._step_fixed_takeover_yaw = None
            self._step_fixed_reverse_active = False
            self._step_fixed_reverse_start_pose = None
            self._step_fixed_reverse_traveled = 0.0
            self._step_fixed_reverse_target_yaw = None
            self._step_fixed_reverse_start_time = 0.0
            self._step_fixed_reverse_reacquire_frames = 0
            self._step_fixed_reverse_target_distance = 0.0
            self._step_fixed_start_confirm_frames = 0
            self._step_fixed_too_close_frames = 0
            self._step_depth_history.clear()
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "fixed_false_takeover_released"
            return True

        return False

    def _try_lock_step(self, result: VisionResult, now: float) -> bool:
        """看到台阶后先用视觉把车摆正，再锁定“摆正后的当前距离”。

        录像 15:40 的失败点：STEP 被锁定时
        error=-0.23, near=-0.15, mid=-0.40, far=-0.52，
        说明机器人仍明显处在左弯、车身没有正对台阶。
        旧版此时马上切到 0.50m/s 纯直行，所以保持这个斜角冲向台阶并跳出赛道。

        V1.25 保留比赛程序已有的“弯道禁止接管 + 视觉摆正”门槛；
        一旦真正锁定台阶，后续接近/起跳动作不再自行改造，
        直接使用已在 step_approach_test.py 中实车验证成功的带速跳跃流程。
        """
        if not STEP_ENABLED or self._step_done_this_lap or self._step_state != "IDLE":
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            return False

        if self.phase not in STEP_TRIGGER_PHASES:
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "phase_not_allowed"
            return False

        zone_ready, zone_reason = self._step_arm_window_ready()
        if not zone_ready:
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_fixed_candidate_frames = 0
            self._step_fixed_start_confirm_frames = 0
            self._step_align_last_reason = zone_reason
            if self._step_detection_fresh(now) and now - self._last_step_gate_log >= STATUS_LOG_PERIOD:
                det = self._latest_step_detection
                d = float(det.distance) if det is not None else float("nan")
                h = float(det.height) if det is not None else float("nan")
                c = float(det.confidence) if det is not None else 0.0
                self.get_logger().info(
                    "STEP_GATE_HOLD: %s phase=%s lap=%d seek=%.2fm "
                    "det=%d d=%.3fm h=%.3fm conf=%.2f candE=%.1f"
                    % (
                        zone_reason,
                        self.phase,
                        self.lap_count,
                        self._seek_distance,
                        int(bool(det.detected)) if det is not None else 0,
                        d,
                        h,
                        c,
                        self._step_race_candidate_evidence,
                    )
                )
                self._last_step_gate_log = now
            return False

        # V1.43：近处丢深度后进入“倒车找回台阶”恢复。
        # 倒车期间不依赖黑线继续向前，只有检测重新稳定且距离退回>=0.75m才退出恢复。
        if (
            STEP_FIXED_SPRINT_ENABLED
            and STEP_FIXED_REVERSE_RECOVERY_ENABLED
            and self._step_fixed_reverse_active
        ):
            if not self._step_odom_fresh(now):
                self._step_waiting_alignment = True
                self._step_align_last_reason = "fixed_reverse_wait_odom"
                return False

            self._step_fixed_reverse_traveled = self._fixed_reverse_travel()
            reverse_age = now - self._step_fixed_reverse_start_time

            detection_fresh = self._step_detection_fresh(now)
            detection_now = self._latest_step_detection
            filtered_d = self._filtered_step_distance(now)

            reacquired = (
                detection_fresh
                and detection_now is not None
                and math.isfinite(filtered_d)
                and filtered_d >= STEP_FIXED_REVERSE_REACQUIRE_DISTANCE_M
            )

            if reacquired:
                self._step_fixed_reverse_reacquire_frames += 1
            else:
                self._step_fixed_reverse_reacquire_frames = 0

            depth_reacquired = (
                self._step_fixed_reverse_reacquire_frames
                >= STEP_FIXED_REVERSE_REACQUIRE_FRAMES
            )
            odom_target_reached = (
                self._step_fixed_reverse_traveled
                >= self._step_fixed_reverse_target_distance
            )
            safety_limit_reached = (
                self._step_fixed_reverse_traveled
                    >= STEP_FIXED_REVERSE_MAX_DISTANCE_M
                or reverse_age >= STEP_FIXED_REVERSE_MAX_SECONDS
            )

            if depth_reacquired or odom_target_reached or safety_limit_reached:
                d = (
                    filtered_d
                    if math.isfinite(filtered_d)
                    else self._step_fixed_last_valid_distance
                )
                if math.isfinite(d):
                    self._step_fixed_last_valid_distance = d

                traveled = self._step_fixed_reverse_traveled
                target = self._step_fixed_reverse_target_distance

                self._step_fixed_lost_since = 0.0
                self._step_fixed_reverse_active = False
                self._step_fixed_reverse_start_pose = None
                self._step_fixed_reverse_traveled = 0.0
                self._step_fixed_reverse_target_yaw = None
                self._step_fixed_reverse_start_time = 0.0
                self._step_fixed_reverse_reacquire_frames = 0
                self._step_fixed_reverse_target_distance = 0.0
                self._step_fixed_start_confirm_frames = 0
                self._step_fixed_too_close_frames = 0
                self._step_waiting_alignment = True

                why = (
                    "depth"
                    if depth_reacquired
                    else "odom_target"
                    if odom_target_reached
                    else "safety_fallback"
                )
                self._step_align_last_reason = (
                    f"fixed_reverse_done {why} d={d:.3f}"
                    if math.isfinite(d)
                    else f"fixed_reverse_done {why}"
                )
                self.get_logger().warn(
                    "STEP_REVERSE_DONE: "
                    f"reason={why} back={traveled:.3f}/{target:.3f}m "
                    f"filtered_d={d:.3f}m；停止倒车，先原地重新获取深度/再靠近"
                    if math.isfinite(d)
                    else
                    "STEP_REVERSE_DONE: "
                    f"reason={why} back={traveled:.3f}/{target:.3f}m；"
                    "停止倒车，先原地重新获取深度"
                )
                return False

            self._step_waiting_alignment = True
            self._step_align_last_reason = (
                f"fixed_reverse_search back={self._step_fixed_reverse_traveled:.3f}/"
                f"{self._step_fixed_reverse_target_distance:.3f}"
            )
            return False

        # V1.37：标定一旦接管，深度瞬时丢失时必须“停车等深度回来”，
        # 不能把 _step_waiting_alignment 清零后重新交给普通巡线，否则会一直向前冲。
        if not self._step_detection_fresh(now):
            if STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched:
                last_d = self._step_fixed_last_valid_distance

                # V1.49：近处深度短暂停顿先停车等待，不再0.5s一超时就倒车。
                if self._step_fixed_lost_since <= 0.0:
                    self._step_fixed_lost_since = now
                lost_for = now - self._step_fixed_lost_since

                if (
                    math.isfinite(last_d)
                    and last_d <= STEP_FIXED_RELEASE_IF_LOST_ABOVE_M
                    and lost_for < STEP_FIXED_NEAR_LOSS_GRACE_SECONDS
                ):
                    self._step_waiting_alignment = True
                    self._step_align_frames = 0
                    self._step_align_last_reason = (
                        f"step_depth_loss_grace {lost_for:.2f}/"
                        f"{STEP_FIXED_NEAR_LOSS_GRACE_SECONDS:.2f}s"
                    )
                    return False

                if (
                    STEP_FIXED_REVERSE_RECOVERY_ENABLED
                    and math.isfinite(last_d)
                    and last_d <= STEP_FIXED_RELEASE_IF_LOST_ABOVE_M
                ):
                    return self._start_fixed_reverse_recovery(
                        now,
                        "near_depth_lost_after_grace",
                        last_d,
                    )

                if self._release_false_fixed_takeover_if_needed(now):
                    return False
                self._step_waiting_alignment = True
                self._step_align_frames = 0
                self._step_align_last_reason = "step_depth_lost_hold"
                return False
            if STEP_SPRINT_CALIBRATION_MODE and self._step_calib_takeover_latched:
                self._step_waiting_alignment = True
                self._step_align_frames = 0
                self._step_align_last_reason = "step_depth_lost_hold"
                return False
            self._step_fixed_candidate_frames = 0
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "no_fresh_detection"
            return False

        detection = self._latest_step_detection
        if detection is None:
            if STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched:
                last_d = self._step_fixed_last_valid_distance
                if (
                    STEP_FIXED_REVERSE_RECOVERY_ENABLED
                    and math.isfinite(last_d)
                    and last_d <= STEP_FIXED_RELEASE_IF_LOST_ABOVE_M
                ):
                    return self._start_fixed_reverse_recovery(
                        now,
                        "near_detector_missing",
                        last_d,
                    )
                if self._release_false_fixed_takeover_if_needed(now):
                    return False
                self._step_waiting_alignment = True
                self._step_align_frames = 0
                self._step_align_last_reason = "step_detector_missing_hold"
                return False

            self._step_fixed_candidate_frames = 0
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "detector_missing"
            return False

        # V1.48：detector 自己 detected=True 当然直接认可；
        # 若还只是 candidate，但 race-side 证据已经积够，也允许进入后续门槛。
        race_candidate_ready = (
            bool(detection.detected)
            or self._step_race_candidate_evidence
                >= STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER
        )

        candidate_fields_valid = (
            math.isfinite(float(detection.distance))
            and math.isfinite(float(detection.height))
            and float(detection.distance) <= STEP_RACE_CANDIDATE_MAX_DISTANCE_M
            and STEP_RACE_CANDIDATE_MIN_HEIGHT_M
                <= float(detection.height) <= STEP_RACE_CANDIDATE_MAX_HEIGHT_M
            and float(detection.confidence)
                >= STEP_RACE_CANDIDATE_MIN_CONFIDENCE
        )

        if not candidate_fields_valid:
            if STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched:
                last_d = self._step_fixed_last_valid_distance

                if self._step_fixed_lost_since <= 0.0:
                    self._step_fixed_lost_since = now
                lost_for = now - self._step_fixed_lost_since

                if (
                    math.isfinite(last_d)
                    and last_d <= STEP_FIXED_RELEASE_IF_LOST_ABOVE_M
                    and lost_for < STEP_FIXED_NEAR_LOSS_GRACE_SECONDS
                ):
                    self._step_waiting_alignment = True
                    self._step_align_frames = 0
                    self._step_align_last_reason = (
                        f"step_candidate_loss_grace {lost_for:.2f}/"
                        f"{STEP_FIXED_NEAR_LOSS_GRACE_SECONDS:.2f}s"
                    )
                    return False

                if (
                    STEP_FIXED_REVERSE_RECOVERY_ENABLED
                    and math.isfinite(last_d)
                    and last_d <= STEP_FIXED_RELEASE_IF_LOST_ABOVE_M
                ):
                    return self._start_fixed_reverse_recovery(
                        now,
                        "near_candidate_lost_after_grace",
                        last_d,
                    )
                if self._release_false_fixed_takeover_if_needed(now):
                    return False
                self._step_waiting_alignment = True
                self._step_align_frames = 0
                self._step_align_last_reason = "step_candidate_lost_hold"
                return False

            self._step_fixed_candidate_frames = 0
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "candidate_fields_invalid"
            return False

        if not race_candidate_ready and not self._step_fixed_takeover_latched:
            self._step_fixed_candidate_frames = 0
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = (
                f"candidate_evidence_wait "
                f"{self._step_race_candidate_evidence:.1f}/"
                f"{STEP_RACE_CANDIDATE_EVIDENCE_TRIGGER:.1f}"
            )
            return False

        raw_distance = float(detection.distance)
        filtered_distance = self._filtered_step_distance(now)
        distance = (
            filtered_distance
            if math.isfinite(filtered_distance)
            else raw_distance
        )
        if not math.isfinite(distance):
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "invalid_distance"
            return False

        if STEP_FIXED_SPRINT_ENABLED:
            self._step_fixed_lost_since = 0.0
            self._step_fixed_last_valid_distance = distance

        # V1.24：深度确认只是“候选”，不能立即抢底盘。
        # 当前仍在明显转弯时直接忽略这次深度候选，保持原来的正常循迹。
        # 等车通过弯道、视觉航向差变小后，如果台阶确实存在，深度检测会再次确认。
        near_mid_delta = abs(float(result.mid_error) - float(result.near_error))
        mid_far_delta = abs(float(result.far_error) - float(result.mid_error))
        near_far_delta = abs(float(result.far_error) - float(result.near_error))
        center_error = (
            float(result.near_error)
            + float(result.mid_error)
            + float(result.far_error)
        ) / 3.0
        last_turning = abs(float(self._last_command[1]))

        strict_arm_ready = (
            bool(result.line_valid)
            and result.confidence >= STEP_ARM_MIN_CONFIDENCE
            and result.valid_bands >= STEP_ARM_MIN_VALID_BANDS
            and near_mid_delta <= STEP_ARM_MAX_HEADING_DELTA
            and mid_far_delta <= STEP_ARM_MAX_MID_FAR_DELTA
            and near_far_delta <= STEP_ARM_MAX_NEAR_FAR_DELTA
            and abs(center_error) <= STEP_ARM_MAX_CENTER_ERROR
            and last_turning <= STEP_ARM_MAX_LAST_ANGULAR
        )

        # 只有深度候选证据已经达到 race_candidate_ready 时才允许 soft arm。
        # 235857 第二圈关键画面约41.5s：conf≈0.58，但 hd≈0.27，
        # 原门槛0.24会直接忽略；这时真实台阶已经很近，应该先接管再慢速摆正。
        soft_arm_ready = (
            race_candidate_ready
            and bool(result.line_valid)
            and result.confidence >= STEP_SOFT_ARM_MIN_CONFIDENCE
            and result.valid_bands >= STEP_SOFT_ARM_MIN_VALID_BANDS
            and near_mid_delta <= STEP_SOFT_ARM_MAX_HEADING_DELTA
            and mid_far_delta <= STEP_SOFT_ARM_MAX_MID_FAR_DELTA
            and near_far_delta <= STEP_SOFT_ARM_MAX_NEAR_FAR_DELTA
            and abs(center_error) <= STEP_SOFT_ARM_MAX_CENTER_ERROR
            and last_turning <= STEP_SOFT_ARM_MAX_LAST_ANGULAR
        )

        step_arm_ready = strict_arm_ready or soft_arm_ready
        force_step_align = (
            race_candidate_ready
            and distance <= STEP_FORCE_ALIGN_ON_CONFIRMED_DISTANCE_M
            and float(detection.confidence) >= STEP_FIXED_TAKEOVER_MIN_CONFIDENCE
        )
        depth_pose_valid = (
            bool(getattr(detection, "pose_valid", False))
            and math.isfinite(float(getattr(detection, "edge_angle_rad", float("nan"))))
            and math.isfinite(float(getattr(detection, "lateral_offset_m", float("nan"))))
        )
        depth_pose_ready = (
            force_step_align
            and depth_pose_valid
            and distance <= STEP_DEPTH_POSE_JUMP_POINT_DISTANCE_M
        )
        if depth_pose_ready:
            edge_angle = float(detection.edge_angle_rad)
            lateral_offset = float(detection.lateral_offset_m)
            depth_aligned = (
                abs(edge_angle) <= STEP_DEPTH_ALIGN_MAX_EDGE_ANGLE_RAD
                and abs(lateral_offset) <= STEP_DEPTH_ALIGN_MAX_LATERAL_OFFSET_M
            )
            if depth_aligned:
                self._step_align_frames += 1
                self._step_align_last_reason = (
                    f"depth_pose_aligned "
                    f"{self._step_align_frames}/{STEP_DEPTH_ALIGN_CONFIRM_FRAMES} "
                    f"angle={math.degrees(edge_angle):+.1f}deg "
                    f"lat={lateral_offset:+.3f}"
                )
                if self._step_align_frames >= STEP_DEPTH_ALIGN_CONFIRM_FRAMES:
                    if STEP_FIXED_SPRINT_ENABLED:
                        return self._lock_fixed_sprint_from_here(
                            now,
                            distance,
                            "depth_pose_aligned_fixed_sprint",
                        )
                    return self._lock_depth_approach_from_here(
                        now,
                        distance,
                        detection,
                        "depth_pose_aligned",
                    )
                self._step_waiting_alignment = True
                return False

            self._step_align_frames = 0
            self._step_waiting_alignment = True
            self._step_align_last_reason = (
                f"depth_pose_aligning "
                f"angle={math.degrees(edge_angle):+.1f}deg "
                f"lat={lateral_offset:+.3f}"
            )
            return False

        if not step_arm_ready and not (
            STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched
        ):
            if force_step_align:
                self._step_waiting_alignment = True
                self._step_align_frames = 0
                self._step_align_last_reason = (
                    f"step_confirmed_slow_align d={distance:.3f}"
                )
                if now - self._last_step_gate_log >= STATUS_LOG_PERIOD:
                    self.get_logger().info(
                        "台阶已确认但车头还没正：先低速摆正，不再继续普通巡线；"
                        f"d={distance:.2f}m seek={self._seek_distance:.2f}m"
                    )
                    self._last_step_gate_log = now
                return False

            if STEP_FIXED_SPRINT_ENABLED:
                self._step_fixed_candidate_frames = 0
            self._step_waiting_alignment = False
            self._step_align_frames = 0
            if not result.line_valid:
                self._step_align_last_reason = "step_candidate_ignore_line_invalid"
            elif near_mid_delta > STEP_ARM_MAX_HEADING_DELTA:
                self._step_align_last_reason = (
                    f"step_candidate_ignore_near_mid {near_mid_delta:.3f}"
                )
            elif mid_far_delta > STEP_ARM_MAX_MID_FAR_DELTA:
                self._step_align_last_reason = (
                    f"step_candidate_ignore_mid_far {mid_far_delta:.3f}"
                )
            elif near_far_delta > STEP_ARM_MAX_NEAR_FAR_DELTA:
                self._step_align_last_reason = (
                    f"step_candidate_ignore_near_far {near_far_delta:.3f}"
                )
            elif abs(center_error) > STEP_ARM_MAX_CENTER_ERROR:
                self._step_align_last_reason = (
                    f"step_candidate_ignore_center {center_error:+.3f}"
                )
            elif last_turning > STEP_ARM_MAX_LAST_ANGULAR:
                self._step_align_last_reason = (
                    f"step_candidate_ignore_turning w={last_turning:.3f}"
                )
            else:
                self._step_align_last_reason = "step_candidate_ignore_weak_line"
            if now - self._last_step_gate_log >= STATUS_LOG_PERIOD:
                self.get_logger().info(
                    "STEP_GATE_HOLD: %s phase=%s lap=%d seek=%.2fm "
                    "d=%.3fm h=%.3fm conf=%.2f candE=%.1f "
                    "N/M/F=%+.3f/%+.3f/%+.3f dNM/MF/NF=%.3f/%.3f/%.3f "
                    "center=%+.3f last_w=%.3f"
                    % (
                        self._step_align_last_reason,
                        self.phase,
                        self.lap_count,
                        self._seek_distance,
                        distance,
                        float(detection.height),
                        float(detection.confidence),
                        self._step_race_candidate_evidence,
                        float(result.near_error),
                        float(result.mid_error),
                        float(result.far_error),
                        near_mid_delta,
                        mid_far_delta,
                        near_far_delta,
                        center_error,
                        last_turning,
                    )
                )
                self._last_step_gate_log = now
            return False

        # 只有“深度确认 + 当前不是明显大弯”才允许进入跳台摆正流程。
        self._step_waiting_alignment = True

        if STEP_SPRINT_CALIBRATION_MODE:
            self._step_calib_takeover_latched = True
            if not self._step_calib_takeover_logged:
                self._step_calib_takeover_logged = True
                self.get_logger().warn(
                    "SPRINT_CALIB_TAKEOVER: 台阶标定已接管；"
                    "从现在起即使深度瞬时丢失也只会停车等待，不会恢复普通巡线前冲"
                )

        if STEP_FIXED_SPRINT_ENABLED:
            height = float(detection.height)
            confidence = float(detection.confidence)
            candidate_quality_ok = (
                distance <= STEP_FIXED_TAKEOVER_MAX_DISTANCE_M
                and math.isfinite(height)
                and STEP_FIXED_TAKEOVER_MIN_HEIGHT_M
                    <= height <= STEP_FIXED_TAKEOVER_MAX_HEIGHT_M
                and confidence >= STEP_FIXED_TAKEOVER_MIN_CONFIDENCE
            )

            if not self._step_fixed_takeover_latched:
                if not candidate_quality_ok:
                    self._step_fixed_candidate_frames = 0
                    self._step_waiting_alignment = False
                    self._step_align_last_reason = (
                        f"fixed_candidate_reject d={distance:.3f} "
                        f"h={height:.3f} conf={confidence:.2f}"
                    )
                    return False

                self._step_fixed_candidate_frames += 1
                self._step_align_last_reason = (
                    f"fixed_candidate_confirm "
                    f"{self._step_fixed_candidate_frames}/"
                    f"{STEP_FIXED_TAKEOVER_CONFIRM_FRAMES}"
                )
                if self._step_fixed_candidate_frames < STEP_FIXED_TAKEOVER_CONFIRM_FRAMES:
                    self._step_waiting_alignment = False
                    return False

                self._step_fixed_takeover_latched = True
                self._step_fixed_candidate_frames = 0
                pose_now = self._odom_snapshot()
                if pose_now is not None:
                    self._step_fixed_takeover_yaw = float(pose_now[2])
                self._step_locked_height = float(detection.height)
                self._step_locked_confidence = float(detection.confidence)

            if not self._step_fixed_takeover_logged:
                self._step_fixed_takeover_logged = True
                self.get_logger().warn(
                    "STEP_FIXED_TAKEOVER_CONFIRMED: 台阶候选已连续确认，"
                    f"d={distance:.3f}m h={height:.3f}m conf={confidence:.2f}；"
                    "正式接管，摆正靠近到0.70m后固定冲刺并跳跃"
                )

        if not self._step_odom_fresh(now):
            self._step_align_frames = 0
            self._step_align_last_reason = "odom_not_fresh"
            return False

        heading_delta = abs(float(result.mid_error) - float(result.near_error))
        heading_yaw, heading_source = self._step_heading_snapshot(now)

        # ------------------------------------------------------------------
        # V1.38 正式固定冲刺：
        # 使用已经验证能把车头摆正的低速对线控制持续靠近；
        # 进入 0.70±0.015m 统一窗口并连续确认后，才作为固定冲刺起点。
        # 之后完全交给原有 APPROACH/JUMP 状态机：
        #   v=0.50, w=0；名义target=0.605m，但提前0.055m触发 /cmd_jump=True
        # ------------------------------------------------------------------
        if STEP_FIXED_SPRINT_ENABLED:
            near_mid_delta = abs(float(result.mid_error) - float(result.near_error))
            center_error = (
                float(result.near_error)
                + float(result.mid_error)
                + float(result.far_error)
            ) / 3.0

            start_hi = (
                STEP_FIXED_SPRINT_START_DEPTH_M
                + STEP_FIXED_START_TOLERANCE_M
            )
            start_lo = (
                STEP_FIXED_SPRINT_START_DEPTH_M
                - STEP_FIXED_START_TOLERANCE_M
            )

            # 还在统一冲刺起点之外：继续低速摆正靠近。
            if distance > start_hi:
                self._step_fixed_start_confirm_frames = 0
                self._step_fixed_too_close_frames = 0
                self._step_align_last_reason = (
                    f"fixed_sprint_approach depth={distance:.3f}>"
                    f"{start_hi:.3f} "
                    f"center={center_error:+.3f} hd={near_mid_delta:.3f}"
                )
                return False

            # V1.49：低于窗口不能再“一帧就倒车”。
            # 先原地停住连续确认；录像001646中0.79->0.65就是单帧/短时抖动，
            # 旧版因此触发了不必要的倒车。
            if distance < start_lo:
                self._step_fixed_start_confirm_frames = 0
                self._step_fixed_too_close_frames += 1
                self._step_align_last_reason = (
                    f"fixed_start_too_close_confirm "
                    f"{self._step_fixed_too_close_frames}/"
                    f"{STEP_FIXED_TOO_CLOSE_CONFIRM_FRAMES} "
                    f"filtered_d={distance:.3f} raw_d={raw_distance:.3f}"
                )
                if (
                    self._step_fixed_too_close_frames
                    < STEP_FIXED_TOO_CLOSE_CONFIRM_FRAMES
                ):
                    return False
                self._step_fixed_too_close_frames = 0
                return self._start_fixed_reverse_recovery(
                    now,
                    "start_too_close_confirmed",
                    distance,
                )

            self._step_fixed_too_close_frames = 0

            # 进入0.70±1.5cm窗口后连续确认3帧，过滤深度抖动。
            self._step_fixed_start_confirm_frames += 1
            self._step_align_last_reason = (
                f"fixed_start_confirm "
                f"{self._step_fixed_start_confirm_frames}/"
                f"{STEP_FIXED_START_CONFIRM_FRAMES} "
                f"d={distance:.3f}"
            )
            if (
                self._step_fixed_start_confirm_frames
                < STEP_FIXED_START_CONFIRM_FRAMES
            ):
                return False

            self._step_fixed_start_confirm_frames = 0
            return self._lock_fixed_sprint_from_here(
                now,
                distance,
                "normalized_depth_start",
            )

        # ------------------------------------------------------------------
        # V1.37 标定模式：彻底与正式跳跃的“连续摆正后才LOCK”逻辑分开。
        #
        # 目标只有两个：
        # 1) 台阶一旦接管，持续低速边走边修正；
        # 2) 深度 <= 0.70m 时无条件停车并打印，绝不因为姿态条件没通过而越过目标点。
        #
        # 姿态数据仍打印出来给我们调“靠近过程中的摆正控制”，但不再阻止停车。
        # ------------------------------------------------------------------
        if STEP_SPRINT_CALIBRATION_MODE:
            near_mid_delta = abs(float(result.mid_error) - float(result.near_error))
            mid_far_delta = abs(float(result.far_error) - float(result.mid_error))
            near_far_delta = abs(float(result.far_error) - float(result.near_error))
            center_error = (
                float(result.near_error)
                + float(result.mid_error)
                + float(result.far_error)
            ) / 3.0

            if distance > STEP_CALIB_TARGET_DEPTH_M:
                self._step_align_last_reason = (
                    f"calib_approach depth={distance:.3f}>"
                    f"{STEP_CALIB_TARGET_DEPTH_M:.3f} "
                    f"center={center_error:+.3f} hd={near_mid_delta:.3f}"
                )
                return False

            # 到达固定深度：先锁存，再进入 CALIB_HOLD。
            self._step_calib_at_target_depth = True
            self._step_calib_pose = self._odom_snapshot()
            self._step_calib_depth_distance = distance
            self._step_calib_lap_travel = float(self._seek_distance)

            pose = self._step_calib_pose
            if pose is None:
                self._step_align_last_reason = "calib_target_depth_odom_missing"
                return False

            x, y, yaw = pose
            suggested_sprint = max(0.0, distance - STEP_STOP_DISTANCE_M)

            self._step_waiting_alignment = False
            self._step_align_frames = 0
            self._step_align_last_reason = "sprint_calibration_target_reached"
            self._set_step_state(
                "CALIB_HOLD",
                "到达固定0.70m深度：无条件停车标定",
            )

            if not self._step_calib_printed:
                self._step_calib_printed = True
                banner = (
                    "\n"
                    "============================================================\n"
                    "[SPRINT_CALIB_TARGET_REACHED] 已到固定深度，立即停车。\n"
                    f"  lap                 = {self.lap_count + 1}/{TOTAL_LAPS}\n"
                    f"  lap_relative_travel = {self._seek_distance:.3f} m\n"
                    f"  target_depth        = {STEP_CALIB_TARGET_DEPTH_M:.3f} m\n"
                    f"  depth_step_distance = {distance:.3f} m\n"
                    f"  front_expected      = 约0.59m（按前三次实测关系，仅作对照）\n"
                    f"  odom_x/y/yaw        = {x:.3f}, {y:.3f}, {yaw:+.3f} rad\n"
                    f"  N/M/F               = {result.near_error:+.3f} / "
                    f"{result.mid_error:+.3f} / {result.far_error:+.3f}\n"
                    f"  center_error        = {center_error:+.3f}\n"
                    f"  dNM/dMF/dNF         = {near_mid_delta:.3f} / "
                    f"{mid_far_delta:.3f} / {near_far_delta:.3f}\n"
                    f"  line_conf/bands     = {result.confidence:.3f} / {result.valid_bands}\n"
                    f"  step_height/conf    = {float(detection.height):.3f} m / "
                    f"{float(detection.confidence):.3f}\n"
                    "------------------------------------------------------------\n"
                    "本版先验证两件事：①一定会在0.70m停；②此时车头姿态是否改善。\n"
                    "============================================================"
                )
                print(banner, flush=True)
                self.get_logger().warn(
                    "SPRINT_CALIB_TARGET_REACHED: target=%.3fm depth=%.3fm "
                    "center=%+.3f dNM/dMF/dNF=%.3f/%.3f/%.3f"
                    % (
                        STEP_CALIB_TARGET_DEPTH_M,
                        distance,
                        center_error,
                        near_mid_delta,
                        mid_far_delta,
                        near_far_delta,
                    )
                )
            return True

        aligned = (
            bool(result.line_valid)
            and result.confidence >= STEP_ALIGN_MIN_CONFIDENCE
            and result.valid_bands >= STEP_ALIGN_MIN_VALID_BANDS
            and abs(result.near_error) <= STEP_ALIGN_MAX_NEAR_ABS
            and heading_delta <= STEP_ALIGN_MAX_HEADING_DELTA
            and heading_yaw is not None
        )

        near_step_force_lock = (
            distance <= STEP_ALIGN_FORCE_LOCK_DISTANCE_M
            and bool(result.line_valid)
            and result.confidence >= STEP_ALIGN_MIN_CONFIDENCE
            and result.valid_bands >= STEP_ALIGN_MIN_VALID_BANDS
            and abs(result.near_error) <= STEP_ALIGN_MAX_NEAR_ABS
            and heading_delta <= STEP_ALIGN_FORCE_MAX_HEADING_DELTA
            and heading_yaw is not None
        )

        if aligned:
            self._step_align_frames += 1
            self._step_align_last_reason = "heading_ok"
        elif near_step_force_lock:
            self._step_align_frames = STEP_ALIGN_CONFIRM_FRAMES
            self._step_align_last_reason = "near_step_force_lock"
        else:
            self._step_align_frames = 0
            if not result.line_valid:
                self._step_align_last_reason = "line_invalid"
            elif result.valid_bands < STEP_ALIGN_MIN_VALID_BANDS:
                self._step_align_last_reason = "too_few_bands"
            elif result.confidence < STEP_ALIGN_MIN_CONFIDENCE:
                self._step_align_last_reason = "low_conf"
            elif abs(result.near_error) > STEP_ALIGN_MAX_NEAR_ABS:
                self._step_align_last_reason = "line_too_far_side"
            elif heading_delta > STEP_ALIGN_FORCE_MAX_HEADING_DELTA:
                self._step_align_last_reason = "heading_too_slanted"
            elif heading_yaw is None:
                self._step_align_last_reason = "odom_heading_missing"
            else:
                self._step_align_last_reason = "heading_wait"
            return False

        if self._step_align_frames < STEP_ALIGN_CONFIRM_FRAMES:
            return False

        pose = self._odom_snapshot()
        if pose is None:
            self._step_align_frames = 0
            self._step_align_last_reason = "odom_missing_at_lock"
            return False

        # 关键：不是使用“最早看到台阶时”的旧距离，
        # 而是等车真正摆正后，重新读取这一帧最新的深度距离。
        self._step_detected_distance = distance
        self._step_locked_height = float(detection.height)
        self._step_locked_confidence = float(detection.confidence)
        self._step_lock_pose = pose
        self._step_lock_time = now

        # 保留锁定瞬间 yaw 仅用于调试显示；成功测试版 APPROACH 不使用 yaw 修正。
        target_yaw, yaw_source = self._step_heading_snapshot(now)
        self._step_target_yaw = target_yaw
        self._step_yaw_source = yaw_source
        self._step_yaw_error = 0.0
        self._step_target_travel = max(0.0, distance - STEP_STOP_DISTANCE_M)
        self._step_odom_traveled = 0.0
        self._step_commanded_traveled = 0.0
        self._step_trigger_traveled = 0.0
        self._step_remaining = self._step_target_travel
        self._step_approach_start_time = 0.0
        self._step_last_approach_time = 0.0

        self._step_waiting_alignment = False
        self._step_align_frames = 0
        self._step_align_last_reason = "locked_after_alignment"

        self._set_step_state("LOCKED", "车身已连续摆正，锁定此刻最新台阶距离")
        self.get_logger().warn(
            "航向确认后锁定台阶: distance=%.3fm height=%.3fm conf=%.2f "
            "stop=%.3fm target_travel=%.3fm"
            % (
                self._step_detected_distance,
                self._step_locked_height,
                self._step_locked_confidence,
                STEP_STOP_DISTANCE_M,
                self._step_target_travel,
            )
        )
        return True

    def _step_forward_travel(self) -> float:
        current = self._odom_snapshot()
        if current is None or self._step_lock_pose is None:
            return 0.0
        x, y, _yaw = current
        x0, y0, yaw0 = self._step_lock_pose
        return (x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)

    def _publish_step_aux(self, jump: bool) -> None:
        # Exactly the same low-jump profile/posture used by the tested standalone node.
        profile = Float32MultiArray()
        if STEP_USE_LOW_JUMP_PROFILE:
            profile.data = [STEP_JUMP_MIN_HEIGHT_M, STEP_JUMP_MAX_HEIGHT_M]
        else:
            profile.data = [0.0, 0.0]
        self.cmd_jump_profile_pub.publish(profile)

        posture = JointState()
        posture.header.stamp = self.get_clock().now().to_msg()
        posture.name = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
        posture.position = [STEP_STAND_HEIGHT_M, 0.0, 0.0, 0.0]
        self.cmd_posture_pub.publish(posture)

        jump_msg = Bool()
        jump_msg.data = bool(jump)
        self.cmd_jump_pub.publish(jump_msg)

    def _publish_jump_false_only(self) -> None:
        try:
            msg = Bool()
            msg.data = False
            self.cmd_jump_pub.publish(msg)
        except Exception:
            pass

    def _manual_jump_confirm_worker(self) -> None:
        try:
            input()
            self._step_jump_confirm_event.set()
        except EOFError:
            pass
        except Exception as error:
            print(f"[JUMP_CONFIRM] 输入线程异常: {error}", flush=True)

    def _request_manual_jump_confirm(self, now: float, reason: str) -> bool:
        if not STEP_MANUAL_CONFIRM_BEFORE_JUMP:
            return False

        if not self._step_jump_confirm_prompted:
            self._step_jump_confirm_prompted = True
            self._step_jump_confirm_event.clear()
            self._step_jump_confirm_request_time = now

            det = self._latest_step_detection
            live_d = float(det.distance) if det is not None else float("nan")
            live_h = float(det.height) if det is not None else float("nan")
            live_c = float(det.confidence) if det is not None else 0.0
            filtered_d = self._filtered_step_distance(now)

            banner = (
                "\n"
                "============================================================\n"
                "已到起跳点，车已停住。\n"
                f"第 {self.lap_count + 1}/{TOTAL_LAPS} 圈，当前阶段：{phase_text(self.phase)}\n"
                f"深度看到台阶约 {live_d:.2f}m，滤波后 {filtered_d:.2f}m，"
                f"高度 {live_h:.2f}m，可信度 {live_c:.2f}\n"
                f"本次冲刺目标 {self._step_target_travel:.2f}m，"
                f"程序认为已走 {self._step_commanded_traveled:.2f}m，"
                f"还差 {self._step_remaining:.2f}m\n"
                f"跳跃高度 {STEP_JUMP_MIN_HEIGHT_M:.2f}->{STEP_JUMP_MAX_HEIGHT_M:.2f}m，"
                f"起跳提前量 {STEP_JUMP_TRIGGER_LEAD_M:.3f}m\n"
                "------------------------------------------------------------\n"
                "位置合适：按回车跳。\n"
                "车离台阶太近/撞台阶：把 JUMP_TRIGGER_LEAD 加大一点。\n"
                "车离台阶太远/够不到：把 JUMP_TRIGGER_LEAD 减小一点。\n"
                "不想跳：按 Ctrl+C 停程序，改 StepParams 后重新跑。\n"
                "============================================================\n"
            )
            print(banner, flush=True)
            self.get_logger().warn(
                "已到起跳点并停车，等待你在终端按回车；"
                f"目标{self._step_target_travel:.2f}m，"
                f"已走{self._step_commanded_traveled:.2f}m，"
                f"剩余{self._step_remaining:.2f}m"
            )

            self._step_jump_confirm_thread = threading.Thread(
                target=self._manual_jump_confirm_worker,
                name="jump_confirm_input",
                daemon=True,
            )
            self._step_jump_confirm_thread.start()

        self._set_step_state("JUMP_CONFIRM", "到达起跳点，等待终端回车确认")
        self._publish_step_aux(False)
        return True

    def _step_begin_jump(self, now: float) -> None:
        """按已验证的 step_approach_test 逻辑：到点后直接进入带速 JUMP。"""
        self._step_jump_confirm_prompted = False
        self._step_jump_confirm_event.clear()
        self._set_step_state("JUMP", "到达起跳点，直接开始带速跳跃脉冲")
        self._step_deadline = now + STEP_JUMP_PULSE_SECONDS
        self._step_stop_reason = "JUMP 脉冲中"
        self.get_logger().warn(
            "到达起跳点，直接触发 /cmd_jump：v=%.2fm/s,w=0,pulse=%.2fs "
            "profile=%.2f->%.2fm"
            % (
                STEP_JUMP_FORWARD_SPEED_M_S,
                STEP_JUMP_PULSE_SECONDS,
                STEP_JUMP_MIN_HEIGHT_M,
                STEP_JUMP_MAX_HEIGHT_M,
            )
        )

    def _reset_step_takeover_latches(self, reason: str) -> None:
        """清空“本圈台阶接管”的所有锁存状态。

        V1.39 的问题：第一圈跳完后 _step_fixed_takeover_latched 仍为 True。
        第一圈结束仅把 _step_done_this_lap=False，第二圈一开始没有台阶深度，
        于是旧锁存触发 step_detector_lost_hold，wait_align=1，车辆永久停车。
        """
        self._step_waiting_alignment = False
        self._step_align_frames = 0
        self._step_align_last_reason = reason

        self._step_fixed_takeover_latched = False
        self._step_fixed_takeover_logged = False
        self._step_fixed_candidate_frames = 0
        self._step_race_candidate_evidence = 0.0
        self._step_race_candidate_plausible = False
        self._step_fixed_last_valid_distance = float("nan")
        self._step_fixed_lost_since = 0.0
        self._step_fixed_takeover_yaw = None
        self._step_fixed_reverse_active = False
        self._step_fixed_reverse_start_pose = None
        self._step_fixed_reverse_traveled = 0.0
        self._step_fixed_reverse_target_yaw = None
        self._step_fixed_reverse_start_time = 0.0
        self._step_fixed_reverse_reacquire_frames = 0
        self._step_fixed_reverse_target_distance = 0.0
        self._step_fixed_start_confirm_frames = 0
        self._step_fixed_too_close_frames = 0
        self._step_depth_history.clear()

        self._step_calib_takeover_latched = False
        self._step_calib_takeover_logged = False
        self._step_calib_alignment_latched = False
        self._step_calib_at_target_depth = False
        self._step_calib_align_frames = 0
        self._step_calib_final_frames = 0

        self._step_target_yaw = None
        self._step_yaw_source = "none"
        self._step_yaw_error = 0.0
        self._step_jump_confirm_prompted = False
        self._step_jump_confirm_event.clear()
        self._step_jump_confirm_request_time = 0.0

    def _post_step_reacquire_distance(self) -> float:
        current = self._odom_snapshot()
        start = self._post_step_reacquire_start_pose
        if current is None or start is None:
            return 0.0
        x, y, _yaw = current
        x0, y0, _yaw0 = start
        distance = math.hypot(x - x0, y - y0)
        if distance > 2.0:
            return float("inf")
        return distance

    def _start_post_step_reacquire(self, now: float) -> None:
        if not STEP_POST_REACQUIRE_ENABLED:
            self._post_step_reacquire_active = False
            return
        self._post_step_reacquire_active = True
        self._post_step_reacquire_until = now + STEP_POST_REACQUIRE_SECONDS
        self._post_step_reacquire_start_pose = self._odom_snapshot()
        self._last_valid_line_time = now
        self._last_good_angular = STEP_POST_REACQUIRE_TURN_RAD_S
        direction = "右" if STEP_POST_REACQUIRE_TURN_RAD_S < 0.0 else "左"
        self.get_logger().warn(
            "跳完开始找线：当前黑线可能还没进视野，"
            f"低速向{direction}找线，最多{STEP_POST_REACQUIRE_SECONDS:.1f}s/"
            f"{STEP_POST_REACQUIRE_MAX_DISTANCE_M:.2f}m"
        )

    def _stop_post_step_reacquire(self, reason: str) -> None:
        if not self._post_step_reacquire_active:
            return
        distance = self._post_step_reacquire_distance()
        self._post_step_reacquire_active = False
        self._post_step_reacquire_until = 0.0
        self._post_step_reacquire_start_pose = None
        self.get_logger().warn(f"跳后找线结束：{reason}，已走{distance:.2f}m")

    def _finish_step_recovery(self, now: float) -> None:
        self._step_done_this_lap = True
        self._step_state = "IDLE"

        # V1.40：本圈台阶已经完成，旧的 takeover 锁存必须立刻清掉。
        # _step_done_this_lap=True 仍然保证本圈后续不会再次触发台阶。
        self._reset_step_takeover_latches("step_done_latches_cleared")
        self._step_stop_reason = "本圈台阶完成"
        self._step_last_mode = "step_done_resume_tracking"
        self._last_valid_line_time = now
        self._last_good_angular = 0.0
        self._have_previous_error = False
        # 本圈台阶结束：普通赛道目标继续保持0.00中心。
        # 下一次视觉帧会固定 target offset=0.0。
        self.vision._effective_target_offset_ratio = 0.0
        self.vision.reset_tracking()

        # Avoid counting the jump displacement as one artificial SEEK segment.
        if self.phase == "SEEK_ROUNDABOUT":
            self._seek_last_odom = self._odom_snapshot()

        self._start_post_step_reacquire(now)
        self.get_logger().warn("台阶跳跃完成并恢复完毕：本圈镜头偏移已关闭，后续固定为0；重新交还给视觉循迹")

    def _step_command(self, now: float) -> Tuple[float, float, str]:
        """锁定后复用 step_approach_test 的已验证接近/跳跃动作。

        与独立测试保持一致：
        - V1.50固定冲刺模式：名义target_travel=0.605m，实际提前0.055m触发；
        - trigger_distance_source=commanded；
        - APPROACH 恒定 v=0.50m/s、w=0；
        - 到点直接 v=0.50m/s + /cmd_jump=True；
        - 跳跃脉冲 0.20s；
        - LAND 以 0.30m/s 前行 2s。

        比赛程序只在 LAND 后额外保留原来的 RECOVER，再恢复视觉巡线。
        """
        state = self._step_state

        if state == "LOCKED":
            if self._step_target_travel <= (
                STEP_POSITION_TOLERANCE_M + STEP_JUMP_TRIGGER_LEAD_M
            ):
                if self._request_manual_jump_confirm(now, "locked_target_too_short"):
                    return 0.0, 0.0, "step_wait_enter_to_jump"
                self._step_begin_jump(now)
                self._publish_step_aux(True)
                return STEP_JUMP_FORWARD_SPEED_M_S, 0.0, "step_jump_early_pulse"

            if now - self._step_lock_time < STEP_LOCK_HOLD_SECONDS:
                self._publish_step_aux(False)
                return 0.0, 0.0, "step_locked_hold"

            self._step_approach_start_time = now
            self._step_last_approach_time = now
            self._set_step_state("APPROACH", "开始按已验证参数直行到起跳距离")
            self._publish_step_aux(False)
            return 0.0, 0.0, "step_approach_begin"

        if state == "APPROACH":
            # 与 step_approach_test.py 一致：APPROACH 期间要求 /odom 新鲜，
            # 但实际起跳触发距离使用 commanded=速度×时间。
            if not self._step_odom_fresh(now):
                self._publish_step_aux(False)
                self._step_stop_reason = "APPROACH 等待新鲜 /odom"
                return 0.0, 0.0, "step_wait_odom"

            self._step_odom_traveled = self._step_forward_travel()
            dt = max(0.0, min(now - self._step_last_approach_time, 0.20))
            self._step_last_approach_time = now
            self._step_commanded_traveled += STEP_APPROACH_SPEED_M_S * dt

            if STEP_TRIGGER_DISTANCE_SOURCE == "commanded":
                traveled = self._step_commanded_traveled
            else:
                traveled = self._step_odom_traveled

            self._step_trigger_traveled = traveled
            self._step_remaining = self._step_target_travel - traveled

            jump_trigger_margin = (
                STEP_POSITION_TOLERANCE_M
                + STEP_JUMP_TRIGGER_LEAD_M
            )
            if self._step_remaining <= jump_trigger_margin:
                # V1.50：提前发jump，而不是等前轮几乎撞到台阶才触发。
                # 名义target仍是0.605m，仅把触发点提前STEP_JUMP_TRIGGER_LEAD_M。
                self._step_stop_reason = (
                    "EARLY_JUMP_TRIGGER remaining=%.3f <= %.3f "
                    "(tol=%.3f lead=%.3f)"
                    % (
                        self._step_remaining,
                        jump_trigger_margin,
                        STEP_POSITION_TOLERANCE_M,
                        STEP_JUMP_TRIGGER_LEAD_M,
                    )
                )
                if self._request_manual_jump_confirm(now, self._step_stop_reason):
                    return 0.0, 0.0, "step_wait_enter_to_jump"
                self._step_begin_jump(now)
                self._publish_step_aux(True)
                return STEP_JUMP_FORWARD_SPEED_M_S, 0.0, "step_jump_early_pulse"

            if (
                STEP_MAX_APPROACH_SECONDS > 0.0
                and now - self._step_approach_start_time > STEP_MAX_APPROACH_SECONDS
            ):
                self._set_step_state("STOPPED", "APPROACH 超时；安全停车")
                self._publish_step_aux(False)
                return 0.0, 0.0, "step_approach_timeout_stop"

            self._step_stop_reason = (
                "APPROACH 恒速前进 source=%s cmd=%.3f odom=%.3f remaining=%.3f"
                % (
                    STEP_TRIGGER_DISTANCE_SOURCE,
                    self._step_commanded_traveled,
                    self._step_odom_traveled,
                    self._step_remaining,
                )
            )
            self._step_yaw_error = 0.0
            self._publish_step_aux(False)
            return STEP_APPROACH_SPEED_M_S, 0.0, "step_approach_tested"

        if state == "JUMP_CONFIRM":
            self._publish_step_aux(False)
            self._step_odom_traveled = self._step_forward_travel()
            self._step_stop_reason = "JUMP_CONFIRM 等待终端回车"
            if self._step_jump_confirm_event.is_set():
                self.get_logger().warn("收到回车，开始跳跃")
                self._step_begin_jump(now)
                self._publish_step_aux(True)
                return STEP_JUMP_FORWARD_SPEED_M_S, 0.0, "step_jump_after_enter"
            return 0.0, 0.0, "step_wait_enter_to_jump"

        if state == "JUMP":
            if now >= self._step_deadline:
                self._set_step_state("LAND", "跳跃脉冲结束，落地继续前行")
                self._step_deadline = now + STEP_LANDING_FORWARD_SECONDS
                self._publish_step_aux(False)
                return (
                    STEP_JUMP_FORWARD_SPEED_M_S * STEP_LANDING_SPEED_SCALE,
                    0.0,
                    "step_land_forward",
                )
            self._publish_step_aux(True)
            return STEP_JUMP_FORWARD_SPEED_M_S, 0.0, "step_jump_pulse"

        if state == "LAND":
            if now >= self._step_deadline:
                self._set_step_state("RECOVER", "落地前行结束，停车恢复")
                self._step_deadline = now + STEP_RECOVER_SECONDS
                self._publish_step_aux(False)
                return 0.0, 0.0, "step_recover_stop"
            self._publish_step_aux(False)
            return (
                STEP_JUMP_FORWARD_SPEED_M_S * STEP_LANDING_SPEED_SCALE,
                0.0,
                "step_land_forward",
            )

        if state == "RECOVER":
            self._publish_step_aux(False)
            if now >= self._step_deadline:
                self._finish_step_recovery(now)
                return 0.0, 0.0, "step_resume_tracking"
            return 0.0, 0.0, "step_recover_stop"

        if state == "CALIB_HOLD":
            # 标定模式到达冲刺起点后永久停车，直到用户重启程序做下一次测量。
            self._publish_step_aux(False)
            return 0.0, 0.0, "step_sprint_calibration_hold"

        if state == "STOPPED":
            self._publish_step_aux(False)
            return 0.0, 0.0, "step_failed_stop"

        return 0.0, 0.0, "step_idle"

    def _record_active_step_frame(
        self, linear: float, angular: float, mode: str, now: float
    ) -> None:
        """比赛录像里保留真正的 APPROACH/JUMP/LAND 过程，方便下次精确定位。

        旧版一进入 STEP active 就在读取 RGB 之前 return，所以录像会从
        STEP=APPROACH 直接跳到 done=1，看不到实际起跳。这里直接取采集线程
        的最新帧，不再跑视觉算法，因此不会明显增加跳跃控制负担。
        """
        with self._frame_lock:
            frame = (
                self._latest_frame.copy()
                if self._latest_frame is not None
                else None
            )
        if frame is None:
            return

        self._record_raw_frame(frame)
        debug = frame.copy()
        remaining = self._step_remaining
        lines = [
            (f"台阶状态：{step_text(self._step_state)}", (0, 255, 255)),
            (
                f"速度 {linear:.2f}m/s  已走 {self._step_commanded_traveled:.2f}m  剩余 {remaining:.2f}m",
                (255, 255, 255),
            ),
            (
                f"跳高 {STEP_JUMP_MIN_HEIGHT_M:.2f}->{STEP_JUMP_MAX_HEIGHT_M:.2f}m  提前量 {STEP_JUMP_TRIGGER_LEAD_M:.3f}m",
                (255, 220, 120),
            ),
        ]
        if self._step_state == "JUMP_CONFIRM":
            lines.append(("已到起跳点：按回车跳，Ctrl+C取消", (0, 180, 255)))
        debug = draw_overlay(debug, lines, font_size=20, line_height=28)
        self._record_frame(debug)

        if self._debug_window_ready:
            try:
                cv2.imshow("WheelLeg Race Vision", debug)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    self.request_exit()
            except cv2.error:
                self._debug_window_ready = False

    def _run_active_step_cycle(self, now: float) -> Tuple[float, float, str]:
        """Run step task even if the RGB camera momentarily drops during the jump."""
        linear, angular, mode = self._step_command(now)

        if ENABLE_MOTION:
            # Critical: do NOT use the race acceleration limiter here.
            # The tested controller assumes exact 0.50 m/s when integrating commanded distance.
            self._publish_command(linear, angular)
            self._last_command = (linear, angular)
        else:
            self._publish_command(0.0, 0.0)
            self._publish_jump_false_only()
            self._last_command = (0.0, 0.0)
            mode = "DRY_RUN/" + mode

        self._last_motion_update_time = now
        self._step_last_mode = mode
        self._record_active_step_frame(linear, angular, mode, now)
        self._write_race_log(None, now, mode, linear, angular)
        return linear, angular, mode

    # ------------------------------ state ----------------------------------
    def _roundabout_approach_target(self) -> float:
        if self.lap_count >= 1:
            base_target = ROUNDABOUT_SECOND_LAP_APPROACH_DISTANCE
        else:
            base_target = ROUNDABOUT_FIRST_LAP_APPROACH_DISTANCE
        return max(0.20, base_target - self._roundabout_approach_compensation)

    def _set_phase(self, phase: str, now: float) -> None:
        old = self.phase
        self.phase = phase
        self.phase_enter_time = now
        self._roundabout_gate_frames = 0
        self._roundabout_exit_frames = 0
        self._roundabout_gate_evidence = 0.0
        self._roundabout_return_evidence = 0.0
        self._crosswalk_frames = 0
        self._yellow_frames = 0
        if phase == "SEEK_ROUNDABOUT":
            self._reset_seek_roundabout_progress()
            self._roundabout_gate_first_seen_seek = None
            self._roundabout_approach_compensation = 0.0
        elif phase == "ROUNDABOUT_APPROACH":
            self._reset_roundabout_approach_progress()
        elif phase == "CROSSWALK_APPROACH":
            self._reset_crosswalk_approach_progress()
        elif phase == "FINISH_PASS":
            self._reset_finish_pass_progress()

        # V1.7：SEEK->APPROACH->ROUNDABOUT 都是同一条物理轨迹的连续过程，
        # 不应在状态切换瞬间清掉上一帧轨迹，否则绿色路径容易突然跳到另一条虚线。
        preserve_tracking = (
            (old == "SEEK_ROUNDABOUT" and phase == "ROUNDABOUT_APPROACH")
            or (old == "ROUNDABOUT_APPROACH" and phase == "ROUNDABOUT")
            # ICN：ENTRY_LEFT 结束后继续 FOLLOW，因此不清空视觉历史。
            or (old == "ROUNDABOUT" and phase == "ROUNDABOUT_FOLLOW")
            or (old == "ROUNDABOUT_FOLLOW" and phase == "ROUNDABOUT_EXIT")
            or (old == "ROUNDABOUT_EXIT" and phase == "SEEK_CROSSWALK")
        )
        if not preserve_tracking:
            self.vision.reset_tracking()
        if phase == "SEEK_ROUNDABOUT":
            self._roundabout_gate_source = "none"
        if old == "ROUNDABOUT" and phase == "ROUNDABOUT_FOLLOW":
            self._roundabout_exit_guard_until = now + ROUNDABOUT_EXIT_GUARD_SECONDS
        self._have_previous_error = False
        self.get_logger().info(f"状态切换: {old} -> {phase}")

    def _handle_yellow_finish(self, now: float, source: str) -> None:
        if self.lap_count + 1 < TOTAL_LAPS:
            self.lap_count += 1

            self._reset_step_takeover_latches("new_lap_step_reset")
            self._step_state = "IDLE"
            self._step_done_this_lap = False

            self._latest_step_detection = None
            self._latest_step_detection_time = 0.0

            self._set_phase("SEEK_ROUNDABOUT", now)
            self.get_logger().warn(
                f"黄色终点线确认：第 {self.lap_count} 圈完成，开始下一圈；"
                f"来源={source}；第二圈重新启用台阶检测"
            )
        else:
            self._set_phase("FINISH_PASS", now)
            self.get_logger().warn(
                f"第二圈黄色终点线确认：来源={source}；继续循迹冲过黄线 "
                f"{FINISH_PASS_DISTANCE:.2f}m 后再结束"
            )

    def _sync_course_state(self, result: VisionResult, now: float) -> None:
        phase_age = now - self.phase_enter_time

        if self.phase == "SEEK_ROUNDABOUT":
            self._update_seek_roundabout_progress()
            odom_ready = self._odom_snapshot() is not None
            travel_ready = self._seek_distance >= ROUNDABOUT_GATE_MIN_TRAVEL
            fallback_ready = (
                (not odom_ready)
                and phase_age >= ROUNDABOUT_GATE_NO_ODOM_FALLBACK_SECONDS
            )
            gate_armed = (
                phase_age >= ROUNDABOUT_GATE_DETECT_DELAY
                and (travel_ready or fallback_ready)
            )
            if not gate_armed:
                self._roundabout_gate_frames = 0
                self._roundabout_gate_evidence = 0.0
                self._roundabout_gate_first_seen_seek = None
                self._roundabout_approach_compensation = 0.0
                return

            if (
                ROUNDABOUT_SECOND_LAP_USE_FIRST_POSITION
                and self.lap_count >= 1
                and self._step_done_this_lap
                and self._first_roundabout_gate_seek is not None
            ):
                trigger_seek = max(
                    ROUNDABOUT_GATE_MIN_TRAVEL,
                    self._first_roundabout_gate_seek
                    - ROUNDABOUT_SECOND_LAP_ADVANCE_M,
                )
                if self._seek_distance >= trigger_seek:
                    approach_target = self._roundabout_approach_target()
                    self._roundabout_gate_source = "first_lap_position"
                    self._set_phase("ROUNDABOUT_APPROACH", now)
                    self.get_logger().warn(
                        "第二圈按第一圈环岛位置提前接管："
                        f"first={self._first_roundabout_gate_seek:.2f}m，"
                        f"advance={ROUNDABOUT_SECOND_LAP_ADVANCE_M:.2f}m，"
                        f"now={self._seek_distance:.2f}m；"
                        f"继续巡线接近 {approach_target:.2f}m 后进环"
                    )
                    return

            # V1.7：用“证据积分”代替严格连续帧。
            # 录像中 gate 在 15.6~15.8s 连续出现，但控制线程会跳过部分帧，
            # 所以原来的连续4帧条件可能永远达不到。
            if (
                result.roundabout_gate
                and result.roundabout_gate_score >= ROUNDABOUT_GATE_STRONG_SCORE
            ):
                if self._roundabout_gate_first_seen_seek is None:
                    self._roundabout_gate_first_seen_seek = self._seek_distance
                self._roundabout_gate_evidence = min(
                    ROUNDABOUT_GATE_EVIDENCE_MAX,
                    self._roundabout_gate_evidence
                    + ROUNDABOUT_GATE_EVIDENCE_STRONG_GAIN,
                )
            elif result.roundabout_gate_score >= ROUNDABOUT_GATE_WEAK_SCORE:
                self._roundabout_gate_evidence = min(
                    ROUNDABOUT_GATE_EVIDENCE_MAX,
                    self._roundabout_gate_evidence
                    + ROUNDABOUT_GATE_EVIDENCE_WEAK_GAIN,
                )
            else:
                self._roundabout_gate_evidence = max(
                    0.0,
                    self._roundabout_gate_evidence
                    - ROUNDABOUT_GATE_EVIDENCE_DECAY,
                )

            if self._roundabout_gate_evidence >= ROUNDABOUT_GATE_EVIDENCE_TRIGGER:
                # 这里只说明“远处已经看见环岛”，不能马上强制左转。
                # 先继续沿当前黑线走一小段，让车真正到达入口几何位置。
                evidence = self._roundabout_gate_evidence
                first_seen_seek = (
                    self._roundabout_gate_first_seen_seek
                    if self._roundabout_gate_first_seen_seek is not None
                    else self._seek_distance
                )
                confirm_delay_m = max(0.0, self._seek_distance - first_seen_seek)
                self._roundabout_approach_compensation = clamp(
                    confirm_delay_m - ROUNDABOUT_GATE_CONFIRM_DELAY_ALLOWANCE_M,
                    0.0,
                    ROUNDABOUT_GATE_CONFIRM_DELAY_MAX_COMP_M,
                )
                if self.lap_count == 0 and self._first_roundabout_gate_seek is None:
                    self._first_roundabout_gate_seek = self._seek_distance
                self._roundabout_gate_source = "vision"
                approach_target = self._roundabout_approach_target()
                self._set_phase("ROUNDABOUT_APPROACH", now)
                self.get_logger().info(
                    f"视觉确认远处环岛虚线：gate={result.roundabout_gate_score:.2f}，"
                    f"evidence={evidence:.2f}，本圈已行驶 {self._seek_distance:.2f}m；"
                    f"首次强证据 {first_seen_seek:.2f}m，确认延迟 {confirm_delay_m:.2f}m，"
                    f"补偿 {self._roundabout_approach_compensation:.2f}m；"
                    f"继续巡线接近 {approach_target:.2f}m 后再进环"
                )
                return

        elif self.phase == "ROUNDABOUT_APPROACH":
            self._update_roundabout_approach_progress()
            approach_target = self._roundabout_approach_target()
            odom_ready = self._odom_snapshot() is not None
            distance_ready = (
                phase_age >= ROUNDABOUT_APPROACH_MIN_SECONDS
                and self._approach_distance >= approach_target
            )
            fallback_ready = (
                (not odom_ready)
                and phase_age >= ROUNDABOUT_APPROACH_FALLBACK_SECONDS
            )
            if distance_ready or fallback_ready:
                self._set_phase("ROUNDABOUT", now)
                self._reset_roundabout_progress()
                self.get_logger().info(
                    f"到达环岛实际进环点：approach={self._approach_distance:.2f}m，"
                    "现在才启用左环控制，环岛短程 odom 清零"
                )
                return

        elif self.phase == "ROUNDABOUT":
            # V1.8：完全回到 icn.py 已经跑通两圈的环岛运动逻辑。
            # 不找“出口框”、不累计 ring 路程、不用 yaw；只比较当前位置与进环起点。
            self._update_roundabout_progress()
            elapsed = now - self.phase_enter_time
            odom_ready = self._odom_snapshot() is not None and self._rb_entry_pose is not None
            displacement_ready = (
                odom_ready
                and self._rb_entry_displacement >= ROUNDABOUT_ENTRY_TURN_DISTANCE
            )
            fallback_ready = elapsed >= ROUNDABOUT_ENTRY_FALLBACK_SECONDS
            if displacement_ready or fallback_ready:
                displacement = self._rb_entry_displacement
                finish_reason = (
                    "距离到达"
                    if displacement_ready
                    else "时间兜底"
                )
                self._set_phase("ROUNDABOUT_FOLLOW", now)
                self.get_logger().info(
                    f"ICN 环岛强制左转结束：entry_disp={displacement:.2f}m/"
                    f"{ROUNDABOUT_ENTRY_TURN_DISTANCE:.2f}m，elapsed={elapsed:.1f}s，"
                    f"reason={finish_reason}；进入 ROUNDABOUT_FOLLOW，"
                    f"接下来 {ROUNDABOUT_FOLLOW_SECONDS:.1f}s 保持环岛FOLLOW：有线循迹但禁止大幅右转，丢线沿低速左弧线找线"
                )
                return

        elif self.phase == "ROUNDABOUT_FOLLOW":
            # V1.44：FOLLOW_SECONDS 不再承担“什么时候解除左环保护”的唯一职责。
            # 用户实测已经证明：
            # - 太短：出口还没真正过，普通巡线立即抓右支路，转回环岛；
            # - 太长：明明到了出口，仍保持左环逻辑，错过出口。
            #
            # 现在改为：
            # 1) 至少在环岛内保持 ROUNDABOUT_MIN_SECONDS；
            # 2) 之后再次看到高置信度虚线交汇，积累返回证据；
            # 3) 证据够了进入 ROUNDABOUT_EXIT；
            # 4) EXIT 再保持一小段“禁止大右转”，越过分叉后才恢复普通巡线。
            if phase_age >= ROUNDABOUT_MIN_SECONDS:
                if (
                    result.roundabout_gate
                    and result.roundabout_gate_score >= ROUNDABOUT_RETURN_GATE_MIN_SCORE
                ):
                    self._roundabout_return_evidence = min(
                        6.0,
                        self._roundabout_return_evidence
                        + ROUNDABOUT_RETURN_EVIDENCE_GAIN,
                    )
                else:
                    self._roundabout_return_evidence = max(
                        0.0,
                        self._roundabout_return_evidence
                        - ROUNDABOUT_RETURN_EVIDENCE_DECAY,
                    )

                if (
                    self._roundabout_return_evidence
                    >= ROUNDABOUT_RETURN_EVIDENCE_TRIGGER
                ):
                    evidence = self._roundabout_return_evidence
                    score = result.roundabout_gate_score
                    self._set_phase("ROUNDABOUT_EXIT", now)
                    self.get_logger().warn(
                        f"视觉确认环岛出口交汇：follow={phase_age:.1f}s "
                        f"gate={score:.2f} return_evidence={evidence:.2f}；"
                        f"进入出口提交段 {ROUNDABOUT_EXIT_COMMIT_SECONDS:.2f}s，"
                        "暂时禁止大幅右转"
                    )
                    return

            # 28s 只做最终兜底，不直接进入普通巡线，同样先经过 EXIT 提交段。
            if phase_age >= ROUNDABOUT_FOLLOW_SECONDS:
                self._set_phase("ROUNDABOUT_EXIT", now)
                self.get_logger().warn(
                    f"ROUNDABOUT_FOLLOW 达到最大兜底 {phase_age:.1f}s；"
                    f"进入出口提交段 {ROUNDABOUT_EXIT_COMMIT_SECONDS:.2f}s，"
                    "而不是立即解除右转保护"
                )
                return

        elif self.phase == "ROUNDABOUT_EXIT":
            if phase_age >= ROUNDABOUT_EXIT_COMMIT_SECONDS:
                self._set_phase("SEEK_CROSSWALK", now)
                self.get_logger().info(
                    f"环岛出口提交完成 {phase_age:.2f}s："
                    "现在才恢复普通赛道循迹并寻找人行横道"
                )
                return

        elif self.phase == "SEEK_CROSSWALK":
            if phase_age < CROSSWALK_DETECT_DELAY_AFTER_ROUNDABOUT:
                self._crosswalk_frames = 0
                self._yellow_frames = 0
                return
            if result.crosswalk:
                self._crosswalk_frames += 1
            else:
                self._crosswalk_frames = 0
            if self._crosswalk_frames >= CROSSWALK_CONFIRM_FRAMES:
                self._set_phase("CROSSWALK_APPROACH", now)
                self.get_logger().info(
                    f"视觉确认人行横道：score={result.crosswalk_score:.2f}；"
                    f"先继续循迹靠近 {CROSSWALK_APPROACH_DISTANCE:.2f}m，再停车"
                )
                return
            if YELLOW_ACCEPT_WHILE_SEEK_CROSSWALK:
                if result.yellow_finish:
                    self._yellow_frames += 1
                else:
                    self._yellow_frames = 0
                if self._yellow_frames >= YELLOW_FINISH_CONFIRM_FRAMES:
                    self.get_logger().warn(
                        "找人行道阶段已看到黄色终点线："
                        "判定人行道漏检，直接按黄线进入下一圈/终点"
                    )
                    self._handle_yellow_finish(now, "crosswalk_missed_yellow")
                    return

        elif self.phase == "CROSSWALK_APPROACH":
            self._update_crosswalk_approach_progress()
            odom_ready = (
                self._odom_snapshot() is not None
                and self._crosswalk_approach_start_pose is not None
            )
            distance_ready = (
                phase_age >= CROSSWALK_APPROACH_MIN_SECONDS
                and self._crosswalk_approach_distance >= CROSSWALK_APPROACH_DISTANCE
            )
            fallback_ready = (
                (not odom_ready)
                and phase_age >= CROSSWALK_APPROACH_FALLBACK_SECONDS
            )
            if distance_ready or fallback_ready:
                self._stop_until = now + CROSSWALK_STOP_SECONDS
                traveled = self._crosswalk_approach_distance
                self._set_phase("CROSSWALK_STOP", now)
                self.get_logger().info(
                    f"到达人行横道停车点：approach={traveled:.2f}m，"
                    f"停车 {CROSSWALK_STOP_SECONDS:.1f}s"
                )
                return

        elif self.phase == "CROSSWALK_STOP":
            if now >= self._stop_until:
                self._set_phase("SEEK_FINISH", now)
                return

        elif self.phase == "SEEK_FINISH":
            if phase_age < FINISH_DETECT_DELAY_AFTER_CROSSWALK:
                self._yellow_frames = 0
                return
            if result.yellow_finish:
                self._yellow_frames += 1
            else:
                self._yellow_frames = 0
            if self._yellow_frames >= YELLOW_FINISH_CONFIRM_FRAMES:
                self._handle_yellow_finish(now, "seek_finish")
                return

        elif self.phase == "FINISH_PASS":
            self._update_finish_pass_progress()
            odom_ready = (
                self._odom_snapshot() is not None
                and self._finish_pass_start_pose is not None
            )
            distance_ready = (
                phase_age >= FINISH_PASS_MIN_SECONDS
                and self._finish_pass_distance >= FINISH_PASS_DISTANCE
            )
            fallback_ready = (
                (not odom_ready)
                and phase_age >= FINISH_PASS_FALLBACK_SECONDS
            )
            if distance_ready or fallback_ready:
                traveled = self._finish_pass_distance
                self.lap_count = TOTAL_LAPS
                self._set_phase("FINISHED", now)
                self.get_logger().warn(
                    f"已冲过第二圈黄色终点线：pass={traveled:.2f}m，两圈完成，停车"
                )
                return

    # ------------------------------ control --------------------------------
    def _normal_angular(self, result: VisionResult, now: float) -> float:
        if self._have_previous_error:
            filtered = (
                ERROR_FILTER_ALPHA * result.error
                + (1.0 - ERROR_FILTER_ALPHA) * self._previous_error
            )
            dt = max(1e-3, now - self._previous_control_time)
            derivative = clamp(
                (filtered - self._previous_error) / dt,
                -MAX_ERROR_DERIVATIVE,
                MAX_ERROR_DERIVATIVE,
            )
        else:
            filtered = result.error
            derivative = 0.0
            self._have_previous_error = True
        self._previous_error = float(filtered)
        self._previous_control_time = now
        angular = STEERING_SIGN * (
            STEERING_KP * filtered + STEERING_KD * derivative
        )
        angular_limit = MAX_ANGULAR_SPEED
        if result.reason == "line_edge_single_rescue":
            angular_limit = min(angular_limit, LINE_EDGE_RESCUE_MAX_ANGULAR)
        elif result.reason == "line_hard_turn_centering":
            angular_limit = min(angular_limit, LINE_HARD_TURN_MAX_ANGULAR)
        return clamp(angular, -angular_limit, angular_limit)

    @staticmethod
    def _approach(current: float, target: float, max_delta: float) -> float:
        return clamp(target, current - max_delta, current + max_delta)

    def _limit_command(self, linear: float, angular: float, now: float) -> Tuple[float, float]:
        dt = clamp(now - self._last_motion_update_time, 0.0, 2.0 / CONTROL_RATE)
        linear = self._approach(
            self._last_command[0], linear, MAX_LINEAR_ACCELERATION * dt
        )
        angular = self._approach(
            self._last_command[1], angular, MAX_ANGULAR_ACCELERATION * dt
        )
        self._last_motion_update_time = now
        return linear, angular

    def _planned_command(self, result: VisionResult, now: float) -> Tuple[float, float, str]:
        if self.phase in ("FINISHED", "CROSSWALK_STOP"):
            return 0.0, 0.0, self.phase.lower()

        # ROUNDABOUT_APPROACH 仍按普通视觉巡线。真正进入 ROUNDABOUT 后，
        # 完全复用 icn.py 已验证的固定运动，不让虚线视觉左右拉扯。
        if self.phase == "ROUNDABOUT":
            # icn.py 的固定进环期间，视觉检测仍持续运行；如果还能看到线，
            # last_valid 时间会保持新鲜。这里同步这一行为，避免 1.00m 固定左转结束后
            # 第一帧恰好丢线就因为 last_valid 太旧而立即停车。
            if result.line_valid:
                self._last_valid_line_time = now
            self._last_good_angular = ROUNDABOUT_ENTRY_LEFT_ANGULAR
            return (
                ROUNDABOUT_FORWARD_SPEED,
                ROUNDABOUT_ENTRY_LEFT_ANGULAR,
                "icn_entry_left",
            )

        # V1.45：ROUNDABOUT_FOLLOW / ROUNDABOUT_EXIT 的最高优先级安全护栏。
        # 如果前方蓝色边界明显横到车头前，而白色赛道明显在某一侧，
        # 立即朝白色赛道一侧低速转回。此时不相信黑线，也不执行普通盲走。
        if (
            self.phase in ("ROUNDABOUT_FOLLOW", "ROUNDABOUT_EXIT")
            and self.vision._roundabout_blue_guard_active
        ):
            sign = self.vision._roundabout_blue_guard_turn_sign
            angular = sign * ROUNDABOUT_BLUE_GUARD_ANGULAR
            self._last_good_angular = angular
            self._have_previous_error = False
            side = "left" if sign > 0.0 else "right"
            return (
                ROUNDABOUT_BLUE_GUARD_SPEED,
                angular,
                f"roundabout_blue_guard_{side}",
            )

        # V1.44：视觉确认出口后先做短暂“出口提交”。
        # 关键不是强行向左，而是暂时不允许普通循迹突然给出大右转，
        # 让车真正越过分叉，再恢复普通赛道控制。
        if self.phase == "ROUNDABOUT_EXIT":
            if result.line_valid:
                self._last_valid_line_time = now
                angular = self._normal_angular(result, now)
                angular = clamp(
                    angular,
                    ROUNDABOUT_EXIT_MIN_ANGULAR,
                    ROUNDABOUT_EXIT_MAX_ANGULAR,
                )
                self._last_good_angular = angular
                return (
                    ROUNDABOUT_EXIT_SPEED,
                    angular,
                    "roundabout_exit_commit",
                )

            # 出口提交段暂时丢线时沿一个很缓的左弧线向前，不原地找线。
            angular = min(
                ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR,
                ROUNDABOUT_EXIT_MAX_ANGULAR,
            )
            self._last_good_angular = angular
            return (
                ROUNDABOUT_EXIT_SPEED,
                angular,
                "roundabout_exit_blind_commit",
            )

        # V1.24：台阶已确认但尚未锁定时，使用专用“航向摆正”控制。
        # 普通巡线会把 near/mid 的相反误差互相抵消；这里让 heading 项占主导，
        # 目标是把黑线从“斜着穿过画面”转成近/中场基本竖直。
        if self._step_waiting_alignment:
            # V1.43：近处丢深度时低速倒车重新找回台阶，同时用 odom yaw 保持车头。
            if (
                STEP_FIXED_SPRINT_ENABLED
                and STEP_FIXED_REVERSE_RECOVERY_ENABLED
                and self._step_fixed_reverse_active
            ):
                self._publish_step_aux(False)
                self._have_previous_error = False
                angular = self._fixed_yaw_hold_angular()
                self._last_good_angular = angular
                return (
                    -STEP_FIXED_REVERSE_SPEED_M_S,
                    angular,
                    "step_fixed_reverse_recovery",
                )

            # V1.49：正在确认“是否真的太近”时先停车，不能一边确认一边继续前冲。
            if (
                STEP_FIXED_SPRINT_ENABLED
                and self._step_fixed_too_close_frames > 0
            ):
                self._publish_step_aux(False)
                self._have_previous_error = False
                return 0.0, 0.0, "step_start_too_close_confirm_hold"

            # V1.50：已经进入0.70±1.5cm窗口并在做连续3帧确认时，
            # 必须原地保持，不能每帧还以0.045m/s继续往前蹭。
            if (
                STEP_FIXED_SPRINT_ENABLED
                and self._step_fixed_start_confirm_frames > 0
            ):
                self._publish_step_aux(False)
                self._have_previous_error = False
                return 0.0, 0.0, "step_start_window_confirm_hold"

            # V1.37：标定已接管后，如果深度当前无效/丢失，原地停车等待，
            # 绝不恢复普通巡线继续向前。
            if (
                (
                    (STEP_SPRINT_CALIBRATION_MODE and self._step_calib_takeover_latched)
                    or (STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched)
                )
                and (
                    not self._step_detection_fresh(now)
                    or self._latest_step_detection is None
                    or not bool(self._latest_step_detection.detected)
                )
            ):
                self._publish_step_aux(False)
                self._have_previous_error = False
                return 0.0, 0.0, "step_fixed_depth_lost_hold"

            # 与原来单独测试 step_approach_test.py 一致：
            # 在真正起跳前就持续刷新低跳 profile + 0.25m posture，并明确 jump=False。
            # 底盘侧 jump profile 1 秒不刷新会自动失效，所以这里提前预热并保持新鲜。
            self._publish_step_aux(False)

            if result.line_valid:
                self._last_valid_line_time = now
                heading_error = float(result.mid_error) - float(result.near_error)
                if STEP_SPRINT_CALIBRATION_MODE or STEP_FIXED_SPRINT_ENABLED:
                    # 继续复用已经实测能把车头摆正的控制：
                    # 用 N/M/F 平均位置修正横向位置，near-mid 修正车头方向。
                    center_error_now = (
                        float(result.near_error)
                        + float(result.mid_error)
                        + float(result.far_error)
                    ) / 3.0
                    align_signal = (
                        STEP_CALIB_CENTER_KP * center_error_now
                        + STEP_ALIGN_HEADING_KP * heading_error
                    )
                else:
                    align_signal = (
                        STEP_ALIGN_CENTER_KP * float(result.near_error)
                        + STEP_ALIGN_HEADING_KP * heading_error
                    )
                angular = clamp(
                    STEERING_SIGN * align_signal,
                    -STEP_ALIGN_MAX_ANGULAR,
                    STEP_ALIGN_MAX_ANGULAR,
                )
                self._last_good_angular = angular
                self._have_previous_error = False

                if STEP_FIXED_SPRINT_ENABLED and self._step_fixed_takeover_latched:
                    pose_now = self._odom_snapshot()
                    if pose_now is not None:
                        self._step_fixed_takeover_yaw = float(pose_now[2])

                live_distance = float("nan")
                if self._step_detection_fresh(now) and self._latest_step_detection is not None:
                    live_distance = float(self._latest_step_detection.distance)

                filtered_live_distance = self._filtered_step_distance(now)
                control_distance = (
                    filtered_live_distance
                    if math.isfinite(filtered_live_distance)
                    else live_distance
                )

                if (
                    math.isfinite(control_distance)
                    and control_distance
                        <= STEP_STOP_DISTANCE_M + STEP_ALIGN_NEAR_STEP_HOLD_MARGIN
                ):
                    linear = 0.0
                    mode = "step_align_rotate_in_place"
                elif (
                    math.isfinite(control_distance)
                    and control_distance <= STEP_ALIGN_CREEP_DEPTH_M
                ):
                    linear = STEP_ALIGN_CREEP_SPEED_M_S
                    mode = "step_align_creep"
                elif (
                    math.isfinite(control_distance)
                    and control_distance <= STEP_ALIGN_SLOWDOWN_DEPTH_M
                ):
                    linear = STEP_ALIGN_SLOW_SPEED_M_S
                    mode = "step_align_slow"
                else:
                    linear = STEP_ALIGN_FORWARD_SPEED
                    mode = "step_align_heading"

                return linear, angular, mode

            # 黑线丢失但台阶姿态仍有效时，不能傻停等黑线。
            # 使用深度检测到的台阶前沿角度和左右偏移原地转正。
            det = self._latest_step_detection
            if (
                det is not None
                and bool(getattr(det, "detected", False))
                and bool(getattr(det, "pose_valid", False))
                and math.isfinite(float(getattr(det, "edge_angle_rad", float("nan"))))
                and math.isfinite(float(getattr(det, "lateral_offset_m", float("nan"))))
            ):
                edge_angle = float(det.edge_angle_rad)
                lateral_offset = float(det.lateral_offset_m)
                angular = clamp(
                    STEP_DEPTH_ALIGN_EDGE_KP * edge_angle
                    - STEP_DEPTH_ALIGN_LATERAL_KP * lateral_offset,
                    -STEP_DEPTH_ALIGN_MAX_ANGULAR,
                    STEP_DEPTH_ALIGN_MAX_ANGULAR,
                )
                self._last_good_angular = angular
                self._have_previous_error = False
                return 0.0, angular, "step_depth_pose_align"

            # 黑线和台阶姿态都不可用时才停车，避免盲目前进。
            # 如果随后深度也丢失，V1.43 会自动进入倒车恢复。
            return 0.0, 0.0, "step_align_line_lost_stop"

        if self._post_step_reacquire_active:
            if self.phase != "SEEK_ROUNDABOUT":
                self._stop_post_step_reacquire("阶段已切换")
            elif result.line_valid:
                self._stop_post_step_reacquire("已经重新看到黑线")
            else:
                distance = self._post_step_reacquire_distance()
                if (
                    now <= self._post_step_reacquire_until
                    and distance <= STEP_POST_REACQUIRE_MAX_DISTANCE_M
                ):
                    angular = clamp(
                        STEP_POST_REACQUIRE_TURN_RAD_S,
                        -LOSS_MAX_ANGULAR,
                        LOSS_MAX_ANGULAR,
                    )
                    self._last_good_angular = angular
                    self._have_previous_error = False
                    direction = "right" if angular < 0.0 else "left"
                    return (
                        STEP_POST_REACQUIRE_SPEED_M_S,
                        angular,
                        f"step_post_reacquire_{direction}",
                    )
                self._stop_post_step_reacquire("超过找线时间/距离仍未看到黑线")
                return 0.0, 0.0, "step_post_reacquire_stop"

        if result.line_valid:
            self._last_valid_line_time = now
            angular = self._normal_angular(result, now)

            if self.phase == "ROUNDABOUT_FOLLOW":
                # 本次失败录像的关键帧约 28.75s：
                # reason=line_edge_single_rescue, e=+0.64，普通控制给出右转 w<0，
                # 于是抓住了回 3-4 的来路。环岛出口阶段遇到这种“仅右侧单点”时，
                # 宁可按 ICN 左找，也不能相信它并右拐。
                if (
                    ROUNDABOUT_REJECT_RIGHT_EDGE_RESCUE
                    and result.reason == "line_edge_single_rescue"
                    and angular < 0.0
                ):
                    self._last_good_angular = ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR
                    return (
                        ROUNDABOUT_BLIND_RECOVERY_SPEED,
                        ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR,
                        "roundabout_reject_right_keep_arc",
                    )

                # 左环 + 正确出口都不需要大幅右拐。允许极小右修正，但禁止选回程支路。
                angular = max(angular, ROUNDABOUT_FOLLOW_MIN_ANGULAR)

                if now < self._roundabout_exit_guard_until:
                    angular = max(angular, ROUNDABOUT_EXIT_GUARD_MIN_ANGULAR)

            self._last_good_angular = angular
            turn_ratio = min(1.0, abs(angular) / MAX_ANGULAR_SPEED)
            # V1.3：远场只负责“告诉车提前减速”，不直接决定转向。
            # 这样进入 S 弯/90° 弯时会先把速度降下来，再由近场把车身拉回线中心，
            # 避免为了追远处曲线而切内弯。
            local_curve = abs(result.mid_error - result.near_error)
            far_curve = abs(result.far_error - result.mid_error)
            total_curve = abs(result.far_error - result.near_error)
            geometry_curve = min(1.0, max(local_curve, 0.80 * far_curve, 0.65 * total_curve))
            centering_need = min(1.0, 1.35 * abs(result.near_error))
            slowdown = max(turn_ratio, 0.95 * geometry_curve, 0.80 * centering_need)
            linear = max(
                MINIMUM_FORWARD_SPEED,
                FORWARD_SPEED * (1.0 - CURVE_SLOWDOWN * slowdown),
            )

            # V1.52：普通赛道/S弯只通过降速提高“骑中线”能力；环岛FOLLOW/EXIT明确排除。
            # 普通弯限制到0.17m/s，急弯限制到0.145m/s；直线仍可0.25m/s。
            if (
                CURVE_CENTER_RIDING_ENABLED
                and self.phase not in ("ROUNDABOUT_FOLLOW", "ROUNDABOUT_EXIT")
            ):
                if geometry_curve >= CURVE_CENTER_STRONG_SHAPE:
                    linear = min(linear, CURVE_CENTER_STRONG_SPEED_CAP)
                elif geometry_curve >= CURVE_CENTER_SHAPE_THRESHOLD:
                    linear = min(linear, CURVE_CENTER_SPEED_CAP)

            if result.reason == "line_edge_single_rescue":
                linear = min(linear, LINE_EDGE_RESCUE_SPEED_CAP)
            elif result.reason == "line_hard_turn_centering":
                linear = min(linear, LINE_HARD_TURN_SPEED_CAP)

            # V1.48：第一圈是从静止起步，第二圈过黄线后仍带着速度。
            # 第二圈到台阶之前专门留一个低速观察区，避免一晃就把深度候选窗口错过。
            if (
                SECOND_LAP_PRESTEP_SLOW_ENABLED
                and self.lap_count >= 1
                and not self._step_done_this_lap
                and self.phase == "SEEK_ROUNDABOUT"
                and self._seek_distance >= SECOND_LAP_PRESTEP_SLOW_START_M
                and not self._step_fixed_takeover_latched
            ):
                linear = min(linear, SECOND_LAP_PRESTEP_SPEED_CAP)

            if (
                self.phase not in ("ROUNDABOUT_FOLLOW", "ROUNDABOUT_EXIT")
                and (
                    abs(angular) >= BIG_TURN_LOSS_MIN_RECENT_TURN
                    or result.reason
                    in ("line_curve_center_riding", "line_hard_turn_centering", "line_edge_single_rescue")
                )
            ):
                self._last_curve_turn_angular = angular
                self._last_curve_turn_time = now

            if self.phase == "ROUNDABOUT_FOLLOW":
                if now < self._roundabout_exit_guard_until:
                    linear = min(linear, ROUNDABOUT_EXIT_GUARD_SPEED)
                    return linear, angular, "roundabout_exit_guard"
                return linear, angular, "roundabout_follow_left_locked"
            return linear, angular, "tracking"

        # V1.19：严格只有 ROUNDABOUT_FOLLOW 才复制 ICN 的默认左转找线。
        # 有线时上面走正常循迹；只有当前帧确实丢线才进入这里。\n        # V1.41 丢线恢复保持约0.625m转弯半径，不再用0.12m小半径原地绕圈。
        if self.phase == "ROUNDABOUT_FOLLOW":
            self._last_good_angular = ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR
            return (
                ROUNDABOUT_BLIND_RECOVERY_SPEED,
                ROUNDABOUT_BLIND_RECOVERY_LEFT_ANGULAR,
                "roundabout_blind_arc_recovery",
            )

        # 普通赛道 90 度大弯丢线：沿最近一次明显转弯方向低速续弯找线。
        # 只给很短时间，找不到仍会停车，避免长期盲跑。
        elapsed = now - self._last_valid_line_time if self._last_valid_line_time > 0 else 99.0
        hint_age = now - self._last_curve_turn_time if self._last_curve_turn_time > 0 else 99.0
        if (
            elapsed <= BIG_TURN_LOSS_SECONDS
            and hint_age <= BIG_TURN_LOSS_HINT_KEEP_SECONDS
            and abs(self._last_curve_turn_angular) >= BIG_TURN_LOSS_MIN_RECENT_TURN
        ):
            turn_sign = 1.0 if self._last_curve_turn_angular >= 0.0 else -1.0
            angular = turn_sign * max(
                BIG_TURN_LOSS_TURN,
                abs(self._last_curve_turn_angular) * 0.85,
            )
            angular = clamp(angular, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            self._last_good_angular = angular
            self._have_previous_error = False
            return BIG_TURN_LOSS_SPEED, angular, "big_turn_blind_recovery"

        if elapsed <= SHORT_LOSS_SECONDS:
            angular = self._last_good_angular * LOSS_ANGULAR_DECAY
            angular = clamp(angular, -LOSS_MAX_ANGULAR, LOSS_MAX_ANGULAR)
            return LOSS_FORWARD_SPEED, angular, "line_bridge_loss"
        if elapsed <= SEARCH_LOSS_SECONDS:
            return LOSS_SEARCH_SPEED, 0.0, "line_loss_hold"

        # 还没找到线就停车，等人工处理；继续盲跑更容易直接冲出赛道。
        return 0.0, 0.0, "line_lost_stop"

    def _publish_command(self, linear: float, angular: float) -> None:
        msg = Twist()
        msg.linear.x = float(LINEAR_DIRECTION_SIGN * linear)
        msg.angular.z = float(angular)
        self.cmd_vel_pub.publish(msg)

    def _publish_zero(self) -> None:
        try:
            self._publish_command(0.0, 0.0)
        except Exception:
            pass
        self._last_command = (0.0, 0.0)

    def _control_timer(self) -> None:
        try:
            self._run_control_cycle()
        except Exception as error:
            self.get_logger().error(f"控制异常，立即停车: {error}")
            self._publish_zero()

    def _run_control_cycle(self) -> None:
        now = time.monotonic()

        # Once a step is locked, the tested approach/jump sequence owns motion until
        # LAND+RECOVER finishes. It must not depend on RGB frame timing.
        if self._step_active():
            step_linear, step_angular, step_mode = self._run_active_step_cycle(now)
            self._log_status(
                now,
                f"台阶：{step_text(self._step_state)}；"
                f"速度{step_linear:.2f}，已走{self._step_trigger_traveled:.2f}m，"
                f"剩余{self._step_remaining:.2f}m",
            )
            return

        with self._frame_lock:
            healthy = self._camera_healthy
            frame_time = self._latest_frame_time
            sequence = self._frame_sequence
            frame = self._latest_frame.copy() if self._latest_frame is not None else None

        if not healthy or frame is None or now - frame_time > IMAGE_TIMEOUT_SECONDS:
            self._publish_zero()
            self._write_race_log(None, now, "camera_timeout", 0.0, 0.0)
            self._log_status(now, "STOP camera unavailable/timeout")
            return
        if sequence == self._last_processed_sequence:
            self._publish_command(*self._last_command)
            return
        self._last_processed_sequence = sequence

        if not self._race_clock_started:
            self._race_clock_started = True
            self.phase_enter_time = now
            self._reset_seek_roundabout_progress()
            self.get_logger().info(
                "收到第一帧有效图像：比赛视觉计时和寻找环岛局部里程从这里开始"
            )

        self._record_raw_frame(frame)
        # 每圈：台阶完成前使用 +0.12 -> -0.12 动态偏移；
        # 台阶完成后本圈立即固定为 0 偏移。第一圈结束时
        # _step_done_this_lap 会重新置 False，第二圈自动再次启用。
        result = self.vision.process(
            frame,
            roundabout=(
                self.phase
                in ("ROUNDABOUT", "ROUNDABOUT_FOLLOW", "ROUNDABOUT_EXIT")
            ),
            use_dynamic_offset=(not self._step_done_this_lap),
            show_yellow_debug=(
                self.phase in ("SEEK_FINISH", "FINISH_PASS")
            ),
        )

        # Depth detector runs in parallel. Only after a confirmed 5cm step is locked
        # do we pause the normal course state machine and hand motion to the tested jump controller.
        step_locked_now = self._try_lock_step(result, now)
        if step_locked_now or self._step_active():
            step_linear, step_angular, mode = self._run_active_step_cycle(now)
            self._draw_debug(result, step_linear, step_angular, mode, now)
            return

        # V1.35：只要台阶候选已经接管“摆正/标定靠近”，就冻结赛道事件状态机。
        # 原来的问题是：_try_lock_step() 尚未真正 LOCK 时会返回 False，
        # 但 _step_waiting_alignment=True；随后程序仍执行 _sync_course_state()。
        # 台阶前的黄黑纹理会同时产生较高 gate/yellow 分数，于是误把台阶区
        # 当成环岛虚线入口，导致 SEEK_ROUNDABOUT -> ROUNDABOUT_APPROACH。
        #
        # 冻结期间视觉仍正常运行、调试分数仍显示，但绝不推进环岛/斑马线/终点状态。
        # 台阶完成后 _step_waiting_alignment=False，状态机自动恢复。
        if not self._step_waiting_alignment:
            self._sync_course_state(result, now)

        planned_linear, planned_angular, mode = self._planned_command(result, now)
        if self._step_waiting_alignment:
            mode = mode + "/course_events_frozen"
        if self.phase in ("FINISHED", "CROSSWALK_STOP"):
            cmd_linear, cmd_angular = 0.0, 0.0
            self._last_motion_update_time = now
        else:
            cmd_linear, cmd_angular = self._limit_command(
                planned_linear, planned_angular, now
            )
        planned_cmd_linear, planned_cmd_angular = cmd_linear, cmd_angular
        if ENABLE_MOTION:
            self._publish_command(cmd_linear, cmd_angular)
            self._last_command = (cmd_linear, cmd_angular)
        else:
            self._publish_command(0.0, 0.0)
            self._last_command = (0.0, 0.0)
            mode = "DRY_RUN/" + mode

        self._draw_debug(result, planned_cmd_linear, planned_cmd_angular, mode, now)
        self._write_race_log(result, now, mode, planned_cmd_linear, planned_cmd_angular)
        phase_age = now - self.phase_enter_time
        if self.phase == "SEEK_ROUNDABOUT":
            roundabout_log = f"环岛未进入，本圈{self._seek_distance:.2f}m"
        elif self.phase == "ROUNDABOUT_APPROACH":
            approach_target = self._roundabout_approach_target()
            comp_text = (
                f"，补偿{self._roundabout_approach_compensation:.2f}m"
                if self._roundabout_approach_compensation > 0.0
                else ""
            )
            roundabout_log = (
                f"接近环岛{self._approach_distance:.2f}/"
                f"{approach_target:.2f}m{comp_text}"
            )
        elif self.phase == "ROUNDABOUT":
            roundabout_log = (
                f"进环左转{self._rb_entry_displacement:.2f}/"
                f"{ROUNDABOUT_ENTRY_TURN_DISTANCE:.2f}m"
            )
        elif self.phase == "ROUNDABOUT_FOLLOW":
            roundabout_log = (
                f"找出口{phase_age:.1f}/{ROUNDABOUT_FOLLOW_SECONDS:.1f}s"
            )
        elif self.phase == "ROUNDABOUT_EXIT":
            roundabout_log = (
                f"出环保护{phase_age:.1f}/{ROUNDABOUT_EXIT_COMMIT_SECONDS:.1f}s"
            )
        else:
            roundabout_log = "环岛已结束"
        self._log_status(
            now,
            f"阶段：{phase_text(self.phase)} 第{min(self.lap_count + 1, TOTAL_LAPS)}/{TOTAL_LAPS}圈；"
            f"线={'有' if result.line_valid else '无'}；"
            f"速度{planned_cmd_linear:.2f} 转向{planned_cmd_angular:+.2f}；"
            f"台阶距{(float(self._latest_step_detection.distance) if self._latest_step_detection is not None else float('nan')):.2f}m；"
            f"{roundabout_log}；"
            f"人行道{result.crosswalk_score:.2f} 黄线{result.yellow_score:.2f}",
        )

    # ------------------------------ debug / record -------------------------

    def _output_log_dir(self) -> Path:
        path = Path(__file__).resolve().parent / "log"
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _open_race_log(self) -> bool:
        if self._race_log_failed:
            return False
        if self._race_log_writer is not None:
            return True

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self._output_log_dir() / f"race_log_{timestamp}.csv"
        fieldnames = [
            "t",
            "phase",
            "phase_age",
            "lap",
            "seek_distance",
            "roundabout_gate_score",
            "roundabout_gate_evidence",
            "roundabout_gate_source",
            "first_roundabout_gate_seek",
            "roundabout_gate_first_seen_seek",
            "roundabout_approach_compensation",
            "roundabout_entry_age",
            "roundabout_entry_disp",
            "mode",
            "cmd_v",
            "cmd_w",
            "line_valid",
            "line_reason",
            "line_error",
            "near_error",
            "mid_error",
            "far_error",
            "line_confidence",
            "valid_bands",
            "crosswalk_detected",
            "crosswalk_score",
            "yellow_detected",
            "yellow_score",
            "near_mid_delta",
            "mid_far_delta",
            "near_far_delta",
            "center_error",
            "last_cmd_w",
            "step_state",
            "step_done_this_lap",
            "step_waiting_alignment",
            "step_align_reason",
            "step_fixed_latched",
            "step_reverse_active",
            "step_candidate_evidence",
            "step_detected",
            "step_distance",
            "step_filtered_distance",
            "step_height",
            "step_confidence",
            "step_pose_valid",
            "step_edge_angle_rad",
            "step_lateral_offset_m",
            "step_confirm_frames",
            "step_required_frames",
            "step_target_travel",
            "step_commanded_traveled",
            "step_odom_traveled",
            "step_trigger_traveled",
            "step_remaining",
            "step_stop_reason",
            "odom_fresh",
            "yaw_error",
            "jump_profile_min",
            "jump_profile_max",
        ]

        try:
            self._race_log_file = path.open("w", newline="", encoding="utf-8")
            self._race_log_writer = csv.DictWriter(
                self._race_log_file,
                fieldnames=fieldnames,
            )
            self._race_log_writer.writeheader()
            self._race_log_path = path
            self.get_logger().info(f"比赛CSV日志: {path}")
            return True
        except OSError as error:
            self._race_log_failed = True
            self.get_logger().warn(f"比赛CSV日志创建失败，继续比赛: {error}")
            return False

    def _write_race_log(
        self,
        result: Optional[VisionResult],
        now: float,
        mode: str,
        linear: float,
        angular: float,
    ) -> None:
        if not self._open_race_log() or self._race_log_writer is None:
            return

        det = self._latest_step_detection
        det_d = float(det.distance) if det is not None else float("nan")
        det_h = float(det.height) if det is not None else float("nan")
        det_c = float(det.confidence) if det is not None else 0.0
        det_ok = int(bool(det.detected)) if det is not None else 0
        det_frames = int(det.confirm_frames) if det is not None else 0
        det_need = int(det.required_confirm_frames) if det is not None else 0
        det_pose_ok = int(bool(getattr(det, "pose_valid", False))) if det is not None else 0
        det_edge_angle = (
            float(getattr(det, "edge_angle_rad", float("nan")))
            if det is not None
            else float("nan")
        )
        det_lateral_offset = (
            float(getattr(det, "lateral_offset_m", float("nan")))
            if det is not None
            else float("nan")
        )

        if result is None:
            line_valid = 0
            line_reason = ""
            line_error = near_error = mid_error = far_error = float("nan")
            line_confidence = 0.0
            valid_bands = 0
            roundabout_gate_score = 0.0
            crosswalk_detected = 0
            crosswalk_score = 0.0
            yellow_detected = 0
            yellow_score = 0.0
        else:
            line_valid = int(bool(result.line_valid))
            line_reason = result.reason
            line_error = float(result.error)
            near_error = float(result.near_error)
            mid_error = float(result.mid_error)
            far_error = float(result.far_error)
            line_confidence = float(result.confidence)
            valid_bands = int(result.valid_bands)
            roundabout_gate_score = float(result.roundabout_gate_score)
            crosswalk_detected = int(bool(result.crosswalk))
            crosswalk_score = float(result.crosswalk_score)
            yellow_detected = int(bool(result.yellow_finish))
            yellow_score = float(result.yellow_score)

        near_mid_delta = abs(mid_error - near_error)
        mid_far_delta = abs(far_error - mid_error)
        near_far_delta = abs(far_error - near_error)
        center_error = (near_error + mid_error + far_error) / 3.0

        row = {
            "t": f"{now:.3f}",
            "phase": self.phase,
            "phase_age": f"{now - self.phase_enter_time:.3f}",
            "lap": self.lap_count,
            "seek_distance": f"{self._seek_distance:.3f}",
            "roundabout_gate_score": f"{roundabout_gate_score:.4f}",
            "roundabout_gate_evidence": f"{self._roundabout_gate_evidence:.4f}",
            "roundabout_gate_source": self._roundabout_gate_source,
            "first_roundabout_gate_seek": (
                f"{self._first_roundabout_gate_seek:.3f}"
                if self._first_roundabout_gate_seek is not None
                else ""
            ),
            "roundabout_gate_first_seen_seek": (
                f"{self._roundabout_gate_first_seen_seek:.3f}"
                if self._roundabout_gate_first_seen_seek is not None
                else ""
            ),
            "roundabout_approach_compensation": (
                f"{self._roundabout_approach_compensation:.3f}"
            ),
            "roundabout_entry_age": (
                f"{now - self.phase_enter_time:.3f}"
                if self.phase == "ROUNDABOUT"
                else ""
            ),
            "roundabout_entry_disp": (
                f"{self._rb_entry_displacement:.3f}"
                if self.phase == "ROUNDABOUT"
                else ""
            ),
            "mode": mode,
            "cmd_v": f"{linear:.4f}",
            "cmd_w": f"{angular:.4f}",
            "line_valid": line_valid,
            "line_reason": line_reason,
            "line_error": f"{line_error:.4f}",
            "near_error": f"{near_error:.4f}",
            "mid_error": f"{mid_error:.4f}",
            "far_error": f"{far_error:.4f}",
            "line_confidence": f"{line_confidence:.4f}",
            "valid_bands": valid_bands,
            "crosswalk_detected": crosswalk_detected,
            "crosswalk_score": f"{crosswalk_score:.4f}",
            "yellow_detected": yellow_detected,
            "yellow_score": f"{yellow_score:.4f}",
            "near_mid_delta": f"{near_mid_delta:.4f}",
            "mid_far_delta": f"{mid_far_delta:.4f}",
            "near_far_delta": f"{near_far_delta:.4f}",
            "center_error": f"{center_error:.4f}",
            "last_cmd_w": f"{self._last_command[1]:.4f}",
            "step_state": self._step_state,
            "step_done_this_lap": int(self._step_done_this_lap),
            "step_waiting_alignment": int(self._step_waiting_alignment),
            "step_align_reason": self._step_align_last_reason,
            "step_fixed_latched": int(self._step_fixed_takeover_latched),
            "step_reverse_active": int(self._step_fixed_reverse_active),
            "step_candidate_evidence": f"{self._step_race_candidate_evidence:.3f}",
            "step_detected": det_ok,
            "step_distance": f"{det_d:.4f}",
            "step_filtered_distance": f"{self._filtered_step_distance(now):.4f}",
            "step_height": f"{det_h:.4f}",
            "step_confidence": f"{det_c:.4f}",
            "step_pose_valid": det_pose_ok,
            "step_edge_angle_rad": f"{det_edge_angle:.4f}",
            "step_lateral_offset_m": f"{det_lateral_offset:.4f}",
            "step_confirm_frames": det_frames,
            "step_required_frames": det_need,
            "step_target_travel": f"{self._step_target_travel:.4f}",
            "step_commanded_traveled": f"{self._step_commanded_traveled:.4f}",
            "step_odom_traveled": f"{self._step_odom_traveled:.4f}",
            "step_trigger_traveled": f"{self._step_trigger_traveled:.4f}",
            "step_remaining": f"{self._step_remaining:.4f}",
            "step_stop_reason": self._step_stop_reason,
            "odom_fresh": int(self._step_odom_fresh(now)),
            "yaw_error": f"{self._step_yaw_error:.4f}",
            "jump_profile_min": f"{STEP_JUMP_MIN_HEIGHT_M:.3f}",
            "jump_profile_max": f"{STEP_JUMP_MAX_HEIGHT_M:.3f}",
        }

        try:
            self._race_log_writer.writerow(row)
            if self._race_log_file is not None:
                self._race_log_file.flush()
        except (OSError, ValueError) as error:
            self._race_log_failed = True
            self.get_logger().warn(f"比赛CSV日志写入失败，后续停止记录: {error}")

    def _open_raw_writer(self, frame: np.ndarray) -> bool:
        if self._raw_failed or not RECORD_RAW_VIDEO:
            return False
        h, w = frame.shape[:2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self._output_log_dir() / f"race_raw_{timestamp}.avi"
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*DEBUG_VIDEO_CODEC),
            CONTROL_RATE,
            (w, h),
        )
        if not writer.isOpened():
            writer.release()
            self._raw_failed = True
            self.get_logger().warn("RAW 原始录像创建失败，继续比赛")
            return False
        self._raw_writer = writer
        self._raw_path = path
        self.get_logger().info(f"RAW 原始录像: {path}")
        return True

    def _record_raw_frame(self, frame: np.ndarray) -> None:
        if not RECORD_RAW_VIDEO or self._raw_failed:
            return
        if self._raw_writer is None and not self._open_raw_writer(frame):
            return
        try:
            self._raw_writer.write(frame)
        except cv2.error:
            self._raw_failed = True
            if self._raw_writer is not None:
                self._raw_writer.release()
                self._raw_writer = None

    def _open_video_writer(self, frame: np.ndarray) -> bool:
        if self._video_failed or not RECORD_DEBUG_VIDEO:
            return False
        h, w = frame.shape[:2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self._output_log_dir() / f"race_vision_{timestamp}.avi"
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*DEBUG_VIDEO_CODEC),
            CONTROL_RATE,
            (w, h),
        )
        if not writer.isOpened():
            writer.release()
            self._video_failed = True
            self.get_logger().warn("调试录像创建失败，继续比赛")
            return False
        self._video_writer = writer
        self._video_path = path
        self.get_logger().info(f"调试录像: {path}")
        return True

    def _record_frame(self, frame: np.ndarray) -> None:
        if not RECORD_DEBUG_VIDEO or self._video_failed:
            return
        if self._video_writer is None and not self._open_video_writer(frame):
            return
        try:
            self._video_writer.write(frame)
        except cv2.error:
            self._video_failed = True
            if self._video_writer is not None:
                self._video_writer.release()
                self._video_writer = None

    def _draw_debug(
        self,
        result: VisionResult,
        linear: float,
        angular: float,
        mode: str,
        now: float,
    ) -> None:
        debug = result.debug_frame
        phase_age = now - self.phase_enter_time
        det = self._latest_step_detection
        det_d = float(det.distance) if det is not None else float("nan")
        det_h = float(det.height) if det is not None else float("nan")
        det_c = float(det.confidence) if det is not None else 0.0
        det_ok = int(bool(det.detected)) if det is not None else 0
        det_pose_ok = bool(getattr(det, "pose_valid", False)) if det is not None else False
        det_angle = (
            float(getattr(det, "edge_angle_rad", float("nan")))
            if det is not None
            else float("nan")
        )
        det_lateral = (
            float(getattr(det, "lateral_offset_m", float("nan")))
            if det is not None
            else float("nan")
        )
        line_text = "看到线" if result.line_valid else "丢线"
        race_step_usable = (
            bool(det_ok)
            and math.isfinite(det_d)
            and math.isfinite(det_h)
            and det_d <= STEP_RACE_CANDIDATE_MAX_DISTANCE_M
            and STEP_RACE_CANDIDATE_MIN_HEIGHT_M
                <= det_h <= STEP_RACE_CANDIDATE_MAX_HEIGHT_M
            and det_c >= STEP_RACE_CANDIDATE_MIN_CONFIDENCE
        )
        step_status = "已确认" if race_step_usable else "未确认"
        if self._step_state == "JUMP_CONFIRM":
            step_status = "等回车"
        elif self._step_waiting_alignment:
            step_status = "摆正中"
        elif self._post_step_reacquire_active:
            step_status = "跳后找线"
        lap = min(self.lap_count + 1, TOTAL_LAPS)
        if self.phase == "SEEK_ROUNDABOUT":
            roundabout_text = f"环岛：未进入   本圈已走 {self._seek_distance:.2f}m"
        elif self.phase == "ROUNDABOUT_APPROACH":
            approach_target = self._roundabout_approach_target()
            comp_text = (
                f" 补{self._roundabout_approach_compensation:.2f}m"
                if self._roundabout_approach_compensation > 0.0
                else ""
            )
            roundabout_text = (
                f"环岛：接近入口 {self._approach_distance:.2f}/"
                f"{approach_target:.2f}m{comp_text}"
            )
        elif self.phase == "ROUNDABOUT":
            roundabout_text = (
                f"环岛：固定左转 {self._rb_entry_displacement:.2f}/"
                f"{ROUNDABOUT_ENTRY_TURN_DISTANCE:.2f}m"
            )
        elif self.phase == "ROUNDABOUT_FOLLOW":
            roundabout_text = (
                f"环岛：寻找出口 {phase_age:.1f}/"
                f"{ROUNDABOUT_FOLLOW_SECONDS:.1f}s"
            )
        elif self.phase == "ROUNDABOUT_EXIT":
            roundabout_text = (
                f"环岛：出环保护 {phase_age:.1f}/"
                f"{ROUNDABOUT_EXIT_COMMIT_SECONDS:.1f}s"
            )
        else:
            roundabout_text = "环岛：已结束"
        lines = [
            (
                f"状态：{phase_text(self.phase)}   第 {lap}/{TOTAL_LAPS} 圈   {line_text}",
                (0, 255, 0) if self.phase != "FINISHED" else (0, 0, 255),
            ),
            (
                f"速度：前进 {linear:.2f}m/s   转向 {angular:+.2f}",
                (255, 255, 255),
            ),
            (
                f"台阶：{step_status}   距离 {det_d:.2f}m   高度 {det_h:.2f}m   可信 {det_c:.2f}",
                (0, 200, 255),
            ),
            (
                roundabout_text,
                (255, 220, 120),
            ),
            (
                f"人行道 {result.crosswalk_score:.2f}   黄线 {result.yellow_score:.2f}",
                (180, 255, 180),
            ),
        ]
        if det_pose_ok and self._step_waiting_alignment:
            lines.append(
                (
                    f"台阶摆正：角度 {math.degrees(det_angle):+.1f}deg   左右 {det_lateral:+.2f}m",
                    (0, 200, 255),
                )
            )
        if self._step_state == "JUMP_CONFIRM":
            lines.append(("已到起跳点：按回车跳，Ctrl+C取消", (0, 180, 255)))
        elif self._post_step_reacquire_active:
            distance = self._post_step_reacquire_distance()
            direction = "右" if STEP_POST_REACQUIRE_TURN_RAD_S < 0.0 else "左"
            lines.append(
                (
                    f"跳后找线：向{direction} {distance:.2f}/"
                    f"{STEP_POST_REACQUIRE_MAX_DISTANCE_M:.2f}m",
                    (0, 180, 255),
                )
            )
        debug = draw_overlay(debug, lines, font_size=19, line_height=26)
        self._record_frame(debug)

        if self._debug_window_ready:
            try:
                if not self._debug_window_sized:
                    h, w = debug.shape[:2]
                    cv2.resizeWindow("WheelLeg Race Vision", w, h)
                    self._debug_window_sized = True
                cv2.imshow("WheelLeg Race Vision", debug)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    self.request_exit()
            except cv2.error:
                self._debug_window_ready = False

    def _log_status(self, now: float, text: str) -> None:
        if now - self._last_status_log >= STATUS_LOG_PERIOD:
            self.get_logger().info(text)
            self._last_status_log = now

    # ------------------------------ lifecycle -------------------------------
    @property
    def exit_requested(self) -> bool:
        return self._exit_requested

    def request_exit(self) -> None:
        self._publish_zero()
        self._exit_requested = True

    def close(self) -> None:
        self._closing = True
        self._stop_capture.set()
        with self._capture_lock:
            capture = self._capture
        if capture is not None:
            capture.release()
        if hasattr(self, "_capture_thread"):
            self._capture_thread.join(timeout=1.0)
        if rclpy.ok():
            for _ in range(10):
                self._publish_command(0.0, 0.0)
                self._publish_jump_false_only()
                time.sleep(0.02)
        if self._raw_writer is not None:
            self._raw_writer.release()
            self._raw_writer = None
            if self._raw_path is not None:
                self.get_logger().info(f"RAW 原始录像已保存: {self._raw_path}")
        if self._video_writer is not None:
            self._video_writer.release()
            self._video_writer = None
            if self._video_path is not None:
                self.get_logger().info(f"调试录像已保存: {self._video_path}")
        if self._race_log_file is not None:
            try:
                self._race_log_file.close()
            except OSError:
                pass
            self._race_log_file = None
            if self._race_log_path is not None:
                self.get_logger().info(f"比赛CSV日志已保存: {self._race_log_path}")
        if self._debug_window_ready:
            try:
                cv2.destroyWindow("WheelLeg Race Vision")
            except cv2.error:
                pass


def main(args=None) -> None:
    node: Optional[WheelLegRaceNode] = None
    exit_signal = threading.Event()

    def handle_signal(signum, _frame) -> None:
        exit_signal.set()
        if node is not None:
            node.request_exit()

    old_sigint = signal.signal(signal.SIGINT, handle_signal)
    old_sigterm = signal.signal(signal.SIGTERM, handle_signal)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = WheelLegRaceNode()
        while rclpy.ok() and not exit_signal.is_set() and not node.exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, old_sigint)
        signal.signal(signal.SIGTERM, old_sigterm)


if __name__ == "__main__":
    main()
