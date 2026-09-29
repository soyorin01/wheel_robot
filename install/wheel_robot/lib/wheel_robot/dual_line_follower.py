#!/usr/bin/env python3

from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import signal
import time
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


# ======================== 默认参数（推荐在 config/dual_line_follower.yaml 调整） ========================

ENABLE_MOTION = True

# ---------- 最常用：速度参数 ----------
NORMAL_SPEED = 0.60
MIN_SPEED = 0.60
MAX_ANGULAR_SPEED = 0.56
CURVE_SLOWDOWN = 0.45
LOST_SEARCH_SPEED = 0.055
LOST_SEARCH_ANGULAR = 0.18

# 摄像头与处理尺寸。
CAMERA_BACKEND = "v4l2"
CAMERA_DEVICE = "/dev/video0"
CAMERA_ID = 0
CAMERA_PIXEL_FORMAT = "YUYV"
CAPTURE_FPS = 30.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
IMAGE_TIMEOUT_SECONDS = 0.35
CONTROL_RATE_HZ = 25.0

# 双边白线通常在画面下半部分；底部略裁掉一点，避开车体/强畸变。
ROI_TOP_RATIO = 0.42
ROI_BOTTOM_RATIO = 0.96
ROI_LEFT_RATIO = 0.00
ROI_RIGHT_RATIO = 1.00
PROCESS_WIDTH = 320
PROCESS_HEIGHT = 160

# 车道中心偏置。正数让车道中心保持在画面右侧，车体实际向左偏。
LANE_TARGET_OFFSET_PX = 0.0

# HSV 白线分割：低饱和度 + 动态亮度 + 红底排除。
WHITE_H_MIN = 0
WHITE_H_MAX = 180
WHITE_S_MAX = 72
DYNAMIC_V_PERCENTILE = 82.0
DYNAMIC_V_OFFSET = 16.0
DYNAMIC_V_MIN = 115
DYNAMIC_V_MAX = 225
STRONG_WHITE_V_MARGIN = 28
ADAPTIVE_BLOCK_SIZE = 31
ADAPTIVE_C = -5.0
LOCAL_DARK_C = 7.0
COOL_LINE_H_MIN = 25
COOL_LINE_H_MAX = 112
COOL_LINE_S_MIN = 8
COOL_LINE_S_MAX = 150
COOL_LINE_V_MIN = 48

RED_H_LOW_1 = 0
RED_H_HIGH_1 = 13
RED_H_LOW_2 = 167
RED_H_HIGH_2 = 180
RED_S_MIN = 65
RED_V_MIN = 35

OPEN_KERNEL_SIZE = 3
CLOSE_KERNEL_SIZE = 5
OPEN_ITERATIONS = 1
CLOSE_ITERATIONS = 1

# 三段融合。近端负责稳定当前姿态，中远端提前看弯。
FAR_RANGE = (0.00, 0.32)
MID_RANGE = (0.32, 0.66)
NEAR_RANGE = (0.66, 1.00)
NEAR_WEIGHT = 0.52
MID_WEIGHT = 0.32
FAR_WEIGHT = 0.26

# 单条边线宽度和左右线间距（处理图像像素）。
MIN_LINE_WIDTH_PX = 2.0
MAX_LINE_WIDTH_PX = 48.0
EXPECTED_LINE_WIDTH_PX = 12.0
# 透视下远处两边线间距会变窄，三段使用不同的合理范围。
NEAR_LANE_WIDTH_RATIO = (0.46, 0.90, 0.66)
MID_LANE_WIDTH_RATIO = (0.34, 0.84, 0.56)
FAR_LANE_WIDTH_RATIO = (0.22, 0.74, 0.42)
LANE_WIDTH_TOLERANCE_RATIO = 0.24
MIN_PAIR_ROW_COVERAGE = 0.18
MAX_CENTER_JUMP_RATIO = 0.18
TRACK_MEMORY_FRAMES = 8
MIN_DETECTION_CONFIDENCE = 0.08
MIN_ACTIVE_SEGMENTS = 1
MIN_WHITE_PIXEL_RATIO = 0.0008
MAX_WHITE_PIXEL_RATIO = 0.20
ALLOW_SINGLE_BOUNDARY_MEMORY = True
SINGLE_BOUNDARY_QUALITY = 0.32

# PD/PID。error>0 表示车道中心在画面右侧；本车右转通常要求 angular.z<0。
KP = 0.64
KI = 0.00
KD = 0.025
STEERING_SIGN = -1.0
INTEGRAL_LIMIT = 0.70
ERROR_FILTER_ALPHA = 0.36
DERIVATIVE_FILTER_ALPHA = 0.24
MAX_ERROR_DERIVATIVE = 2.6
MAX_LINEAR_ACCELERATION = 0.45
MAX_ANGULAR_ACCELERATION = 1.70
ERROR_SLOWDOWN_START = 0.18

LOST_STOP_SECONDS = 0.75

# 正式巡线建议关闭窗口；录像/CSV 继续保存，便于复盘。
SHOW_DEBUG_WINDOW = False
DEBUG_WINDOW_NAME = "Dual White Line (Q/Esc = STOP)"
OUTPUT_DIRECTORY = Path(__file__).resolve().parents[3] / "recordings"
RECORD_DEBUG_VIDEO = True
DEBUG_EVERY_N_FRAMES = 3
DEBUG_OUTPUT_SCALE = 0.5
VIDEO_CODEC = "MJPG"
VIDEO_FPS = 8.0
LOG_FLUSH_SECONDS = 1.0
STATUS_LOG_SECONDS = 2.0
CMD_VEL_TOPIC = "/cmd_vel"


Run = Tuple[int, int, float, int]


@dataclass
class SegmentLaneDetection:
    name: str
    y0: int
    y1: int
    valid: bool = False
    center_x: Optional[float] = None
    left_x: Optional[float] = None
    right_x: Optional[float] = None
    lane_width: Optional[float] = None
    row_coverage: float = 0.0
    quality: float = 0.0
    reason: str = "none"


@dataclass
class DetectionResult:
    valid: bool
    center_x: Optional[float]
    left_x: Optional[float]
    right_x: Optional[float]
    lane_width: Optional[float]
    error: float
    confidence: float
    near: SegmentLaneDetection
    mid: SegmentLaneDetection
    far: SegmentLaneDetection
    mask: np.ndarray
    roi_bgr: np.ndarray
    dynamic_v_threshold: int
    white_pixel_ratio: float
    reason: str


class DualWhiteLineDetector:
    """Find paired white boundaries and follow the midpoint between them."""

    def __init__(self) -> None:
        self._previous_center: Dict[str, float] = {}
        self._previous_width: Dict[str, float] = {}
        self._misses: Dict[str, int] = {"near": 0, "mid": 0, "far": 0}
        self._open_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE)
        )
        self._close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE)
        )

    @staticmethod
    def _clip_odd(value: int) -> int:
        value = max(3, int(value))
        return value if value % 2 == 1 else value + 1

    @staticmethod
    def _runs(row: np.ndarray) -> List[Run]:
        padded = np.pad((row > 0).astype(np.int8), (1, 1))
        changes = np.diff(padded)
        runs: List[Run] = []
        for start, end in zip(
            np.flatnonzero(changes == 1),
            np.flatnonzero(changes == -1),
        ):
            width = int(end - start)
            if MIN_LINE_WIDTH_PX <= width <= MAX_LINE_WIDTH_PX:
                runs.append((int(start), int(end), 0.5 * (start + end - 1), width))
        return runs

    @staticmethod
    def _lane_width_profile(name: str) -> Tuple[float, float, float]:
        if name == "near":
            return NEAR_LANE_WIDTH_RATIO
        if name == "mid":
            return MID_LANE_WIDTH_RATIO
        return FAR_LANE_WIDTH_RATIO

    def _make_mask(self, roi_bgr: np.ndarray) -> Tuple[np.ndarray, int]:
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        low_saturation = s <= WHITE_S_MAX
        sample = v[low_saturation]
        if sample.size >= 64:
            base_v = float(np.percentile(sample, DYNAMIC_V_PERCENTILE))
        else:
            base_v = float(np.percentile(v, DYNAMIC_V_PERCENTILE))
        dynamic_v = int(
            np.clip(base_v + DYNAMIC_V_OFFSET, DYNAMIC_V_MIN, DYNAMIC_V_MAX)
        )

        absolute_bright = v >= dynamic_v
        local_bright = cv2.adaptiveThreshold(
            v,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self._clip_odd(ADAPTIVE_BLOCK_SIZE),
            ADAPTIVE_C,
        ) > 0
        local_dark = cv2.adaptiveThreshold(
            v,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self._clip_odd(ADAPTIVE_BLOCK_SIZE),
            LOCAL_DARK_C,
        ) > 0
        strong_bright = v >= min(255, dynamic_v + STRONG_WHITE_V_MARGIN)

        red_1 = (
            (h >= RED_H_LOW_1)
            & (h <= RED_H_HIGH_1)
            & (s >= RED_S_MIN)
            & (v >= RED_V_MIN)
        )
        red_2 = (
            (h >= RED_H_LOW_2)
            & (h <= RED_H_HIGH_2)
            & (s >= RED_S_MIN)
            & (v >= RED_V_MIN)
        )
        bright_white = (
            (h >= WHITE_H_MIN)
            & (h <= WHITE_H_MAX)
            & low_saturation
            & absolute_bright
            & (local_bright | strong_bright)
            & ~(red_1 | red_2)
        )
        cool_lane_paint = (
            (h >= COOL_LINE_H_MIN)
            & (h <= COOL_LINE_H_MAX)
            & (s >= COOL_LINE_S_MIN)
            & (s <= COOL_LINE_S_MAX)
            & (v >= COOL_LINE_V_MIN)
            & (local_dark | local_bright | strong_bright)
            & ~(red_1 | red_2)
        )
        white = bright_white | cool_lane_paint

        mask = white.astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, self._open_kernel, iterations=OPEN_ITERATIONS
        )
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_CLOSE, self._close_kernel, iterations=CLOSE_ITERATIONS
        )
        return mask, dynamic_v

    def _best_row_pair(
        self,
        runs: List[Run],
        reference_center: float,
        min_lane_width: float,
        max_lane_width: float,
        expected_lane_width: float,
        lane_tolerance: float,
        max_center_jump: float,
    ) -> Optional[Tuple[Run, Run, float, float]]:
        best: Optional[Tuple[float, Run, Run, float, float]] = None
        for left_index in range(len(runs) - 1):
            for right_index in range(left_index + 1, len(runs)):
                left = runs[left_index]
                right = runs[right_index]
                lane_width = right[2] - left[2]
                if not (min_lane_width <= lane_width <= max_lane_width):
                    continue
                center = 0.5 * (left[2] + right[2])
                if abs(center - reference_center) > max_center_jump:
                    continue
                lane_error = abs(lane_width - expected_lane_width) / lane_tolerance
                left_line_error = abs(left[3] - EXPECTED_LINE_WIDTH_PX)
                right_line_error = abs(right[3] - EXPECTED_LINE_WIDTH_PX)
                line_error = (left_line_error + right_line_error) / (
                    2.0 * max(1.0, MAX_LINE_WIDTH_PX - EXPECTED_LINE_WIDTH_PX)
                )
                center_error = abs(center - reference_center) / max(1.0, max_center_jump)
                symmetry_error = abs(left[3] - right[3]) / max(1.0, MAX_LINE_WIDTH_PX)
                score = lane_error + 0.35 * line_error + 0.35 * center_error + 0.15 * symmetry_error
                if best is None or score < best[0]:
                    best = (score, left, right, center, lane_width)
        if best is None:
            return None
        _score, left, right, center, lane_width = best
        return left, right, center, lane_width

    def _infer_from_single_boundary(
        self,
        runs: List[Run],
        reference_center: float,
        expected_lane_width: float,
    ) -> Optional[Tuple[float, float, Optional[float], Optional[float]]]:
        if not ALLOW_SINGLE_BOUNDARY_MEMORY or not runs:
            return None
        half_width = expected_lane_width * 0.5
        candidates: List[Tuple[float, float, Optional[float], Optional[float]]] = []
        for run in runs:
            x = float(run[2])
            left_center = x + half_width
            right_center = x - half_width
            if 0.0 <= left_center <= PROCESS_WIDTH:
                candidates.append((abs(left_center - reference_center), left_center, x, None))
            if 0.0 <= right_center <= PROCESS_WIDTH:
                candidates.append((abs(right_center - reference_center), right_center, None, x))
        if not candidates:
            return None
        _score, center, left_x, right_x = min(candidates, key=lambda item: item[0])
        return center, expected_lane_width, left_x, right_x

    def _segment_candidate(
        self,
        mask: np.ndarray,
        name: str,
        bounds: Tuple[float, float],
    ) -> SegmentLaneDetection:
        height, _width = mask.shape
        y0 = max(0, min(height - 1, int(round(bounds[0] * height))))
        y1 = max(y0 + 1, min(height, int(round(bounds[1] * height))))
        result = SegmentLaneDetection(name=name, y0=y0, y1=y1)
        band = mask[y0:y1]
        band_height = max(1, y1 - y0)
        previous_center = self._previous_center.get(name, PROCESS_WIDTH / 2.0)
        min_lane_ratio, max_lane_ratio, expected_lane_ratio = (
            self._lane_width_profile(name)
        )
        min_lane_width = PROCESS_WIDTH * min_lane_ratio
        max_lane_width = PROCESS_WIDTH * max_lane_ratio
        profile_expected_width = PROCESS_WIDTH * expected_lane_ratio
        previous_width = self._previous_width.get(
            name, profile_expected_width
        )
        expected_lane_width = 0.65 * previous_width + 0.35 * profile_expected_width
        lane_tolerance = max(4.0, PROCESS_WIDTH * LANE_WIDTH_TOLERANCE_RATIO)
        max_center_jump = PROCESS_WIDTH * MAX_CENTER_JUMP_RATIO

        centers: List[float] = []
        widths: List[float] = []
        left_points: List[float] = []
        right_points: List[float] = []
        line_widths: List[float] = []
        inferred_rows = 0

        for row in band:
            runs = self._runs(row)
            if len(runs) >= 2:
                pair = self._best_row_pair(
                    runs,
                    centers[-1] if centers else previous_center,
                    min_lane_width,
                    max_lane_width,
                    expected_lane_width,
                    lane_tolerance,
                    max_center_jump,
                )
                if pair is None and centers:
                    pair = self._best_row_pair(
                        runs,
                        centers[-1],
                        min_lane_width,
                        max_lane_width,
                        expected_lane_width,
                        lane_tolerance,
                        max_center_jump,
                    )
                if pair is not None:
                    left, right, center, lane_width = pair
                    centers.append(center)
                    widths.append(lane_width)
                    left_points.append(float(left[2]))
                    right_points.append(float(right[2]))
                    line_widths.extend((float(left[3]), float(right[3])))
                    continue

            inferred = self._infer_from_single_boundary(
                runs,
                centers[-1] if centers else previous_center,
                expected_lane_width,
            )
            if inferred is not None:
                center, lane_width, left_x, right_x = inferred
                centers.append(center)
                widths.append(lane_width)
                if left_x is not None:
                    left_points.append(left_x)
                if right_x is not None:
                    right_points.append(right_x)
                inferred_rows += 1

        if not centers:
            result.reason = "no_double_line"
            return result

        row_coverage = len(centers) / float(band_height)
        lane_width = float(np.median(widths))
        center_x = float(np.median(centers))
        lane_quality = max(
            0.0,
            1.0
            - abs(lane_width - profile_expected_width)
            / lane_tolerance,
        )
        if line_widths:
            line_quality = float(
                np.mean(
                    [
                        max(
                            0.0,
                            1.0
                            - abs(value - EXPECTED_LINE_WIDTH_PX)
                            / max(1.0, MAX_LINE_WIDTH_PX - EXPECTED_LINE_WIDTH_PX),
                        )
                        for value in line_widths
                    ]
                )
            )
        else:
            line_quality = 0.35
        if len(centers) > 1:
            continuity = float(
                np.mean(
                    np.maximum(
                        0.0,
                        1.0
                        - np.abs(np.diff(centers))
                        / max(1.0, PROCESS_WIDTH * MAX_CENTER_JUMP_RATIO),
                    )
                )
            )
        else:
            continuity = 0.0
        pair_fraction = 1.0 - inferred_rows / float(max(1, len(centers)))
        coverage_score = min(1.0, row_coverage / 0.55)
        quality = (
            0.32 * coverage_score
            + 0.26 * lane_quality
            + 0.18 * line_quality
            + 0.16 * continuity
            + 0.08 * pair_fraction
        )
        if pair_fraction <= 0.05:
            quality = min(quality, SINGLE_BOUNDARY_QUALITY)

        result.valid = row_coverage >= MIN_PAIR_ROW_COVERAGE and quality > 0.18
        result.center_x = center_x
        result.left_x = float(np.median(left_points)) if left_points else None
        result.right_x = float(np.median(right_points)) if right_points else None
        result.lane_width = lane_width
        result.row_coverage = row_coverage
        result.quality = float(np.clip(quality, 0.0, 1.0))
        result.reason = "paired" if pair_fraction > 0.05 else "single_boundary_memory"
        return result

    def _update_history(
        self, detections: List[SegmentLaneDetection], accept: bool
    ) -> None:
        for detection in detections:
            if (
                accept
                and detection.valid
                and detection.center_x is not None
                and detection.lane_width is not None
            ):
                self._previous_center[detection.name] = detection.center_x
                self._previous_width[detection.name] = detection.lane_width
                self._misses[detection.name] = 0
            else:
                self._misses[detection.name] += 1
                if self._misses[detection.name] > TRACK_MEMORY_FRAMES:
                    self._previous_center.pop(detection.name, None)
                    self._previous_width.pop(detection.name, None)

    def detect(self, frame_640x480: np.ndarray) -> DetectionResult:
        height, width = frame_640x480.shape[:2]
        x0 = int(round(width * ROI_LEFT_RATIO))
        x1 = int(round(width * ROI_RIGHT_RATIO))
        y0 = int(round(height * ROI_TOP_RATIO))
        y1 = int(round(height * ROI_BOTTOM_RATIO))
        source_roi = frame_640x480[y0:y1, x0:x1]
        roi_bgr = cv2.resize(
            source_roi,
            (PROCESS_WIDTH, PROCESS_HEIGHT),
            interpolation=cv2.INTER_AREA,
        )
        mask, dynamic_v = self._make_mask(roi_bgr)
        white_ratio = float(cv2.countNonZero(mask)) / float(mask.size)

        far = self._segment_candidate(mask, "far", FAR_RANGE)
        mid = self._segment_candidate(mask, "mid", MID_RANGE)
        near = self._segment_candidate(mask, "near", NEAR_RANGE)
        detections = [near, mid, far]
        valid_detections = [
            item
            for item in detections
            if item.valid and item.center_x is not None and item.lane_width is not None
        ]
        weight_map = {"near": NEAR_WEIGHT, "mid": MID_WEIGHT, "far": FAR_WEIGHT}
        confidence = sum(
            weight_map[item.name] * item.quality for item in valid_detections
        )
        ratio_ok = MIN_WHITE_PIXEL_RATIO <= white_ratio <= MAX_WHITE_PIXEL_RATIO
        valid = (
            ratio_ok
            and len(valid_detections) >= MIN_ACTIVE_SEGMENTS
            and confidence >= MIN_DETECTION_CONFIDENCE
        )
        self._update_history(detections, accept=bool(valid))

        if valid_detections:
            effective_weights = [
                (item, weight_map[item.name] * (0.35 + 0.65 * item.quality))
                for item in valid_detections
            ]
            weight_sum = sum(weight for _item, weight in effective_weights)
            center_x = sum(
                weight * float(item.center_x) for item, weight in effective_weights
            ) / max(1.0e-6, weight_sum)
            lane_width = sum(
                weight * float(item.lane_width) for item, weight in effective_weights
            ) / max(1.0e-6, weight_sum)
            left_values = [item.left_x for item in valid_detections if item.left_x is not None]
            right_values = [item.right_x for item in valid_detections if item.right_x is not None]
            left_x = float(np.median(left_values)) if left_values else center_x - 0.5 * lane_width
            right_x = float(np.median(right_values)) if right_values else center_x + 0.5 * lane_width
            target_center_x = PROCESS_WIDTH / 2.0 + LANE_TARGET_OFFSET_PX
            error = (center_x - target_center_x) / (PROCESS_WIDTH / 2.0)
        else:
            center_x = None
            left_x = None
            right_x = None
            lane_width = None
            error = 0.0

        if not ratio_ok:
            reason = "white_ratio_reject"
        elif len(valid_detections) < MIN_ACTIVE_SEGMENTS:
            reason = "insufficient_double_line"
        elif confidence < MIN_DETECTION_CONFIDENCE:
            reason = "low_confidence"
        else:
            reason = "tracking"

        return DetectionResult(
            valid=bool(valid),
            center_x=center_x,
            left_x=left_x,
            right_x=right_x,
            lane_width=lane_width,
            error=float(np.clip(error, -1.0, 1.0)),
            confidence=float(np.clip(confidence, 0.0, 1.0)),
            near=near,
            mid=mid,
            far=far,
            mask=mask,
            roi_bgr=roi_bgr,
            dynamic_v_threshold=dynamic_v,
            white_pixel_ratio=white_ratio,
            reason=reason,
        )


class DualWhiteLineFollower(Node):
    CSV_FIELDS = [
        "timestamp",
        "frame_id",
        "fps",
        "center_x",
        "left_x",
        "right_x",
        "lane_width",
        "image_center",
        "control_reference_x",
        "lane_target_offset_px",
        "error",
        "d_error",
        "confidence",
        "near_center_x",
        "mid_center_x",
        "far_center_x",
        "near_valid",
        "mid_valid",
        "far_valid",
        "white_pixel_ratio",
        "dynamic_v_threshold",
        "linear_cmd",
        "angular_cmd",
        "line_state",
        "line_reason",
        "near_quality",
        "mid_quality",
        "far_quality",
    ]

    def __init__(self) -> None:
        super().__init__("dual_white_line_follower")
        self.declare_parameter("enable_motion", ENABLE_MOTION)
        self.declare_parameter("show_debug_window", SHOW_DEBUG_WINDOW)
        self.declare_parameter("normal_speed", NORMAL_SPEED)
        self.declare_parameter("min_speed", MIN_SPEED)
        self.declare_parameter("max_angular_speed", MAX_ANGULAR_SPEED)
        self.declare_parameter("curve_slowdown", CURVE_SLOWDOWN)
        self.declare_parameter("lost_search_speed", LOST_SEARCH_SPEED)
        self.declare_parameter("lost_search_angular", LOST_SEARCH_ANGULAR)
        self.enable_motion = bool(self.get_parameter("enable_motion").value)
        self.show_debug_window = bool(
            self.get_parameter("show_debug_window").value
        )
        self.normal_speed = float(self.get_parameter("normal_speed").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_angular_speed = float(
            self.get_parameter("max_angular_speed").value
        )
        self.curve_slowdown = float(
            self.get_parameter("curve_slowdown").value
        )
        self.lost_search_speed = float(
            self.get_parameter("lost_search_speed").value
        )
        self.lost_search_angular = float(
            self.get_parameter("lost_search_angular").value
        )

        self.cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.detector = DualWhiteLineDetector()
        self.capture: Optional[cv2.VideoCapture] = None
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.csv_file = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.stop_requested = False
        self.frame_id = 0
        self.last_valid_time: Optional[float] = None
        self.last_search_direction = 1.0
        self.filtered_error: Optional[float] = None
        self.filtered_d_error = 0.0
        self.error_integral = 0.0
        self.previous_error: Optional[float] = None
        self.previous_control_time: Optional[float] = None
        self.command_linear = 0.0
        self.command_angular = 0.0
        self.fps_ema = 0.0
        self.last_fps_time: Optional[float] = None
        self.last_flush_time = time.monotonic()
        self.last_status_time = 0.0

        OUTPUT_DIRECTORY.mkdir(parents=True, exist_ok=True)
        self.session_stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        basename = f"dual_white_line_{self.session_stamp}"
        self.video_path = OUTPUT_DIRECTORY / f"{basename}.avi"
        self.csv_path = OUTPUT_DIRECTORY / f"{basename}.csv"
        self._open_csv()

        mode = "运动已启用" if self.enable_motion else "仅识别（底盘强制为零）"
        self.get_logger().warn(
            f"红底双白线巡线启动：{mode}；Ctrl-C 或丢线超时会立即停车"
        )
        if self.show_debug_window:
            self.get_logger().info("调试窗口已启用：Q/Esc 可立即停车退出")
        self.get_logger().info(f"同步输出: {self.video_path} / {self.csv_path}")

    def _open_csv(self) -> None:
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.CSV_FIELDS)
        self.csv_writer.writeheader()
        self.csv_file.flush()

    def _open_camera(self) -> None:
        if CAMERA_BACKEND == "v4l2":
            capture = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(CAMERA_ID)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                "无法打开摄像头 "
                f"{CAMERA_DEVICE if CAMERA_BACKEND == 'v4l2' else CAMERA_ID}"
            )
        if CAMERA_BACKEND == "v4l2":
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*CAMERA_PIXEL_FORMAT),
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, IMAGE_WIDTH)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, IMAGE_HEIGHT)
        capture.set(cv2.CAP_PROP_FPS, CAPTURE_FPS)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, CAMERA_BUFFER_SIZE)
        self.capture = capture
        self.get_logger().info(
            "摄像头已打开: "
            f"{int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
            f"{capture.get(cv2.CAP_PROP_FPS):.1f} fps"
        )

    def _open_video(self, frame: np.ndarray) -> None:
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(
            str(self.video_path),
            cv2.VideoWriter_fourcc(*VIDEO_CODEC),
            VIDEO_FPS,
            (width, height),
        )
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(f"无法创建 AVI 录像: {self.video_path}")
        self.video_writer = writer

    def _publish(self, linear: float, angular: float) -> Tuple[float, float]:
        if not self.enable_motion:
            linear = 0.0
            angular = 0.0
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)
        return float(linear), float(angular)

    @staticmethod
    def _slew(current: float, target: float, max_delta: float) -> float:
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def _publish_smooth(
        self, target_linear: float, target_angular: float, dt: float
    ) -> Tuple[float, float]:
        linear = self._slew(
            self.command_linear,
            target_linear,
            MAX_LINEAR_ACCELERATION * dt,
        )
        angular = self._slew(
            self.command_angular,
            target_angular,
            MAX_ANGULAR_ACCELERATION * dt,
        )
        self.command_linear = linear
        self.command_angular = angular
        return self._publish(linear, angular)

    def stop_now(self) -> None:
        zero = Twist()
        self.command_linear = 0.0
        self.command_angular = 0.0
        for _ in range(3):
            self.cmd_pub.publish(zero)

    def _control(
        self, detection: DetectionResult, now: float
    ) -> Tuple[float, float, float, str]:
        if self.previous_control_time is None:
            dt = 1.0 / CONTROL_RATE_HZ
        else:
            dt = float(np.clip(now - self.previous_control_time, 0.01, 0.50))
        self.previous_control_time = now

        if detection.valid:
            measured_error = detection.error
            if self.filtered_error is None:
                self.filtered_error = measured_error
            else:
                self.filtered_error = (
                    ERROR_FILTER_ALPHA * measured_error
                    + (1.0 - ERROR_FILTER_ALPHA) * self.filtered_error
                )
            error = self.filtered_error
            raw_derivative = (
                0.0
                if self.previous_error is None
                else (error - self.previous_error) / dt
            )
            raw_derivative = float(
                np.clip(raw_derivative, -MAX_ERROR_DERIVATIVE, MAX_ERROR_DERIVATIVE)
            )
            self.filtered_d_error = (
                DERIVATIVE_FILTER_ALPHA * raw_derivative
                + (1.0 - DERIVATIVE_FILTER_ALPHA) * self.filtered_d_error
            )
            d_error = self.filtered_d_error
            self.error_integral = float(
                np.clip(
                    self.error_integral + error * dt,
                    -INTEGRAL_LIMIT,
                    INTEGRAL_LIMIT,
                )
            )
            angular = STEERING_SIGN * (
                KP * error + KI * self.error_integral + KD * d_error
            )
            angular = float(
                np.clip(
                    angular,
                    -self.max_angular_speed,
                    self.max_angular_speed,
                )
            )

            magnitude = abs(error)
            normalized_curve = max(
                0.0,
                (magnitude - ERROR_SLOWDOWN_START)
                / max(1.0e-6, 1.0 - ERROR_SLOWDOWN_START),
            )
            speed = self.normal_speed * (
                1.0 - self.curve_slowdown * normalized_curve
            )
            confidence_speed = 0.55 + 0.45 * min(1.0, detection.confidence / 0.80)
            speed *= confidence_speed
            speed = float(np.clip(speed, self.min_speed, self.normal_speed))

            self.previous_error = error
            self.last_valid_time = now
            if abs(measured_error) > 0.035:
                self.last_search_direction = math.copysign(1.0, measured_error)
            linear, angular = self._publish_smooth(speed, angular, dt)
            state = "TRACKING" if self.enable_motion else "DRY_RUN_TRACKING"
            return linear, angular, d_error, state

        self.previous_error = None
        self.filtered_error = None
        self.filtered_d_error = 0.0
        self.error_integral *= 0.85
        if self.last_valid_time is None:
            self.stop_now()
            return 0.0, 0.0, 0.0, "NO_LINE_STOP"

        lost_for = now - self.last_valid_time
        if lost_for >= LOST_STOP_SECONDS:
            self.stop_now()
            return 0.0, 0.0, 0.0, "LOST_TIMEOUT_STOP"

        direction = self.last_search_direction
        angular = STEERING_SIGN * direction * self.lost_search_angular
        linear, angular = self._publish_smooth(
            self.lost_search_speed, angular, dt
        )
        side = "RIGHT" if direction > 0.0 else "LEFT"
        state = f"SEARCH_{side}"
        if not self.enable_motion:
            state = f"DRY_RUN_{state}"
        return linear, angular, 0.0, state

    def _update_fps(self, now: float) -> float:
        if self.last_fps_time is not None:
            instant = 1.0 / max(1.0e-4, now - self.last_fps_time)
            self.fps_ema = instant if self.fps_ema <= 0.0 else 0.15 * instant + 0.85 * self.fps_ema
        self.last_fps_time = now
        return self.fps_ema

    @staticmethod
    def _csv_number(value: Optional[float], digits: int = 6) -> str:
        return "" if value is None else f"{value:.{digits}f}"

    @staticmethod
    def _draw_segment(
        image: np.ndarray,
        detection: SegmentLaneDetection,
        color: Tuple[int, int, int],
    ) -> None:
        cv2.line(image, (0, detection.y0), (PROCESS_WIDTH - 1, detection.y0), color, 1)
        if detection.valid and detection.center_x is not None:
            y = (detection.y0 + detection.y1) // 2
            cv2.circle(image, (int(round(detection.center_x)), y), 5, color, -1)
            if detection.left_x is not None:
                cv2.circle(image, (int(round(detection.left_x)), y), 4, (0, 255, 0), -1)
            if detection.right_x is not None:
                cv2.circle(image, (int(round(detection.right_x)), y), 4, (255, 0, 0), -1)

    def _make_debug(
        self,
        frame: np.ndarray,
        detection: DetectionResult,
        fps: float,
        d_error: float,
        linear: float,
        angular: float,
        state: str,
    ) -> np.ndarray:
        raw = frame.copy()
        roi_y0 = int(round(IMAGE_HEIGHT * ROI_TOP_RATIO))
        roi_y1 = int(round(IMAGE_HEIGHT * ROI_BOTTOM_RATIO)) - 1
        cv2.rectangle(raw, (0, roi_y0), (IMAGE_WIDTH - 1, roi_y1), (255, 180, 0), 2)

        roi_debug = detection.roi_bgr.copy()
        mask_debug = cv2.cvtColor(detection.mask, cv2.COLOR_GRAY2BGR)
        colors = {"near": (0, 255, 0), "mid": (0, 220, 255), "far": (255, 180, 0)}
        for item in (detection.near, detection.mid, detection.far):
            self._draw_segment(roi_debug, item, colors[item.name])
            self._draw_segment(mask_debug, item, colors[item.name])
            if item.valid and item.center_x is not None:
                source_x = int(round(item.center_x * IMAGE_WIDTH / PROCESS_WIDTH))
                source_y = int(
                    round(
                        roi_y0
                        + ((item.y0 + item.y1) * 0.5)
                        * (roi_y1 - roi_y0 + 1)
                        / PROCESS_HEIGHT
                    )
                )
                cv2.circle(raw, (source_x, source_y), 7, colors[item.name], -1)

        target_x = int(round(PROCESS_WIDTH / 2.0 + LANE_TARGET_OFFSET_PX))
        cv2.line(roi_debug, (target_x, 0), (target_x, PROCESS_HEIGHT - 1), (255, 255, 0), 2)
        cv2.line(mask_debug, (target_x, 0), (target_x, PROCESS_HEIGHT - 1), (255, 255, 0), 1)
        raw_target_x = int(round(target_x * IMAGE_WIDTH / PROCESS_WIDTH))
        cv2.line(raw, (raw_target_x, roi_y0), (raw_target_x, roi_y1), (255, 255, 0), 2)

        if detection.center_x is not None:
            cx = int(round(detection.center_x))
            cv2.circle(roi_debug, (cx, PROCESS_HEIGHT // 2), 8, (255, 0, 255), 2)
            if detection.left_x is not None:
                cv2.line(
                    roi_debug,
                    (int(round(detection.left_x)), 0),
                    (int(round(detection.left_x)), PROCESS_HEIGHT - 1),
                    (0, 255, 0),
                    1,
                )
            if detection.right_x is not None:
                cv2.line(
                    roi_debug,
                    (int(round(detection.right_x)), 0),
                    (int(round(detection.right_x)), PROCESS_HEIGHT - 1),
                    (255, 0, 0),
                    1,
                )

        telemetry = np.full((PROCESS_HEIGHT, PROCESS_WIDTH, 3), 22, dtype=np.uint8)
        lines = [
            f"FPS {fps:4.1f}   Vthr {detection.dynamic_v_threshold}",
            f"error {detection.error:+.3f}  d {d_error:+.3f}",
            f"confidence {detection.confidence:.3f}",
            f"lane {self._csv_number(detection.lane_width, 1)} px",
            f"L {self._csv_number(detection.left_x, 1)}  R {self._csv_number(detection.right_x, 1)}",
            f"v {linear:+.3f}   w {angular:+.3f}",
            f"state {state}",
        ]
        for index, line in enumerate(lines):
            cv2.putText(
                telemetry,
                line,
                (8, 18 + 22 * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.47,
                (230, 230, 230) if index != 5 else (0, 220, 255),
                1,
                cv2.LINE_AA,
            )
        return np.hstack((raw, np.vstack((roi_debug, mask_debug, telemetry))))

    def _write_log(
        self,
        detection: DetectionResult,
        fps: float,
        d_error: float,
        linear: float,
        angular: float,
        state: str,
        now: float,
    ) -> None:
        assert self.csv_writer is not None
        self.csv_writer.writerow(
            {
                "timestamp": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "frame_id": self.frame_id,
                "fps": self._csv_number(fps, 3),
                "center_x": self._csv_number(detection.center_x, 3),
                "left_x": self._csv_number(detection.left_x, 3),
                "right_x": self._csv_number(detection.right_x, 3),
                "lane_width": self._csv_number(detection.lane_width, 3),
                "image_center": f"{PROCESS_WIDTH / 2.0:.3f}",
                "control_reference_x": f"{PROCESS_WIDTH / 2.0 + LANE_TARGET_OFFSET_PX:.3f}",
                "lane_target_offset_px": f"{LANE_TARGET_OFFSET_PX:.3f}",
                "error": f"{detection.error:.6f}",
                "d_error": f"{d_error:.6f}",
                "confidence": f"{detection.confidence:.6f}",
                "near_center_x": self._csv_number(detection.near.center_x if detection.near.valid else None, 3),
                "mid_center_x": self._csv_number(detection.mid.center_x if detection.mid.valid else None, 3),
                "far_center_x": self._csv_number(detection.far.center_x if detection.far.valid else None, 3),
                "near_valid": int(detection.near.valid),
                "mid_valid": int(detection.mid.valid),
                "far_valid": int(detection.far.valid),
                "white_pixel_ratio": f"{detection.white_pixel_ratio:.7f}",
                "dynamic_v_threshold": detection.dynamic_v_threshold,
                "linear_cmd": f"{linear:.6f}",
                "angular_cmd": f"{angular:.6f}",
                "line_state": state,
                "line_reason": detection.reason,
                "near_quality": f"{detection.near.quality:.6f}",
                "mid_quality": f"{detection.mid.quality:.6f}",
                "far_quality": f"{detection.far.quality:.6f}",
            }
        )
        if now - self.last_flush_time >= LOG_FLUSH_SECONDS:
            assert self.csv_file is not None
            self.csv_file.flush()
            self.last_flush_time = now

    def run(self) -> None:
        self._open_camera()
        if self.show_debug_window:
            cv2.namedWindow(DEBUG_WINDOW_NAME, cv2.WINDOW_NORMAL)
        next_cycle = time.monotonic()
        consecutive_fail_start: Optional[float] = None
        debug_stride = max(1, int(DEBUG_EVERY_N_FRAMES))

        while rclpy.ok() and not self.stop_requested:
            rclpy.spin_once(self, timeout_sec=0.0)
            assert self.capture is not None
            ok, frame = self.capture.read()
            now = time.monotonic()
            if not ok or frame is None:
                if consecutive_fail_start is None:
                    consecutive_fail_start = now
                if now - consecutive_fail_start >= IMAGE_TIMEOUT_SECONDS:
                    self.get_logger().error("摄像头超时，立即停车")
                    self.stop_now()
                    raise RuntimeError("摄像头连续读取失败")
                time.sleep(0.01)
                continue
            consecutive_fail_start = None

            if frame.shape[:2] != (IMAGE_HEIGHT, IMAGE_WIDTH):
                frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT), interpolation=cv2.INTER_AREA)
            detection = self.detector.detect(frame)
            fps = self._update_fps(now)
            linear, angular, d_error, state = self._control(detection, now)
            self._write_log(detection, fps, d_error, linear, angular, state, now)

            if (
                (self.show_debug_window or RECORD_DEBUG_VIDEO)
                and self.frame_id % debug_stride == 0
            ):
                debug = self._make_debug(frame, detection, fps, d_error, linear, angular, state)
                if DEBUG_OUTPUT_SCALE != 1.0:
                    debug = cv2.resize(
                        debug,
                        None,
                        fx=DEBUG_OUTPUT_SCALE,
                        fy=DEBUG_OUTPUT_SCALE,
                        interpolation=cv2.INTER_AREA,
                    )
                if RECORD_DEBUG_VIDEO:
                    if self.video_writer is None:
                        self._open_video(debug)
                    assert self.video_writer is not None
                    self.video_writer.write(debug)
                if self.show_debug_window:
                    cv2.imshow(DEBUG_WINDOW_NAME, debug)

            if self.show_debug_window:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), ord("Q"), 27):
                    self.get_logger().warn("收到 Q/Esc，立即停车退出")
                    self.stop_now()
                    self.stop_requested = True
                    break

            if now - self.last_status_time >= STATUS_LOG_SECONDS:
                self.get_logger().info(
                    f"{state}: fps={fps:.1f}, error={detection.error:+.3f}, "
                    f"confidence={detection.confidence:.2f}, "
                    f"lane={self._csv_number(detection.lane_width, 1)}, "
                    f"seg={int(detection.near.valid)}"
                    f"{int(detection.mid.valid)}{int(detection.far.valid)}, "
                    f"reason={detection.reason}, v={linear:.2f}, w={angular:.2f}"
                )
                self.last_status_time = now

            self.frame_id += 1
            next_cycle += 1.0 / CONTROL_RATE_HZ
            remaining = next_cycle - time.monotonic()
            if remaining > 0.0:
                time.sleep(remaining)
            elif remaining < -0.5:
                next_cycle = time.monotonic()

    def close(self) -> None:
        self.stop_now()
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        if self.csv_file is not None:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
        cv2.destroyAllWindows()
        self.get_logger().info(f"已停车并保存: {self.video_path} / {self.csv_path}")


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[DualWhiteLineFollower] = None

    def request_stop(_signum=None, _frame=None) -> None:
        if node is not None:
            node.stop_requested = True
            node.stop_now()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = DualWhiteLineFollower()
        node.run()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"双白线巡线程序异常: {exc}")
        else:
            print(f"双白线巡线程序初始化失败: {exc}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
