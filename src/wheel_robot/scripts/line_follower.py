#!/usr/bin/env python3
"""Outdoor red-ground / single-white-line follower for an Orange Pi.

The detector deliberately works on a 320x160 copy of the lower image region.
The 640x480 frame is retained only for display and recording.  This keeps the
hot path small enough for an Orange Pi while preserving useful evidence when a
run needs to be analysed afterwards.
"""

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


# ======================== 默认参数（推荐在 config/line_follower.yaml 调整） ========================
ENABLE_MOTION = True
SHOW_DEBUG_WINDOW = False
# ---------- 最常用：速度参数 ----------
# 单位：线速度 m/s，角速度 rad/s。
NORMAL_SPEED = 0.80              # 直线/小误差时的正常速度
MIN_SPEED = 0.60                 # 大弯自动降速后的最低速度
MAX_ANGULAR_SPEED = 0.60         # 正常巡线最大转向角速度
CURVE_SLOWDOWN = 0.48            # 0~1，越大表示转弯时降速越明显
LOST_SEARCH_SPEED = 0.10         # 短暂丢线时的低速搜索速度
LOST_SEARCH_ANGULAR = 0.20       # 短暂丢线时的搜索角速度

# 摄像头与统一画面。程序无论摄像头实际输出什么尺寸，都会转成 640x480。
CAMERA_BACKEND = "v4l2"          # "v4l2" 或 "auto"
CAMERA_DEVICE = "/dev/video0"
CAMERA_ID = 0
CAMERA_PIXEL_FORMAT = "YUYV"
CAPTURE_FPS = 30.0
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_BUFFER_SIZE = 1
IMAGE_TIMEOUT_SECONDS = 0.35
CONTROL_RATE_HZ = 25.0

# 地面 ROI：从画面高度 45% 到底部，恰好约占下方 55%。
ROI_TOP_RATIO = 0.45
ROI_BOTTOM_RATIO = 1.00
ROI_LEFT_RATIO = 0.00
ROI_RIGHT_RATIO = 1.00
PROCESS_WIDTH = 320
PROCESS_HEIGHT = 160

# 横向跟线偏置（处理图像像素）。负数让白线保持在画面左侧、车体向右移；
# 正数让白线保持在画面右侧、车体向左移。当前“右轮压线”先用 -35。
LINE_TARGET_OFFSET_PX = -35.0

# HSV 白线：低饱和度 + 动态/局部亮度。OpenCV 的 H 范围是 0~180。
WHITE_H_MIN = 0
WHITE_H_MAX = 180
WHITE_S_MAX = 72
DYNAMIC_V_PERCENTILE = 82.0      # 低饱和像素亮度的这个百分位
DYNAMIC_V_OFFSET = 16.0          # 百分位基础上再提高，抑制灰白地面/阴影
DYNAMIC_V_MIN = 115
DYNAMIC_V_MAX = 225
STRONG_WHITE_V_MARGIN = 28       # 极亮像素可绕过局部对比要求
ADAPTIVE_BLOCK_SIZE = 31         # 必须为奇数
ADAPTIVE_C = -5.0                # 负数表示要求比局部均值更亮

# 红底排除（两段 H 覆盖红色跨越 0 度的情况）。
RED_H_LOW_1 = 0
RED_H_HIGH_1 = 13
RED_H_LOW_2 = 167
RED_H_HIGH_2 = 180
RED_S_MIN = 65
RED_V_MIN = 35

# 形态学：按需求固定为 3x3 开运算、5x5 闭运算。
OPEN_KERNEL_SIZE = 3
CLOSE_KERNEL_SIZE = 5
OPEN_ITERATIONS = 1
CLOSE_ITERATIONS = 1

# 三段范围与融合权重（坐标均相对于缩放后的 ROI，0=远端/顶部）。
FAR_RANGE = (0.00, 0.32)
MID_RANGE = (0.32, 0.66)
NEAR_RANGE = (0.66, 1.00)
NEAR_WEIGHT = 0.55
MID_WEIGHT = 0.30
FAR_WEIGHT = 0.15

# 候选线约束。等效线宽 = 连通域面积 / 纵向跨度，曲线也不易误判为宽块。
MIN_COMPONENT_AREA = 20
MAX_COMPONENT_AREA_RATIO = 0.22
MIN_LINE_WIDTH_PX = 2.0
MAX_LINE_WIDTH_PX = 52.0
EXPECTED_LINE_WIDTH_PX = 12.0
MIN_VERTICAL_COVERAGE = 0.24
MIN_SINGLE_RUN_FRACTION = 0.72   # 每个有效行应主要是一条白带，而非反光圆环
MAX_FRAME_JUMP_RATIO = 0.20      # 同一段相邻帧最多跳 PROCESS_WIDTH 的比例
MAX_NEAR_MID_GAP_RATIO = 0.34
MAX_MID_FAR_GAP_RATIO = 0.38
MAX_NEAR_FAR_GAP_RATIO = 0.52
TRACK_MEMORY_FRAMES = 6
MIN_DETECTION_CONFIDENCE = 0.24
SINGLE_SEGMENT_MIN_QUALITY = 0.52
MIN_WHITE_PIXEL_RATIO = 0.0008
MAX_WHITE_PIXEL_RATIO = 0.16

# PD/PID。error>0 表示线在画面右侧；本车右转通常要求 angular.z<0。
KP = 0.68
KI = 0.00
KD = 0.028
STEERING_SIGN = -1.0
INTEGRAL_LIMIT = 0.70
ERROR_FILTER_ALPHA = 0.38
DERIVATIVE_FILTER_ALPHA = 0.25
MAX_ERROR_DERIVATIVE = 2.8
MAX_LINEAR_ACCELERATION = 0.45
MAX_ANGULAR_ACCELERATION = 1.80
ERROR_SLOWDOWN_START = 0.18

# 丢线：短暂丢线按最后方向低速搜；超过此时间立即发布零速度。
LOST_STOP_SECONDS = 0.75         # 建议只在 0.5~1.0 秒内调节

# 单一调试窗、带标记 AVI、同步 CSV。正式巡线建议关闭窗口，避免远程显示拖慢主循环。

DEBUG_WINDOW_NAME = "Outdoor White Line (Q/Esc = STOP)"
OUTPUT_DIRECTORY = Path(__file__).resolve().parents[3] / "recordings"
RECORD_DEBUG_VIDEO = True
DEBUG_EVERY_N_FRAMES = 3
DEBUG_OUTPUT_SCALE = 0.5
VIDEO_CODEC = "MJPG"
VIDEO_FPS = 8.0
LOG_FLUSH_SECONDS = 1.0
STATUS_LOG_SECONDS = 2.0
CMD_VEL_TOPIC = "/cmd_vel"


@dataclass
class SegmentDetection:
    name: str
    y0: int
    y1: int
    valid: bool = False
    x: Optional[float] = None
    y: Optional[float] = None
    area: int = 0
    line_width: float = 0.0
    vertical_coverage: float = 0.0
    single_run_fraction: float = 0.0
    quality: float = 0.0
    reason: str = "none"


@dataclass
class DetectionResult:
    valid: bool
    target_x: Optional[float]
    error: float
    confidence: float
    near: SegmentDetection
    mid: SegmentDetection
    far: SegmentDetection
    mask: np.ndarray
    roi_bgr: np.ndarray
    dynamic_v_threshold: int
    white_pixel_ratio: float
    reason: str


class OutdoorWhiteLineDetector:
    """Low-cost detector for one white line painted on a red outdoor floor."""

    def __init__(self) -> None:
        self._previous_x: Dict[str, float] = {}
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
        not_red = ~(red_1 | red_2)

        hue_ok = (h >= WHITE_H_MIN) & (h <= WHITE_H_MAX)
        white = (
            hue_ok
            & low_saturation
            & absolute_bright
            & (local_bright | strong_bright)
            & not_red
        )
        mask = (white.astype(np.uint8) * 255)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self._open_kernel,
            iterations=OPEN_ITERATIONS,
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self._close_kernel,
            iterations=CLOSE_ITERATIONS,
        )
        return mask, dynamic_v

    def _segment_candidate(
        self, mask: np.ndarray, name: str, bounds: Tuple[float, float]
    ) -> SegmentDetection:
        height, width = mask.shape
        y0 = max(0, min(height - 1, int(round(bounds[0] * height))))
        y1 = max(y0 + 1, min(height, int(round(bounds[1] * height))))
        result = SegmentDetection(name=name, y0=y0, y1=y1)
        band = mask[y0:y1]
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            band, connectivity=8
        )
        previous = self._previous_x.get(name)
        max_jump = width * MAX_FRAME_JUMP_RATIO
        max_area = max(
            MIN_COMPONENT_AREA + 1,
            int(band.size * MAX_COMPONENT_AREA_RATIO),
        )
        Candidate = Tuple[float, int, float, float, float, float, float]
        candidates: List[Candidate] = []

        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            component_height = int(stats[label, cv2.CC_STAT_HEIGHT])
            if area < MIN_COMPONENT_AREA or area > max_area:
                continue
            vertical_coverage = component_height / float(max(1, y1 - y0))
            line_width = area / float(max(1, component_height))
            if not (MIN_LINE_WIDTH_PX <= line_width <= MAX_LINE_WIDTH_PX):
                continue
            if vertical_coverage < MIN_VERTICAL_COVERAGE:
                continue
            component = labels == label
            occupied_rows = component.any(axis=1)
            single_run_rows = 0
            occupied_count = int(np.count_nonzero(occupied_rows))
            for row in component[occupied_rows]:
                padded = np.pad(row.astype(np.int8), (1, 1))
                run_count = int(np.count_nonzero(np.diff(padded) == 1))
                if run_count == 1:
                    single_run_rows += 1
            single_run_fraction = single_run_rows / float(
                max(1, occupied_count)
            )
            if single_run_fraction < MIN_SINGLE_RUN_FRACTION:
                continue
            cx = float(centroids[label, 0])
            cy = y0 + float(centroids[label, 1])
            if previous is not None and abs(cx - previous) > max_jump:
                continue

            width_score = max(
                0.0,
                1.0
                - abs(line_width - EXPECTED_LINE_WIDTH_PX)
                / max(
                    EXPECTED_LINE_WIDTH_PX,
                    MAX_LINE_WIDTH_PX - EXPECTED_LINE_WIDTH_PX,
                ),
            )
            coverage_score = min(1.0, vertical_coverage / 0.72)
            area_score = min(1.0, area / float(max(1, MIN_COMPONENT_AREA * 5)))
            history_score = (
                0.55
                if previous is None
                else max(0.0, 1.0 - abs(cx - previous) / max_jump)
            )
            quality = (
                0.28 * coverage_score
                + 0.23 * width_score
                + 0.17 * area_score
                + 0.17 * history_score
                + 0.15 * single_run_fraction
            )
            candidates.append(
                (
                    quality,
                    area,
                    cx,
                    cy,
                    line_width,
                    vertical_coverage,
                    single_run_fraction,
                )
            )

        if not candidates:
            result.reason = "no_component"
            return result

        quality, area, cx, cy, line_width, coverage, single_run_fraction = max(
            candidates, key=lambda item: item[0]
        )
        result.valid = True
        result.x = cx
        result.y = cy
        result.area = area
        result.line_width = line_width
        result.vertical_coverage = coverage
        result.single_run_fraction = single_run_fraction
        result.quality = float(np.clip(quality, 0.0, 1.0))
        result.reason = "candidate"
        return result

    @staticmethod
    def _invalidate_weaker(
        first: SegmentDetection,
        second: SegmentDetection,
        max_gap: float,
    ) -> None:
        if (
            not first.valid
            or not second.valid
            or first.x is None
            or second.x is None
        ):
            return
        if abs(first.x - second.x) <= max_gap:
            return
        weaker = first if first.quality < second.quality else second
        weaker.valid = False
        weaker.reason = "continuity_reject"

    def _apply_continuity(
        self,
        near: SegmentDetection,
        mid: SegmentDetection,
        far: SegmentDetection,
    ) -> None:
        width = float(PROCESS_WIDTH)
        self._invalidate_weaker(near, mid, width * MAX_NEAR_MID_GAP_RATIO)
        self._invalidate_weaker(mid, far, width * MAX_MID_FAR_GAP_RATIO)
        if not mid.valid:
            self._invalidate_weaker(near, far, width * MAX_NEAR_FAR_GAP_RATIO)

    def _update_history(
        self, detections: List[SegmentDetection], accept: bool
    ) -> None:
        for detection in detections:
            if accept and detection.valid and detection.x is not None:
                self._previous_x[detection.name] = detection.x
                self._misses[detection.name] = 0
            else:
                self._misses[detection.name] += 1
                if self._misses[detection.name] > TRACK_MEMORY_FRAMES:
                    self._previous_x.pop(detection.name, None)

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
        self._apply_continuity(near, mid, far)

        detections = [near, mid, far]
        weight_map = {
            "near": NEAR_WEIGHT,
            "mid": MID_WEIGHT,
            "far": FAR_WEIGHT,
        }
        valid_detections = [
            item for item in detections if item.valid and item.x is not None
        ]
        confidence = sum(
            weight_map[item.name] * item.quality for item in valid_detections
        )
        ratio_ok = (
            MIN_WHITE_PIXEL_RATIO <= white_ratio <= MAX_WHITE_PIXEL_RATIO
        )
        single_segment_ok = any(
            item.name in ("near", "mid")
            and item.quality >= SINGLE_SEGMENT_MIN_QUALITY
            for item in valid_detections
        )
        geometry_ok = len(valid_detections) >= 2 or single_segment_ok
        valid = (
            ratio_ok
            and geometry_ok
            and confidence >= MIN_DETECTION_CONFIDENCE
        )
        self._update_history(detections, accept=valid)

        if valid_detections:
            effective_weights = [
                (
                    item,
                    weight_map[item.name] * (0.35 + 0.65 * item.quality),
                )
                for item in valid_detections
            ]
            weight_sum = sum(weight for _item, weight in effective_weights)
            target_x = sum(
                weight * float(item.x) for item, weight in effective_weights
            ) / max(1.0e-6, weight_sum)
            control_center_x = PROCESS_WIDTH / 2.0 + LINE_TARGET_OFFSET_PX
            error = (target_x - control_center_x) / (PROCESS_WIDTH / 2.0)
        else:
            target_x = None
            error = 0.0

        if not ratio_ok:
            reason = "white_ratio_reject"
        elif not geometry_ok:
            reason = "insufficient_segments"
        elif confidence < MIN_DETECTION_CONFIDENCE:
            reason = "low_confidence"
        else:
            reason = "tracking"

        return DetectionResult(
            valid=valid,
            target_x=target_x,
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


class OutdoorLineFollower(Node):
    CSV_FIELDS = [
        "timestamp",
        "frame_id",
        "fps",
        "target_x",
        "image_center",
        "control_reference_x",
        "line_target_offset_px",
        "error",
        "d_error",
        "confidence",
        "near_x",
        "mid_x",
        "far_x",
        "near_valid",
        "mid_valid",
        "far_valid",
        "white_pixel_ratio",
        "dynamic_v_threshold",
        "linear_cmd",
        "angular_cmd",
        "line_state",
    ]

    def __init__(self) -> None:
        super().__init__("line_follower")
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
        self.detector = OutdoorWhiteLineDetector()
        self.capture: Optional[cv2.VideoCapture] = None
        self.video_writer: Optional[cv2.VideoWriter] = None
        self.csv_file = None
        self.csv_writer: Optional[csv.DictWriter] = None
        self.stop_requested = False
        self.frame_id = 0
        self.last_frame_time = time.monotonic()
        self.last_valid_time: Optional[float] = None
        self.last_valid_error = 0.0
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
        self.session_stamp = (
            datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        )
        basename = f"line_follow_{self.session_stamp}"
        self.video_path = OUTPUT_DIRECTORY / f"{basename}.avi"
        self.csv_path = OUTPUT_DIRECTORY / f"{basename}.csv"
        self._open_csv()

        mode = "运动已启用" if self.enable_motion else "仅识别（底盘强制为零）"
        self.get_logger().warn(
            f"室外红底白线巡线启动：{mode}；Ctrl-C 或丢线超时会立即停车"
        )
        if self.show_debug_window:
            self.get_logger().info("调试窗口已启用：Q/Esc 可立即停车退出")
        self.get_logger().info(f"同步输出: {self.video_path} / {self.csv_path}")

    def _open_csv(self) -> None:
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.csv_writer = csv.DictWriter(
            self.csv_file, fieldnames=self.CSV_FIELDS
        )
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
        self,
        target_linear: float,
        target_angular: float,
        dt: float,
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
        """Bypass controller state and publish explicit zero commands."""
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
        d_error = 0.0

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
                np.clip(
                    raw_derivative,
                    -MAX_ERROR_DERIVATIVE,
                    MAX_ERROR_DERIVATIVE,
                )
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
            speed *= 0.88 + 0.12 * detection.confidence
            speed = float(np.clip(speed, self.min_speed, self.normal_speed))

            self.previous_error = error
            self.last_valid_error = error
            self.last_valid_time = now
            if abs(measured_error) > 0.035:
                self.last_search_direction = math.copysign(1.0, measured_error)
            state = "TRACKING"
            linear, angular = self._publish_smooth(speed, angular, dt)
            if not self.enable_motion:
                state = "DRY_RUN_TRACKING"
            return linear, angular, d_error, state

        # Avoid integrating stale errors or a derivative kick on reacquisition.
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
            if self.fps_ema <= 0.0:
                self.fps_ema = instant
            else:
                self.fps_ema = 0.15 * instant + 0.85 * self.fps_ema
        self.last_fps_time = now
        return self.fps_ema

    @staticmethod
    def _draw_segment(
        image: np.ndarray,
        detection: SegmentDetection,
        color: Tuple[int, int, int],
    ) -> None:
        cv2.line(
            image,
            (0, detection.y0),
            (PROCESS_WIDTH - 1, detection.y0),
            color,
            1,
        )
        if (
            detection.valid
            and detection.x is not None
            and detection.y is not None
        ):
            point = (int(round(detection.x)), int(round(detection.y)))
            cv2.circle(image, point, 6, color, -1, cv2.LINE_AA)
            cv2.putText(
                image,
                detection.name.upper(),
                (max(2, point[0] - 22), max(13, point[1] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
                cv2.LINE_AA,
            )

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
        cv2.rectangle(
            raw,
            (0, roi_y0),
            (IMAGE_WIDTH - 1, roi_y1),
            (255, 180, 0),
            2,
        )
        cv2.putText(
            raw,
            "RAW 640x480",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        roi_debug = detection.roi_bgr.copy()
        cv2.putText(
            roi_debug,
            "ROI 320x160",
            (7, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        colors = {
            "near": (0, 255, 0),
            "mid": (0, 220, 255),
            "far": (255, 180, 0),
        }
        for item in (detection.near, detection.mid, detection.far):
            self._draw_segment(roi_debug, item, colors[item.name])
            if item.valid and item.x is not None and item.y is not None:
                source_x = int(round(item.x * IMAGE_WIDTH / PROCESS_WIDTH))
                source_y = int(
                    round(
                        roi_y0
                        + item.y
                        * (roi_y1 - roi_y0 + 1)
                        / PROCESS_HEIGHT
                    )
                )
                cv2.circle(
                    raw,
                    (source_x, source_y),
                    8,
                    colors[item.name],
                    -1,
                    cv2.LINE_AA,
                )
        if detection.target_x is not None:
            tx = int(round(detection.target_x))
            cv2.circle(
                roi_debug,
                (tx, PROCESS_HEIGHT // 2),
                8,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
            source_tx = int(
                round(detection.target_x * IMAGE_WIDTH / PROCESS_WIDTH)
            )
            source_ty = roi_y0 + (roi_y1 - roi_y0) // 2
            cv2.circle(
                raw,
                (source_tx, source_ty),
                10,
                (255, 0, 255),
                3,
                cv2.LINE_AA,
            )
        cv2.line(
            roi_debug,
            (PROCESS_WIDTH // 2, 0),
            (PROCESS_WIDTH // 2, PROCESS_HEIGHT - 1),
            (255, 255, 255),
            1,
        )
        control_center_x = int(
            round(PROCESS_WIDTH / 2.0 + LINE_TARGET_OFFSET_PX)
        )
        raw_control_x = int(
            round(control_center_x * IMAGE_WIDTH / PROCESS_WIDTH)
        )
        cv2.line(
            raw,
            (raw_control_x, roi_y0),
            (raw_control_x, roi_y1),
            (255, 255, 0),
            2,
        )
        cv2.line(
            roi_debug,
            (control_center_x, 0),
            (control_center_x, PROCESS_HEIGHT - 1),
            (255, 255, 0),
            2,
        )

        mask_debug = cv2.cvtColor(detection.mask, cv2.COLOR_GRAY2BGR)
        for item in (detection.near, detection.mid, detection.far):
            self._draw_segment(mask_debug, item, colors[item.name])
        cv2.putText(
            mask_debug,
            "MASK",
            (7, 16),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (0, 0, 255),
            1,
            cv2.LINE_AA,
        )

        telemetry = np.full(
            (PROCESS_HEIGHT, PROCESS_WIDTH, 3), 22, dtype=np.uint8
        )
        lines = [
            f"FPS {fps:4.1f}   Vthr {detection.dynamic_v_threshold}",
            f"error {detection.error:+.3f}  d {d_error:+.3f}",
            f"confidence {detection.confidence:.3f}",
            f"white ratio {detection.white_pixel_ratio:.4f}",
            f"line reference x {control_center_x}",
            f"v {linear:+.3f}   w {angular:+.3f}",
            f"state {state}",
        ]
        for index, line in enumerate(lines):
            text_y = 18 + 22 * index
            cv2.putText(
                telemetry,
                line,
                (8, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.47,
                (230, 230, 230) if index != 5 else (0, 220, 255),
                1,
                cv2.LINE_AA,
            )

        return np.hstack((raw, np.vstack((roi_debug, mask_debug, telemetry))))

    @staticmethod
    def _csv_number(value: Optional[float], digits: int = 6):
        return "" if value is None else f"{value:.{digits}f}"

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
                "timestamp": datetime.now()
                .astimezone()
                .isoformat(timespec="milliseconds"),
                "frame_id": self.frame_id,
                "fps": self._csv_number(fps, 3),
                "target_x": self._csv_number(detection.target_x, 3),
                "image_center": f"{PROCESS_WIDTH / 2.0:.3f}",
                "control_reference_x": (
                    f"{PROCESS_WIDTH / 2.0 + LINE_TARGET_OFFSET_PX:.3f}"
                ),
                "line_target_offset_px": f"{LINE_TARGET_OFFSET_PX:.3f}",
                "error": f"{detection.error:.6f}",
                "d_error": f"{d_error:.6f}",
                "confidence": f"{detection.confidence:.6f}",
                "near_x": self._csv_number(
                    detection.near.x if detection.near.valid else None, 3
                ),
                "mid_x": self._csv_number(
                    detection.mid.x if detection.mid.valid else None, 3
                ),
                "far_x": self._csv_number(
                    detection.far.x if detection.far.valid else None, 3
                ),
                "near_valid": int(detection.near.valid),
                "mid_valid": int(detection.mid.valid),
                "far_valid": int(detection.far.valid),
                "white_pixel_ratio": f"{detection.white_pixel_ratio:.7f}",
                "dynamic_v_threshold": detection.dynamic_v_threshold,
                "linear_cmd": f"{linear:.6f}",
                "angular_cmd": f"{angular:.6f}",
                "line_state": state,
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
                frame = cv2.resize(
                    frame,
                    (IMAGE_WIDTH, IMAGE_HEIGHT),
                    interpolation=cv2.INTER_AREA,
                )
            detection = self.detector.detect(frame)
            fps = self._update_fps(now)
            linear, angular, d_error, state = self._control(detection, now)
            self._write_log(
                detection, fps, d_error, linear, angular, state, now
            )

            make_debug = (
                (self.show_debug_window or RECORD_DEBUG_VIDEO)
                and self.frame_id % debug_stride == 0
            )
            if make_debug:
                debug = self._make_debug(
                    frame, detection, fps, d_error, linear, angular, state
                )
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
                    f"v={linear:.2f}, w={angular:.2f}"
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
    node: Optional[OutdoorLineFollower] = None

    def request_stop(_signum=None, _frame=None) -> None:
        if node is not None:
            node.stop_requested = True
            node.stop_now()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = OutdoorLineFollower()
        node.run()
    except Exception as exc:
        # Safety boundary: every failure path stops the chassis.
        if node is not None:
            node.get_logger().error(f"巡线程序异常: {exc}")
        else:
            print(f"巡线程序初始化失败: {exc}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
