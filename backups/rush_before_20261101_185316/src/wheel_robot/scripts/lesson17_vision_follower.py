#!/usr/bin/env python3
"""Lesson 17: ROS 2 person/red-cone/blue-cone visual follower.

Unlike the early teaching prototype, this node never opens ``/dev/video0``.
It consumes the Astra ROS image topic, publishes ``/cmd_vel``, and exposes
ROS topics/services for mode selection and safe enable/disable control.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
import threading
import time
from typing import Dict, Optional, Sequence, Tuple

import cv2
import numpy as np

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool


TARGET_PERSON = "person"
TARGET_RED = "red_cone"
TARGET_BLUE = "blue_cone"
TARGET_AUTO = "auto"
VALID_MODES = (TARGET_PERSON, TARGET_RED, TARGET_BLUE, TARGET_AUTO)
REMOTE_DIAGNOSTIC_FAULT_MASK = (1 << 7) | (1 << 8)


def clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))


def evaluate_diagnostics(
    message: DiagnosticArray, ignore_remote_errors: bool
) -> Tuple[Sequence[str], bool]:
    """Return blocking faults while optionally accepting remote-free ROS control."""
    errors = []
    remote_ignored = False
    for status in message.status:
        raw_level = status.level
        if isinstance(raw_level, (bytes, bytearray)):
            level = raw_level[0] if raw_level else 0
        else:
            level = int(raw_level)
        if level < 2:
            continue

        if ignore_remote_errors and status.name == "wheel_robot/remote":
            remote_ignored = True
            continue

        if ignore_remote_errors and status.name == "wheel_robot/self_test":
            values = {item.key: item.value for item in status.values}
            try:
                fault_bits = int(values.get("fault_bits", ""), 0)
            except ValueError:
                fault_bits = 0
            if (
                fault_bits != 0
                and fault_bits & ~REMOTE_DIAGNOSTIC_FAULT_MASK == 0
            ):
                remote_ignored = True
                continue

        errors.append(f"{status.name}:{status.message}")
    return errors, remote_ignored


def retain_recent_observation(
    observation: Optional["TargetObservation"],
    previous: Optional["TargetObservation"],
    now: float,
    timeout: float,
) -> Optional["TargetObservation"]:
    """Bridge brief one-frame HSV misses without inventing a fresh detection."""
    if observation is not None:
        return observation
    if previous is not None and now - previous.detected_at <= timeout:
        return previous
    return None


def smooth_observation(
    previous: Optional["TargetObservation"],
    current: "TargetObservation",
    alpha: float,
) -> "TargetObservation":
    """Low-pass bounding boxes and control errors while keeping a fresh timestamp."""
    if previous is None or previous.label != current.label:
        return current
    alpha = clamp(alpha, 0.0, 1.0)
    old_weight = 1.0 - alpha
    bbox = tuple(
        int(round(old_weight * old + alpha * new))
        for old, new in zip(previous.bbox, current.bbox)
    )
    return TargetObservation(
        label=current.label,
        bbox=bbox,
        confidence=current.confidence,
        center_error=old_weight * previous.center_error + alpha * current.center_error,
        height_ratio=old_weight * previous.height_ratio + alpha * current.height_ratio,
        detected_at=current.detected_at,
        area_ratio=old_weight * previous.area_ratio + alpha * current.area_ratio,
    )


def follow_command(
    center_error: float,
    height_ratio: float,
    target_height_ratio: float,
    center_deadband: float,
    distance_deadband: float,
    forward_center_limit: float,
    linear_kp: float,
    angular_kp: float,
    angular_sign: float,
    minimum_linear_speed: float,
    minimum_angular_speed: float,
    max_linear_speed: float,
    max_reverse_speed: float,
    max_angular_speed: float,
    allow_reverse: bool,
) -> Tuple[float, float, str]:
    """Convert a selected image target into a bounded velocity command."""
    center_error = clamp(center_error, -1.0, 1.0)
    angular = 0.0
    if abs(center_error) > center_deadband:
        angular = angular_sign * angular_kp * center_error
    angular = clamp(angular, -max_angular_speed, max_angular_speed)
    if 0.0 < abs(angular) < minimum_angular_speed:
        angular = math.copysign(minimum_angular_speed, angular)

    if abs(center_error) > forward_center_limit:
        return 0.0, angular, "ALIGN_TARGET"

    distance_error = target_height_ratio - height_ratio
    linear = 0.0
    if abs(distance_error) > distance_deadband:
        linear = linear_kp * distance_error

    if linear < 0.0:
        if not allow_reverse:
            linear = 0.0
        else:
            linear = max(linear, -max_reverse_speed)
    linear = clamp(linear, -max_reverse_speed, max_linear_speed)

    if linear > 0.0 and forward_center_limit > center_deadband:
        usable = forward_center_limit - center_deadband
        scale = 1.0 - max(0.0, abs(center_error) - center_deadband) / usable
        linear *= clamp(scale, 0.25, 1.0)
    if 0.0 < linear < minimum_linear_speed:
        linear = minimum_linear_speed
    linear = clamp(linear, -max_reverse_speed, max_linear_speed)
    return linear, angular, "FOLLOW_TARGET"


def cone_follow_command(
    center_error: float,
    area_ratio: float,
    center_deadband: float,
    slow_area_ratio: float,
    stop_area_ratio: float,
    angular_kp: float,
    angular_sign: float,
    minimum_linear_speed: float,
    max_linear_speed: float,
    max_angular_speed: float,
) -> Tuple[float, float, str]:
    """Reference color-follower control: steer by centroid, drive by blob area."""
    center_error = clamp(center_error, -1.0, 1.0)
    if abs(center_error) < center_deadband:
        center_error = 0.0
    angular = clamp(
        angular_sign * angular_kp * center_error,
        -max_angular_speed,
        max_angular_speed,
    )

    if area_ratio >= stop_area_ratio:
        linear = 0.0
        state = "CONE_DISTANCE_REACHED"
    elif area_ratio <= slow_area_ratio:
        linear = max_linear_speed
        state = "FOLLOW_TARGET"
    else:
        span = stop_area_ratio - slow_area_ratio
        remaining = (stop_area_ratio - area_ratio) / span
        linear = minimum_linear_speed + remaining * (
            max_linear_speed - minimum_linear_speed
        )
        state = "FOLLOW_TARGET"
    return clamp(linear, 0.0, max_linear_speed), angular, state


@dataclass(frozen=True)
class TargetObservation:
    label: str
    bbox: Tuple[int, int, int, int]
    confidence: float
    center_error: float
    height_ratio: float
    detected_at: float
    area_ratio: float = 0.0


class PersonDetector:
    PERSON_CLASS_ID = 15

    def __init__(
        self,
        prototxt: Path,
        model: Path,
        confidence: float,
        tracking_confidence: float,
        nms_threshold: float,
        minimum_area_ratio: float,
        roi_expansion: float,
    ) -> None:
        self.confidence = confidence
        self.tracking_confidence = tracking_confidence
        self.nms_threshold = nms_threshold
        self.minimum_area_ratio = minimum_area_ratio
        self.roi_expansion = roi_expansion
        self.net = cv2.dnn.readNetFromCaffe(str(prototxt), str(model))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    @staticmethod
    def detection_roi(
        previous: Optional[TargetObservation],
        frame_width: int,
        frame_height: int,
        expansion: float,
    ) -> Tuple[int, int, int, int]:
        """Use a square person-centred ROI after acquisition, full frame otherwise."""
        if previous is None:
            return 0, 0, frame_width, frame_height
        x, y, width, height = previous.bbox
        side = max(width, height) * max(1.0, expansion)
        side = max(side, min(frame_width, frame_height) * 0.45)
        side = int(min(side, frame_width, frame_height))
        center_x = x + 0.5 * width
        center_y = y + 0.5 * height
        roi_x = int(clamp(center_x - 0.5 * side, 0, frame_width - side))
        roi_y = int(clamp(center_y - 0.5 * side, 0, frame_height - side))
        return roi_x, roi_y, side, side

    @staticmethod
    def select_target(
        candidates: Sequence[Tuple[Tuple[int, int, int, int], float]],
        previous: Optional[TargetObservation],
        frame_width: int,
        frame_height: int,
    ) -> Optional[Tuple[Tuple[int, int, int, int], float]]:
        if not candidates:
            return None
        if previous is None:
            return max(candidates, key=lambda item: item[0][2] * item[0][3])

        px, py, pw, ph = previous.bbox
        previous_cx = px + 0.5 * pw
        previous_cy = py + 0.5 * ph
        previous_ratio = ph / max(1.0, float(frame_height))

        def cost(item: Tuple[Tuple[int, int, int, int], float]) -> float:
            (x, y, width, height), confidence = item
            dx = (x + 0.5 * width - previous_cx) / max(1.0, frame_width)
            dy = (y + 0.5 * height - previous_cy) / max(1.0, frame_height)
            size_delta = abs(height / max(1.0, frame_height) - previous_ratio)
            return math.hypot(dx, dy) + 0.55 * size_delta - 0.10 * confidence

        selected = min(candidates, key=cost)
        return selected if cost(selected) <= 0.42 else None

    def detect(
        self,
        frame: np.ndarray,
        previous: Optional[TargetObservation],
        now: float,
    ) -> Optional[TargetObservation]:
        frame_height, frame_width = frame.shape[:2]
        roi_x, roi_y, roi_width, roi_height = self.detection_roi(
            previous, frame_width, frame_height, self.roi_expansion
        )
        inference_frame = frame[
            roi_y : roi_y + roi_height, roi_x : roi_x + roi_width
        ]
        blob = cv2.dnn.blobFromImage(
            inference_frame,
            scalefactor=0.007843,
            size=(300, 300),
            mean=(127.5, 127.5, 127.5),
            swapRB=False,
            crop=False,
        )
        self.net.setInput(blob)
        detections = self.net.forward()
        boxes = []
        scores = []
        confidence_threshold = (
            self.tracking_confidence if previous is not None else self.confidence
        )
        minimum_area = frame_width * frame_height * self.minimum_area_ratio
        for index in range(detections.shape[2]):
            class_id = int(detections[0, 0, index, 1])
            confidence = float(detections[0, 0, index, 2])
            if class_id != self.PERSON_CLASS_ID or confidence < confidence_threshold:
                continue
            x1 = roi_x + int(detections[0, 0, index, 3] * roi_width)
            y1 = roi_y + int(detections[0, 0, index, 4] * roi_height)
            x2 = roi_x + int(detections[0, 0, index, 5] * roi_width)
            y2 = roi_y + int(detections[0, 0, index, 6] * roi_height)
            x1 = int(clamp(x1, 0, frame_width - 1))
            y1 = int(clamp(y1, 0, frame_height - 1))
            x2 = int(clamp(x2, 0, frame_width - 1))
            y2 = int(clamp(y2, 0, frame_height - 1))
            width, height = x2 - x1, y2 - y1
            if width <= 0 or height <= 0 or width * height < minimum_area:
                continue
            boxes.append([x1, y1, width, height])
            scores.append(confidence)

        if not boxes:
            return None
        indices = cv2.dnn.NMSBoxes(
            boxes, scores, confidence_threshold, self.nms_threshold
        )
        candidates = []
        for raw_index in np.asarray(indices).reshape(-1):
            index = int(raw_index)
            candidates.append((tuple(boxes[index]), float(scores[index])))
        selected = self.select_target(
            candidates, previous, frame_width, frame_height
        )
        if selected is None:
            return None
        bbox, confidence = selected
        return make_observation(TARGET_PERSON, bbox, confidence, frame.shape, now)


class ConeDetector:
    """Track the largest valid blob in an HSV color mask."""

    def __init__(
        self,
        label: str,
        ranges: Sequence[Tuple[Sequence[int], Sequence[int]]],
        minimum_area: float,
        morph_kernel: int,
    ) -> None:
        self.label = label
        self.ranges = [
            (np.asarray(lower, dtype=np.uint8), np.asarray(upper, dtype=np.uint8))
            for lower, upper in ranges
        ]
        self.minimum_area = minimum_area
        self.morph_element = cv2.getStructuringElement(
            cv2.MORPH_RECT, (morph_kernel, morph_kernel)
        )

    def create_mask(self, hsv: np.ndarray) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for lower, upper in self.ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.morph_element)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.morph_element)

    def detect(
        self,
        hsv: np.ndarray,
        previous: Optional[TargetObservation],
        now: float,
    ) -> Optional[TargetObservation]:
        mask = self.create_mask(hsv)
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None
        # Deliberately ignore history and shape: follow the largest white blob in
        # the binary mask exactly like the proven standalone reference program.
        del previous
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.minimum_area:
            return None
        moments = cv2.moments(contour)
        if moments["m00"] == 0.0:
            return None
        center_x = float(moments["m10"] / moments["m00"])
        bbox = cv2.boundingRect(contour)
        frame_area = float(mask.shape[0] * mask.shape[1])
        return make_observation(
            self.label,
            bbox,
            1.0,
            hsv.shape,
            now,
            center_x=center_x,
            area_ratio=area / max(1.0, frame_area),
        )


def make_observation(
    label: str,
    bbox: Tuple[int, int, int, int],
    confidence: float,
    frame_shape,
    now: float,
    center_x: Optional[float] = None,
    area_ratio: Optional[float] = None,
) -> TargetObservation:
    frame_height, frame_width = frame_shape[:2]
    x, y, width, height = bbox
    del y
    if center_x is None:
        center_x = x + 0.5 * width
    if area_ratio is None:
        area_ratio = (width * height) / max(1.0, float(frame_height * frame_width))
    center_error = (center_x - 0.5 * frame_width) / max(1.0, 0.5 * frame_width)
    return TargetObservation(
        label=label,
        bbox=bbox,
        confidence=float(confidence),
        center_error=clamp(center_error, -1.0, 1.0),
        height_ratio=height / max(1.0, float(frame_height)),
        detected_at=now,
        area_ratio=clamp(area_ratio, 0.0, 1.0),
    )


class Ros2MultiTargetFollower(Node):
    def __init__(self) -> None:
        super().__init__("lesson17_vision_follower")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()
        cv2.setNumThreads(self.opencv_num_threads)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self.status_pub = self.create_publisher(String, "/lesson17/status", 10)
        self.debug_image_pub = self.create_publisher(
            Image, "/lesson17/debug_image", qos_profile_sensor_data
        )
        self.cone_mask_pub = self.create_publisher(
            Image, "/lesson17/cone_mask", qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            DiagnosticArray, self.diagnostics_topic, self._diagnostics_callback, 10
        )
        self.create_subscription(
            String, "/lesson17/target_mode", self._mode_callback, 10
        )
        self.enable_service = self.create_service(
            SetBool, "/lesson17/enable", self._enable_callback
        )

        self._lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_time = 0.0
        self._frame_sequence = 0
        self._processed_sequence = -1
        self._detections: Dict[str, Optional[TargetObservation]] = {
            TARGET_PERSON: None,
            TARGET_RED: None,
            TARGET_BLUE: None,
        }
        self._confirm_counts = {label: 0 for label in self._detections}
        self._inference_ms = 0.0
        self._diagnostic_time = 0.0
        self._diagnostic_ok = False
        self._diagnostic_text = "WAIT"

        self._selected_label: Optional[str] = None
        self._was_tracking = False
        self._acquired_time = 0.0
        self.state = "IDLE" if not self.enable_motion else "WAIT_SENSORS"
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._cone_filtered_angular = 0.0
        self._last_command_time = time.monotonic()
        self._last_status_time = 0.0
        self._closing = False
        self._recording_requested = bool(self.record_video)
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None
        self._recording_started_time = 0.0
        self._next_recording_frame_time = 0.0

        self.person_detector: Optional[PersonDetector] = None
        try:
            self.person_detector = PersonDetector(
                self.prototxt_path,
                self.model_path,
                self.person_confidence,
                self.person_tracking_confidence,
                self.person_nms_threshold,
                self.person_min_area_ratio,
                self.person_roi_expansion,
            )
            self.get_logger().info("MobileNet-SSD person detector loaded")
        except Exception as error:
            self.get_logger().error(f"人体模型加载失败: {error}")

        common_cone_args = dict(
            minimum_area=self.cone_minimum_area,
            morph_kernel=self.cone_morph_kernel,
        )
        self.red_detector = ConeDetector(
            TARGET_RED,
            [
                (self.red1_lower, self.red1_upper),
                (self.red2_lower, self.red2_upper),
            ],
            **common_cone_args,
        )
        self.blue_detector = ConeDetector(
            TARGET_BLUE,
            [(self.blue_lower, self.blue_upper)],
            **common_cone_args,
        )

        self._stop_worker = threading.Event()
        self._worker = threading.Thread(
            target=self._vision_loop, name="lesson17_vision", daemon=True
        )
        self._worker.start()
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.display_timer = self.create_timer(1.0 / self.display_rate, self._display_loop)
        if self.publish_posture:
            self.posture_timer = self.create_timer(0.1, self._publish_posture)

        self._hard_stop("startup")
        self.get_logger().info(
            "第17课 ROS2 多目标跟随启动：P人体，R红锥桶，B蓝锥桶，A自动"
        )
        self.get_logger().info("S启动，Space急停，V录像，M二值图，Q停车退出")

    def _declare_parameters(self) -> None:
        defaults = {
            "enable_motion": False,
            "target_mode": TARGET_PERSON,
            "image_topic": "/camera/color/image_raw",
            "diagnostics_topic": "/robot_diagnostics",
            "control_rate": 20.0,
            "display_rate": 30.0,
            "show_debug": True,
            "debug_window_name": "Lesson 17 - ROS2 Multi Target Follower",
            "show_binary_mask": True,
            "camera_timeout": 0.60,
            "diagnostic_timeout": 1.00,
            "require_diagnostics": True,
            "ignore_remote_diagnostics": True,
            "model_directory": "",
            "opencv_num_threads": 2,
            "person_confidence": 0.40,
            "person_tracking_confidence": 0.25,
            "person_nms_threshold": 0.35,
            "person_min_area_ratio": 0.004,
            "person_roi_expansion": 1.50,
            "person_box_smoothing": 0.35,
            "person_detect_interval": 2,
            "person_confirm_frames": 2,
            "cone_confirm_frames": 1,
            "person_lost_timeout": 0.70,
            "cone_lost_timeout": 0.25,
            "target_acquire_hold_seconds": 0.10,
            "person_target_height_ratio": 0.50,
            "person_max_linear_speed": 0.12,
            "cone_max_linear_speed": 0.08,
            "person_min_linear_speed": 0.06,
            "cone_min_linear_speed": 0.06,
            "minimum_angular_speed": 0.18,
            "distance_deadband": 0.045,
            "center_deadband": 0.12,
            "forward_center_limit": 0.32,
            "linear_kp": 0.55,
            "angular_kp": 0.55,
            "angular_sign": -1.0,
            "max_reverse_speed": 0.04,
            "max_angular_speed": 0.40,
            "allow_reverse": False,
            "max_linear_acceleration": 0.25,
            "max_angular_acceleration": 0.70,
            "cone_center_deadband": 0.05,
            "cone_slow_area_ratio": 0.025,
            "cone_stop_area_ratio": 0.10,
            "cone_angular_kp": 0.45,
            "cone_max_angular_speed": 0.40,
            "cone_steering_smoothing": 0.35,
            "red1_lower": [0, 100, 70],
            "red1_upper": [10, 255, 255],
            "red2_lower": [170, 100, 70],
            "red2_upper": [179, 255, 255],
            "blue_lower": [78, 35, 25],
            "blue_upper": [140, 255, 255],
            "cone_minimum_area": 40.0,
            "cone_blur_kernel": 5,
            "cone_morph_kernel": 5,
            "publish_posture": True,
            "default_height": 0.25,
            "default_roll": 0.0,
            "default_pitch": 0.0,
            "default_slide": 0.0,
            "record_video": False,
            "record_directory": "",
            "record_fps": 15.0,
            "record_codec": "MJPG",
            "record_max_seconds": 300.0,
            "status_period": 0.50,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        names = [parameter.name for parameter in self._parameters.values()]
        for name in names:
            if name == "model_directory":
                continue
            setattr(self, name, self.get_parameter(name).value)

        model_directory = str(self.get_parameter("model_directory").value).strip()
        model_root = (
            Path(model_directory)
            if model_directory
            else Path(__file__).resolve().parent / "models"
        )
        self.prototxt_path = model_root / "deploy.prototxt"
        self.model_path = model_root / "mobilenet_iter_73000.caffemodel"
        configured_record_directory = str(self.record_directory).strip()
        self.record_directory = (
            Path(configured_record_directory).expanduser()
            if configured_record_directory
            else Path.home() / "wheel_robot" / "recordings" / "lesson17"
        )
        self.record_codec = str(self.record_codec).strip().upper()

        for name in (
            "red1_lower",
            "red1_upper",
            "red2_lower",
            "red2_upper",
            "blue_lower",
            "blue_upper",
        ):
            setattr(self, name, [int(value) for value in getattr(self, name)])
        for name in (
            "person_detect_interval",
            "person_confirm_frames",
            "cone_confirm_frames",
            "cone_blur_kernel",
            "cone_morph_kernel",
            "opencv_num_threads",
        ):
            setattr(self, name, int(getattr(self, name)))
        self.target_mode = str(self.target_mode).strip().lower()

    def _validate_parameters(self) -> None:
        if self.target_mode not in VALID_MODES:
            raise ValueError(f"target_mode 必须是 {VALID_MODES}")
        if self.control_rate <= 0.0 or self.display_rate <= 0.0:
            raise ValueError("control_rate/display_rate 必须为正数")
        if self.opencv_num_threads < 1:
            raise ValueError("opencv_num_threads 必须 >= 1")
        if not 0.0 < self.person_tracking_confidence <= self.person_confidence <= 1.0:
            raise ValueError(
                "person_tracking_confidence 必须大于 0 且不高于 person_confidence"
            )
        if self.person_roi_expansion < 1.0:
            raise ValueError("person_roi_expansion 必须 >= 1.0")
        for name in ("person_box_smoothing", "cone_steering_smoothing"):
            if not 0.0 < getattr(self, name) <= 1.0:
                raise ValueError(f"{name} 必须在 (0, 1] 范围内")
        if not 0.0 <= self.cone_center_deadband < 1.0:
            raise ValueError("cone_center_deadband 必须在 [0, 1) 范围内")
        if not 0.0 <= self.cone_slow_area_ratio < self.cone_stop_area_ratio < 1.0:
            raise ValueError("锥桶面积比例必须满足 0 <= slow < stop < 1")
        if self.cone_angular_kp < 0.0 or self.cone_max_angular_speed <= 0.0:
            raise ValueError("锥桶转向增益和最大角速度参数无效")
        if self.record_fps <= 0.0:
            raise ValueError("record_fps 必须为正数")
        if self.record_max_seconds <= 0.0:
            raise ValueError("record_max_seconds 必须为正数")
        if len(self.record_codec) != 4:
            raise ValueError("record_codec 必须是4个字符，例如 MJPG")
        if self.person_detect_interval < 1:
            raise ValueError("person_detect_interval 必须 >= 1")
        if self.person_confirm_frames < 1 or self.cone_confirm_frames < 1:
            raise ValueError("目标确认帧数必须 >= 1")
        if self.person_lost_timeout <= 0.0 or self.cone_lost_timeout <= 0.0:
            raise ValueError("person_lost_timeout/cone_lost_timeout 必须为正数")
        if not 0.0 <= self.person_min_linear_speed <= self.person_max_linear_speed:
            raise ValueError(
                "person_min_linear_speed 必须在 0 和 person_max_linear_speed 之间"
            )
        if not 0.0 <= self.cone_min_linear_speed <= self.cone_max_linear_speed:
            raise ValueError(
                "cone_min_linear_speed 必须在 0 和 cone_max_linear_speed 之间"
            )
        if not 0.0 <= self.minimum_angular_speed <= self.max_angular_speed:
            raise ValueError(
                "minimum_angular_speed 必须在 0 和 max_angular_speed 之间"
            )
        for name in ("cone_blur_kernel", "cone_morph_kernel"):
            value = getattr(self, name)
            if value < 1 or value % 2 == 0:
                raise ValueError(f"{name} 必须是正奇数")

    @staticmethod
    def image_to_bgr(message: Image) -> np.ndarray:
        if message.encoding not in ("bgr8", "rgb8", "mono8"):
            raise ValueError(f"不支持的图像编码: {message.encoding}")
        channels = 1 if message.encoding == "mono8" else 3
        valid_step = int(message.width) * channels
        if int(message.step) < valid_step:
            raise ValueError("图像 step 无效")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required = int(message.step) * int(message.height)
        if raw.size < required:
            raise ValueError("图像数据长度不足")
        rows = raw[:required].reshape(int(message.height), int(message.step))
        pixels = rows[:, :valid_step]
        if channels == 1:
            return cv2.cvtColor(
                pixels.reshape(message.height, message.width), cv2.COLOR_GRAY2BGR
            )
        frame = pixels.reshape(message.height, message.width, 3)
        if message.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    @staticmethod
    def bgr_to_image(frame: np.ndarray, stamp) -> Image:
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = "lesson17_debug"
        message.height, message.width = frame.shape[:2]
        message.encoding = "bgr8"
        message.is_bigendian = False
        message.step = int(message.width) * 3
        message.data = frame.tobytes()
        return message

    @staticmethod
    def mono_to_image(mask: np.ndarray, stamp) -> Image:
        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = "lesson17_cone_mask"
        message.height, message.width = mask.shape[:2]
        message.encoding = "mono8"
        message.is_bigendian = False
        message.step = int(message.width)
        message.data = mask.tobytes()
        return message

    def _image_callback(self, message: Image) -> None:
        try:
            frame = self.image_to_bgr(message)
        except Exception as error:
            self.get_logger().error(f"ROS图像转换失败: {error}")
            return
        with self._lock:
            self._latest_frame = frame
            self._latest_frame_time = time.monotonic()
            self._frame_sequence += 1

    def _diagnostics_callback(self, message: DiagnosticArray) -> None:
        errors, remote_ignored = evaluate_diagnostics(
            message, self.ignore_remote_diagnostics
        )
        with self._lock:
            self._diagnostic_time = time.monotonic()
            self._diagnostic_ok = not errors
            self._diagnostic_text = (
                errors[0][:64]
                if errors
                else "OK/REMOTE_IGNORED" if remote_ignored else "OK"
            )

    def _mode_callback(self, message: String) -> None:
        self._set_mode(message.data)

    def _set_mode(self, mode: str) -> bool:
        mode = str(mode).strip().lower()
        if mode not in VALID_MODES:
            self.get_logger().warning(f"忽略无效目标模式: {mode}")
            return False
        if mode != self.target_mode:
            self.target_mode = mode
            self._selected_label = None
            self._was_tracking = False
            self._hard_stop("target_mode_changed")
            self.get_logger().info(f"目标模式切换为: {mode}")
        return True

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.enable_motion = bool(request.data)
        if self.enable_motion:
            self.state = "WAIT_SENSORS"
            response.message = "第17课目标跟随已启用"
        else:
            self.state = "IDLE"
            self._hard_stop("service_disabled")
            response.message = "第17课目标跟随已停止"
        response.success = True
        return response

    def _vision_loop(self) -> None:
        previous: Dict[str, Optional[TargetObservation]] = {
            TARGET_PERSON: None,
            TARGET_RED: None,
            TARGET_BLUE: None,
        }
        processed_count = 0
        while not self._stop_worker.is_set():
            with self._lock:
                sequence = self._frame_sequence
                frame = None if self._latest_frame is None else self._latest_frame.copy()
                target_mode = self.target_mode
            if frame is None or sequence == self._processed_sequence:
                self._stop_worker.wait(0.01)
                continue
            self._processed_sequence = sequence
            processed_count += 1
            now = time.monotonic()
            red_needed = target_mode in (TARGET_RED, TARGET_AUTO)
            blue_needed = target_mode in (TARGET_BLUE, TARGET_AUTO)
            if red_needed or blue_needed:
                cone_frame = cv2.GaussianBlur(
                    frame, (self.cone_blur_kernel, self.cone_blur_kernel), 0
                )
                hsv = cv2.cvtColor(cone_frame, cv2.COLOR_BGR2HSV)
            else:
                hsv = None
            red = (
                self.red_detector.detect(hsv, previous[TARGET_RED], now)
                if red_needed
                else None
            )
            blue = (
                self.blue_detector.detect(hsv, previous[TARGET_BLUE], now)
                if blue_needed
                else None
            )

            person_needed = target_mode in (TARGET_PERSON, TARGET_AUTO)
            person_ran = (
                person_needed
                and processed_count % self.person_detect_interval == 0
            )
            person = previous[TARGET_PERSON]
            inference_start = time.perf_counter()
            if person_ran:
                if person is not None and now - person.detected_at > self.person_lost_timeout:
                    person = None
                person = (
                    None
                    if self.person_detector is None
                    else self.person_detector.detect(frame, person, now)
                )
            inference_ms = (time.perf_counter() - inference_start) * 1000.0

            results = {TARGET_PERSON: person, TARGET_RED: red, TARGET_BLUE: blue}
            with self._lock:
                if person_ran:
                    self._inference_ms = inference_ms
                elif not person_needed:
                    self._inference_ms = 0.0
                for label, observation in results.items():
                    detector_ran = {
                        TARGET_PERSON: person_ran,
                        TARGET_RED: red_needed,
                        TARGET_BLUE: blue_needed,
                    }[label]
                    if not detector_ran:
                        continue
                    if label != TARGET_PERSON:
                        # Reference behavior: use this frame's largest mask blob
                        # directly.  Do not retain, associate or smooth old boxes.
                        self._detections[label] = observation
                        if observation is None:
                            previous[label] = None
                            self._confirm_counts[label] = 0
                        else:
                            previous[label] = observation
                            self._confirm_counts[label] += 1
                        continue

                    held = retain_recent_observation(
                        observation,
                        previous[label],
                        now,
                        self.person_lost_timeout,
                    )
                    self._detections[label] = held
                    if observation is None:
                        if held is None:
                            previous[label] = None
                            self._confirm_counts[label] = 0
                        continue
                    observation = smooth_observation(
                        previous[label], observation, self.person_box_smoothing
                    )
                    self._detections[label] = observation
                    previous[label] = observation
                    self._confirm_counts[label] += 1

    def _snapshot(self, copy_frame: bool = True):
        with self._lock:
            return (
                None
                if not copy_frame or self._latest_frame is None
                else self._latest_frame.copy(),
                self._latest_frame_time,
                dict(self._detections),
                dict(self._confirm_counts),
                self._diagnostic_time,
                self._diagnostic_ok,
                self._diagnostic_text,
                self._inference_ms,
            )

    def _active_target(
        self,
        detections: Dict[str, Optional[TargetObservation]],
        counts: Dict[str, int],
        now: float,
    ) -> Optional[TargetObservation]:
        def confirmed(label: str) -> bool:
            required = (
                self.person_confirm_frames if label == TARGET_PERSON else self.cone_confirm_frames
            )
            observation = detections.get(label)
            timeout = (
                self.person_lost_timeout
                if label == TARGET_PERSON
                else self.cone_lost_timeout
            )
            return bool(
                observation is not None
                and now - observation.detected_at <= timeout
                and counts.get(label, 0) >= required
            )

        if self.target_mode != TARGET_AUTO:
            self._selected_label = self.target_mode if confirmed(self.target_mode) else None
        elif self._selected_label is None or not confirmed(self._selected_label):
            candidates = [label for label in self._detections if confirmed(label)]
            self._selected_label = (
                max(candidates, key=lambda label: detections[label].height_ratio)
                if candidates
                else None
            )
        return None if self._selected_label is None else detections[self._selected_label]

    def _control_loop(self) -> None:
        now = time.monotonic()
        (
            _frame,
            frame_time,
            detections,
            counts,
            diagnostic_time,
            diagnostic_ok,
            diagnostic_text,
            _inference_ms,
        ) = self._snapshot(copy_frame=False)

        if not self.enable_motion:
            self.state = "IDLE"
            self._hard_stop("motion_disabled")
            self._publish_status(now, None)
            return
        if now - frame_time > self.camera_timeout:
            self.state = "WAIT_CAMERA"
            self._hard_stop("camera_timeout")
            self._publish_status(now, None)
            return
        if self.person_detector is None and self.target_mode in (TARGET_PERSON, TARGET_AUTO):
            self.state = "WAIT_MODEL"
            self._hard_stop("person_detector_unavailable")
            self._publish_status(now, None)
            return
        if self.require_diagnostics:
            if now - diagnostic_time > self.diagnostic_timeout or not diagnostic_ok:
                self.state = "WAIT_DIAGNOSTICS"
                self._hard_stop("diagnostics:" + diagnostic_text)
                self._publish_status(now, None)
                return

        target = self._active_target(detections, counts, now)
        if target is None:
            if self._was_tracking:
                self._was_tracking = False
                self._hard_stop("target_lost")
            else:
                self._hard_stop("search_target")
            self.state = "SEARCH_TARGET"
            self._publish_status(now, None)
            return

        if not self._was_tracking:
            self._was_tracking = True
            self._acquired_time = now
            self.state = "TARGET_ACQUIRED"
            self._hard_stop("target_acquired")
            self._publish_status(now, target)
            return
        if now - self._acquired_time < self.target_acquire_hold_seconds:
            self.state = "TARGET_ACQUIRED"
            self._hard_stop("target_acquire_hold")
            self._publish_status(now, target)
            return

        if target.label == TARGET_PERSON:
            self._cone_filtered_angular = 0.0
            linear, angular, state = follow_command(
                target.center_error,
                target.height_ratio,
                self.person_target_height_ratio,
                self.center_deadband,
                self.distance_deadband,
                self.forward_center_limit,
                self.linear_kp,
                self.angular_kp,
                self.angular_sign,
                self.person_min_linear_speed,
                self.minimum_angular_speed,
                self.person_max_linear_speed,
                self.max_reverse_speed,
                self.max_angular_speed,
                self.allow_reverse,
            )
        else:
            linear, raw_angular, state = cone_follow_command(
                target.center_error,
                target.area_ratio,
                self.cone_center_deadband,
                self.cone_slow_area_ratio,
                self.cone_stop_area_ratio,
                self.cone_angular_kp,
                self.angular_sign,
                self.cone_min_linear_speed,
                self.cone_max_linear_speed,
                self.cone_max_angular_speed,
            )
            if raw_angular == 0.0:
                self._cone_filtered_angular = 0.0
            else:
                alpha = self.cone_steering_smoothing
                self._cone_filtered_angular = (
                    alpha * raw_angular
                    + (1.0 - alpha) * self._cone_filtered_angular
                )
            angular = self._cone_filtered_angular
        self.state = state
        self._publish_target(linear, angular, now)
        self._publish_status(now, target)

    @staticmethod
    def _approach(current: float, target: float, maximum_delta: float) -> float:
        return clamp(target, current - maximum_delta, current + maximum_delta)

    def _publish_target(self, linear: float, angular: float, now: float) -> None:
        dt = clamp(now - self._last_command_time, 0.0, 2.0 / self.control_rate)
        self._current_linear = self._approach(
            self._current_linear, linear, self.max_linear_acceleration * dt
        )
        self._current_angular = self._approach(
            self._current_angular, angular, self.max_angular_acceleration * dt
        )
        self._last_command_time = now
        self._publish_velocity(self._current_linear, self._current_angular)

    def _publish_velocity(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.cmd_vel_pub.publish(message)

    def _hard_stop(self, _reason: str) -> None:
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._cone_filtered_angular = 0.0
        self._last_command_time = time.monotonic()
        try:
            self._publish_velocity(0.0, 0.0)
        except Exception as error:
            if not self._closing:
                self.get_logger().error(f"停车指令发布失败: {error}")

    def _publish_posture(self) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
        message.position = [
            float(self.default_height),
            float(self.default_roll),
            float(self.default_pitch),
            float(self.default_slide),
        ]
        self.posture_pub.publish(message)

    def _publish_status(
        self, now: float, target: Optional[TargetObservation]
    ) -> None:
        if now - self._last_status_time < self.status_period:
            return
        self._last_status_time = now
        label = "none" if target is None else target.label
        message = String()
        message.data = (
            f"state={self.state}; mode={self.target_mode}; target={label}; "
            f"vx={self._current_linear:.3f}; wz={self._current_angular:.3f}"
        )
        self.status_pub.publish(message)
        self.get_logger().info(message.data)

    @staticmethod
    def _target_color(label: str) -> Tuple[int, int, int]:
        return {
            TARGET_PERSON: (0, 255, 0),
            TARGET_RED: (0, 0, 255),
            TARGET_BLUE: (255, 0, 0),
        }[label]

    def _start_recording(self, frame: np.ndarray) -> None:
        if self._video_writer is not None:
            return
        try:
            self.record_directory.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = self.record_directory / f"lesson17_debug_{timestamp}.avi"
            height, width = frame.shape[:2]
            writer = cv2.VideoWriter(
                str(path),
                cv2.VideoWriter_fourcc(*self.record_codec),
                float(self.record_fps),
                (int(width), int(height)),
            )
            if not writer.isOpened():
                writer.release()
                raise RuntimeError(
                    f"VideoWriter 无法使用编码器 {self.record_codec}"
                )
            self._video_writer = writer
            self._video_path = path
            self._recording_started_time = time.monotonic()
            self._next_recording_frame_time = self._recording_started_time
            self.get_logger().info(f"开始录像: {path}")
        except Exception as error:
            self._recording_requested = False
            self.get_logger().error(f"录像启动失败: {error}")

    def _stop_recording(self) -> None:
        writer = self._video_writer
        path = self._video_path
        self._video_writer = None
        self._video_path = None
        if writer is not None:
            writer.release()
            self.get_logger().info(f"录像已保存: {path}")

    def _record_debug_frame(self, frame: np.ndarray, now: float) -> None:
        if self._video_writer is None:
            return
        if now - self._recording_started_time >= self.record_max_seconds:
            self._recording_requested = False
            self.get_logger().warning(
                f"录像达到 {self.record_max_seconds:.0f} 秒上限，自动停止"
            )
            self._stop_recording()
            return
        if now + 1e-3 < self._next_recording_frame_time:
            return
        self._video_writer.write(frame)
        interval = 1.0 / self.record_fps
        self._next_recording_frame_time += interval
        if self._next_recording_frame_time < now - interval:
            self._next_recording_frame_time = now + interval

    def _draw_debug(
        self,
        frame: np.ndarray,
        detections: Dict[str, Optional[TargetObservation]],
        inference_ms: float,
        diagnostic_text: str,
        binary_mask: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        now = time.monotonic()
        for label, observation in detections.items():
            timeout = (
                self.person_lost_timeout
                if label == TARGET_PERSON
                else self.cone_lost_timeout
            )
            if observation is None or now - observation.detected_at > timeout:
                continue
            x, y, width, height = observation.bbox
            color = self._target_color(label)
            selected = label == self._selected_label
            cv2.rectangle(
                frame,
                (x, y),
                (x + width, y + height),
                color,
                4 if selected else 2,
            )
            cv2.putText(
                frame,
                (
                    f"{label} area={observation.area_ratio * 100.0:.2f}%"
                    if label != TARGET_PERSON
                    else f"{label} {observation.confidence:.2f}"
                ),
                (x, max(22, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                color,
                2,
                cv2.LINE_AA,
            )
            if selected:
                tracked_x = int(
                    0.5 * frame.shape[1] * (observation.center_error + 1.0)
                )
                cv2.circle(
                    frame,
                    (tracked_x, int(y + 0.5 * height)),
                    6,
                    (0, 255, 255),
                    -1,
                )

        lines = [
            f"MODE: {self.target_mode}",
            f"STATE: {self.state}",
            f"MOTION: {'ENABLED' if self.enable_motion else 'STOPPED'}",
            f"REC: {'ON' if self._video_writer is not None else 'OFF'}",
            f"CMD vx={self._current_linear:.2f} wz={self._current_angular:.2f}",
            f"DNN {inference_ms:.0f}ms  DIAG {diagnostic_text}",
            "P:Person R:Red B:Blue A:Auto",
            "S:Start SPACE:E-Stop V:Record M:Mask Q:Quit",
        ]
        for index, line in enumerate(lines):
            cv2.putText(
                frame,
                line,
                (12, 27 + index * 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        if binary_mask is not None:
            preview_width = min(180, max(80, frame.shape[1] // 3))
            preview_height = min(135, max(60, frame.shape[0] // 3))
            preview = cv2.resize(
                binary_mask,
                (preview_width, preview_height),
                interpolation=cv2.INTER_NEAREST,
            )
            preview = cv2.cvtColor(preview, cv2.COLOR_GRAY2BGR)
            x1 = frame.shape[1] - preview_width - 10
            y1 = frame.shape[0] - preview_height - 10
            frame[y1 : y1 + preview_height, x1 : x1 + preview_width] = preview
            cv2.rectangle(
                frame,
                (x1 - 2, y1 - 22),
                (x1 + preview_width + 2, y1 + preview_height + 2),
                (0, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                "CONE BINARY MASK",
                (x1 + 3, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
        return frame

    def _cone_binary_mask(self, frame: np.ndarray) -> Optional[np.ndarray]:
        blurred = cv2.GaussianBlur(
            frame, (self.cone_blur_kernel, self.cone_blur_kernel), 0
        )
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        if self.target_mode == TARGET_RED:
            return self.red_detector.create_mask(hsv)
        if self.target_mode == TARGET_BLUE:
            return self.blue_detector.create_mask(hsv)
        # person/auto 模式显示红蓝合并掩膜，方便不切模式也能直接调阈值。
        return cv2.bitwise_or(
            self.red_detector.create_mask(hsv),
            self.blue_detector.create_mask(hsv),
        )

    def _display_loop(self) -> None:
        frame, _, detections, _, _, _, diagnostic_text, inference_ms = self._snapshot()
        if frame is None:
            return
        binary_mask = self._cone_binary_mask(frame)
        if self._recording_requested and self._video_writer is None:
            self._start_recording(frame)
        elif not self._recording_requested and self._video_writer is not None:
            self._stop_recording()
        debug = self._draw_debug(
            frame,
            detections,
            inference_ms,
            diagnostic_text,
            binary_mask if self.show_binary_mask else None,
        )
        self._record_debug_frame(debug, time.monotonic())
        if self.debug_image_pub.get_subscription_count() > 0:
            self.debug_image_pub.publish(
                self.bgr_to_image(debug, self.get_clock().now().to_msg())
            )
        if binary_mask is not None and self.cone_mask_pub.get_subscription_count() > 0:
            self.cone_mask_pub.publish(
                self.mono_to_image(binary_mask, self.get_clock().now().to_msg())
            )
        if not self.show_debug:
            return
        try:
            cv2.imshow(self.debug_window_name, debug)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            self.get_logger().error(f"调试窗口不可用，关闭本地显示: {error}")
            self.show_debug = False
            return
        if key in (ord("p"), ord("P")):
            self._set_mode(TARGET_PERSON)
        elif key in (ord("r"), ord("R")):
            self._set_mode(TARGET_RED)
        elif key in (ord("b"), ord("B")):
            self._set_mode(TARGET_BLUE)
        elif key in (ord("a"), ord("A")):
            self._set_mode(TARGET_AUTO)
        elif key in (ord("s"), ord("S")):
            self.enable_motion = True
            self.state = "WAIT_SENSORS"
        elif key in (ord("v"), ord("V")):
            self._recording_requested = not self._recording_requested
            if not self._recording_requested:
                self._stop_recording()
        elif key in (ord("m"), ord("M")):
            self.show_binary_mask = not self.show_binary_mask
        elif key == 32:
            self.enable_motion = False
            self.state = "IDLE"
            self._hard_stop("keyboard_emergency_stop")
        elif key in (ord("q"), ord("Q"), 27):
            self.enable_motion = False
            self._hard_stop("keyboard_quit")
            rclpy.shutdown()

    def cleanup(self) -> None:
        self._closing = True
        self.enable_motion = False
        self._stop_worker.set()
        self._recording_requested = False
        self._stop_recording()
        for _ in range(3):
            try:
                self._publish_velocity(0.0, 0.0)
                time.sleep(0.03)
            except Exception:
                pass
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ros2MultiTargetFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
