#!/usr/bin/env python3
"""Lesson 18: person-aware mapless patrol with depth obstacle avoidance.

The node is deliberately motion-disabled by default.  Once enabled it cruises
slowly, uses ``wheel_robot/msg/ObstacleStatus`` to avoid obstacles in small
scan-and-verify turns, and temporarily interrupts patrol to face/follow a
detected person.  Losing the person causes an immediate stop followed by a
delayed return to patrol.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random
import threading
import time
from pathlib import Path
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from wheel_robot.msg import ObstacleStatus


def clamp(value: float, low: float, high: float) -> float:
    return float(max(low, min(high, value)))


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi)."""
    return float((angle + math.pi) % (2.0 * math.pi) - math.pi)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return float(math.atan2(sin_yaw, cos_yaw))


def _usable_distance(value: float) -> Optional[float]:
    if not math.isfinite(value) or value <= 0.0:
        return None
    return float(value)


def choose_turn_direction(
    left_distance: float,
    right_distance: float,
    left_status: int,
    right_status: int,
    previous_direction: int = 1,
) -> int:
    """Return +1 for left or -1 for right using valid free-space evidence.

    CLEAR/CAUTION status is considered before distance.  If neither side has a
    trustworthy measurement, alternating away from the previous choice avoids
    repeatedly probing the same blind direction.
    """
    left = _usable_distance(left_distance)
    right = _usable_distance(right_distance)

    def score(status: int, distance: Optional[float]) -> float:
        if status == ObstacleStatus.CLEAR:
            base = 20.0
        elif status == ObstacleStatus.CAUTION:
            base = 10.0
        elif status == ObstacleStatus.STOP:
            base = 0.0
        else:
            base = -10.0
        return base + (min(distance, 8.0) if distance is not None else -2.0)

    left_score = score(int(left_status), left)
    right_score = score(int(right_status), right)
    if abs(left_score - right_score) < 0.05:
        return -1 if previous_direction >= 0 else 1
    return 1 if left_score > right_score else -1


def corridor_is_passable(
    left_status: int,
    center_status: int,
    right_status: int,
) -> bool:
    """Accept a slow escape corridor without demanding three CLEAR zones.

    The center must have a usable measurement, neither edge may be at STOP,
    and at least one edge must be observable.  A single blind edge is accepted
    only as CAUTION by the depth node and the steering controller turns away
    from it.
    """
    left_status = int(left_status)
    center_status = int(center_status)
    right_status = int(right_status)
    if center_status not in (ObstacleStatus.CLEAR, ObstacleStatus.CAUTION):
        return False
    if left_status == ObstacleStatus.STOP or right_status == ObstacleStatus.STOP:
        return False
    return not (
        left_status == ObstacleStatus.UNKNOWN
        and right_status == ObstacleStatus.UNKNOWN
    )


def patrol_command(
    left_distance: float,
    center_distance: float,
    right_distance: float,
    left_status: int,
    center_status: int,
    right_status: int,
    cruise_speed: float,
    caution_speed: float,
    steering_kp: float,
    steering_deadband_m: float,
    max_angular_speed: float,
) -> Tuple[float, float, str]:
    """Generate a smooth free-space-guided patrol command.

    Positive angular velocity turns left.  Therefore a larger left clearance
    creates a positive steering command, while an invalid/close left edge
    steers the robot to the right.
    """
    left_status = int(left_status)
    center_status = int(center_status)
    right_status = int(right_status)
    direction = choose_turn_direction(
        left_distance,
        right_distance,
        left_status,
        right_status,
        previous_direction=1,
    )
    if center_status in (ObstacleStatus.UNKNOWN, ObstacleStatus.STOP):
        return 0.0, direction * max_angular_speed, "FRONT_BLOCKED"
    if left_status == ObstacleStatus.STOP or right_status == ObstacleStatus.STOP:
        return 0.0, direction * max_angular_speed, "EDGE_BLOCKED"

    left = _usable_distance(left_distance)
    right = _usable_distance(right_distance)
    if left_status == ObstacleStatus.UNKNOWN:
        clearance_error = -1.0
    elif right_status == ObstacleStatus.UNKNOWN:
        clearance_error = 1.0
    elif left is None and right is None:
        clearance_error = 0.0
    elif left is None:
        clearance_error = -1.0
    elif right is None:
        clearance_error = 1.0
    else:
        difference = left - right
        if abs(difference) <= steering_deadband_m:
            clearance_error = 0.0
        else:
            clearance_error = difference / max(0.35, left, right)

    angular = clamp(
        steering_kp * clearance_error,
        -max_angular_speed,
        max_angular_speed,
    )
    cautious = any(
        status != ObstacleStatus.CLEAR
        for status in (left_status, center_status, right_status)
    )
    linear = caution_speed if cautious else cruise_speed
    if max_angular_speed > 1.0e-6:
        turn_fraction = abs(angular) / max_angular_speed
        linear *= 1.0 - 0.40 * clamp(turn_fraction, 0.0, 1.0)
    state = "CAUTION_GUIDED" if cautious else "ROAM_GUIDED"
    return max(0.0, linear), angular, state


def person_follow_command(
    center_error: float,
    height_ratio: float,
    target_height_ratio: float,
    center_deadband: float,
    distance_deadband: float,
    forward_center_limit: float,
    linear_kp: float,
    angular_kp: float,
    angular_sign: float,
    max_linear_speed: float,
    max_reverse_speed: float,
    max_angular_speed: float,
    allow_reverse: bool,
) -> Tuple[float, float, str]:
    """Compute a bounded command shared by runtime code and unit tests."""
    center_error = clamp(center_error, -1.0, 1.0)
    if abs(center_error) <= center_deadband:
        angular = 0.0
    else:
        angular = angular_sign * angular_kp * center_error
    angular = clamp(angular, -max_angular_speed, max_angular_speed)

    if abs(center_error) > forward_center_limit:
        return 0.0, angular, "PERSON_ALIGN"

    distance_error = target_height_ratio - height_ratio
    if abs(distance_error) <= distance_deadband:
        linear = 0.0
    else:
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
    return linear, angular, "PERSON_FOLLOW"


@dataclass(frozen=True)
class PersonObservation:
    bbox: Tuple[int, int, int, int]
    confidence: float
    center_error: float
    height_ratio: float
    detected_at: float


def smooth_person_observation(
    previous: Optional[PersonObservation],
    current: PersonObservation,
    alpha: float,
) -> PersonObservation:
    """Low-pass person box geometry while preserving the newest timestamp."""
    if previous is None:
        return current
    alpha = clamp(alpha, 0.0, 1.0)
    old_weight = 1.0 - alpha
    bbox = tuple(
        int(round(old_weight * old + alpha * new))
        for old, new in zip(previous.bbox, current.bbox)
    )
    return PersonObservation(
        bbox=bbox,
        confidence=old_weight * previous.confidence + alpha * current.confidence,
        center_error=old_weight * previous.center_error + alpha * current.center_error,
        height_ratio=old_weight * previous.height_ratio + alpha * current.height_ratio,
        detected_at=current.detected_at,
    )


class PersonDetector:
    """MobileNet-SSD person detector with lightweight target continuity."""

    PERSON_CLASS_ID = 15

    def __init__(
        self,
        prototxt_path: Path,
        model_path: Path,
        confidence_threshold: float,
        nms_threshold: float,
        min_area_ratio: float,
    ) -> None:
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.min_area_ratio = min_area_ratio
        self.net = cv2.dnn.readNetFromCaffe(str(prototxt_path), str(model_path))
        self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    @staticmethod
    def _select_target(
        candidates: Sequence[Tuple[Tuple[int, int, int, int], float]],
        previous: Optional[PersonObservation],
        frame_width: int,
        frame_height: int,
    ) -> Optional[Tuple[Tuple[int, int, int, int], float]]:
        if not candidates:
            return None
        if previous is None:
            return max(candidates, key=lambda item: item[0][2] * item[0][3])

        previous_x, previous_y, previous_w, previous_h = previous.bbox
        previous_cx = previous_x + 0.5 * previous_w
        previous_cy = previous_y + 0.5 * previous_h
        previous_ratio = previous_h / max(1.0, float(frame_height))

        def cost(item: Tuple[Tuple[int, int, int, int], float]) -> float:
            (x, y, width, height), confidence = item
            dx = (x + 0.5 * width - previous_cx) / max(1.0, frame_width)
            dy = (y + 0.5 * height - previous_cy) / max(1.0, frame_height)
            size_delta = abs(height / max(1.0, frame_height) - previous_ratio)
            return math.hypot(dx, dy) + 0.55 * size_delta - 0.10 * confidence

        selected = min(candidates, key=cost)
        # Do not jump to an unrelated person on the other side of the image.
        if cost(selected) > 0.42:
            return None
        return selected

    def detect(
        self,
        frame: np.ndarray,
        previous: Optional[PersonObservation],
        now: float,
    ) -> Optional[PersonObservation]:
        frame_height, frame_width = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame,
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
        minimum_area = frame_width * frame_height * self.min_area_ratio
        for index in range(detections.shape[2]):
            class_id = int(detections[0, 0, index, 1])
            confidence = float(detections[0, 0, index, 2])
            if class_id != self.PERSON_CLASS_ID or confidence < self.confidence_threshold:
                continue
            x1 = int(detections[0, 0, index, 3] * frame_width)
            y1 = int(detections[0, 0, index, 4] * frame_height)
            x2 = int(detections[0, 0, index, 5] * frame_width)
            y2 = int(detections[0, 0, index, 6] * frame_height)
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
            boxes, scores, self.confidence_threshold, self.nms_threshold
        )
        candidates = []
        for raw_index in np.asarray(indices).reshape(-1):
            index = int(raw_index)
            candidates.append((tuple(boxes[index]), float(scores[index])))
        selected = self._select_target(
            candidates, previous, frame_width, frame_height
        )
        if selected is None:
            return None
        (x, y, width, height), confidence = selected
        center_x = x + 0.5 * width
        center_error = (center_x - 0.5 * frame_width) / max(1.0, 0.5 * frame_width)
        return PersonObservation(
            bbox=(x, y, width, height),
            confidence=confidence,
            center_error=clamp(center_error, -1.0, 1.0),
            height_ratio=height / max(1.0, float(frame_height)),
            detected_at=now,
        )


class PersonAwarePatrol(Node):
    def __init__(self) -> None:
        super().__init__("lesson18_person_patrol")
        self._declare_parameters()
        self._read_parameters()
        self._validate_parameters()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(
            Image, self.image_topic, self._image_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            ObstacleStatus, self.obstacle_topic, self._obstacle_callback, 10
        )
        self.create_subscription(Odometry, self.odom_topic, self._odom_callback, 10)
        self.enable_service = self.create_service(
            SetBool, "/lesson18/enable", self._enable_callback
        )

        self._data_lock = threading.Lock()
        self._latest_frame: Optional[np.ndarray] = None
        self._latest_frame_time = 0.0
        self._frame_sequence = 0
        self._processed_sequence = -1
        self._observation: Optional[PersonObservation] = None
        self._person_confirm_count = 0
        self._person_locked = False
        self._inference_ms = 0.0

        self._obstacle: Optional[ObstacleStatus] = None
        self._obstacle_time = 0.0
        self._odom_yaw: Optional[float] = None
        self._odom_time = 0.0
        self.state = "IDLE" if not self.enable_motion else "WAIT_SENSORS"
        self.state_since = time.monotonic()
        self._was_following_person = False
        self._last_person_lost_time = 0.0
        self._avoid_direction = 1
        self._last_turn_direction = 1
        self._avoid_steps = 0
        self._turn_start_yaw: Optional[float] = None
        self._turn_start_time = 0.0
        self._turn_angle = 0.0
        self._turn_direction = 1

        self._random = random.Random(self.random_seed)
        self._next_random_turn_time = self._schedule_random_turn(time.monotonic())

        self._current_linear = 0.0
        self._current_angular = 0.0
        self._last_command_time = time.monotonic()
        self._last_log_time = 0.0
        self._last_stop_reason = "startup"
        self._closing = False

        self.detector: Optional[PersonDetector] = None
        try:
            self.detector = PersonDetector(
                self.prototxt_path,
                self.model_path,
                self.person_confidence,
                self.person_nms_threshold,
                self.person_min_area_ratio,
            )
            self.get_logger().info("MobileNet-SSD person detector loaded")
        except Exception as error:
            self.get_logger().error(f"人体模型加载失败: {error}")

        self._stop_worker = threading.Event()
        self._worker = threading.Thread(
            target=self._inference_loop, name="lesson18_person_dnn", daemon=True
        )
        self._worker.start()
        self.control_timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.display_timer = self.create_timer(1.0 / self.display_rate, self._display_loop)

        self._hard_stop("startup")
        mode = "运动启用" if self.enable_motion else "安全待机"
        self.get_logger().info(
            f"第18课自主巡逻节点启动：{mode}；服务 /lesson18/enable 可启停"
        )
        self.get_logger().info("窗口按键：S启动，Space急停，Q停车退出")

    def _declare_parameters(self) -> None:
        defaults = {
            "enable_motion": False,
            "image_topic": "/camera/color/image_raw",
            "obstacle_topic": "/camera/depth/obstacle_status",
            "odom_topic": "/odom",
            "control_rate": 20.0,
            "display_rate": 15.0,
            "show_debug": True,
            "debug_window_name": "Lesson 18 - Person Aware Patrol",
            "camera_timeout": 0.60,
            "obstacle_timeout": 0.45,
            "odom_timeout": 0.50,
            "require_camera": True,
            "model_directory": "",
            "person_confidence": 0.48,
            "person_nms_threshold": 0.35,
            "person_min_area_ratio": 0.003,
            "person_detect_interval": 1,
            "person_confirm_frames": 2,
            "person_smoothing_alpha": 0.45,
            "person_acquire_hold_seconds": 0.25,
            "person_lost_timeout": 0.70,
            "person_resume_delay": 1.00,
            "person_target_height_ratio": 0.42,
            "person_distance_deadband": 0.040,
            "person_center_deadband": 0.07,
            "person_forward_center_limit": 0.32,
            "person_linear_kp": 0.55,
            "person_angular_kp": 0.70,
            "person_angular_sign": -1.0,
            "person_max_linear_speed": 0.14,
            "person_max_reverse_speed": 0.04,
            "allow_reverse": False,
            "person_center_stop_exemption": 0.18,
            "cruise_speed": 0.16,
            "caution_speed": 0.06,
            "patrol_steering_kp": 0.32,
            "patrol_steering_deadband_m": 0.10,
            "patrol_max_angular_speed": 0.18,
            "turn_speed": 0.28,
            "max_angular_speed": 0.35,
            "max_linear_acceleration": 0.20,
            "max_angular_acceleration": 0.75,
            "stop_hold_seconds": 0.20,
            "verify_hold_seconds": 0.20,
            "turn_step_rad": 0.35,
            "max_avoid_steps": 5,
            "turn_timeout_scale": 1.60,
            "avoid_escape_speed": 0.06,
            "avoid_escape_seconds": 1.20,
            "random_turn_enabled": True,
            "random_turn_min_interval": 14.0,
            "random_turn_max_interval": 24.0,
            "random_turn_min_angle": 0.10,
            "random_turn_max_angle": 0.25,
            "random_turn_linear_speed": 0.07,
            "random_seed": 18,
            "status_log_period": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        for name in (
            "enable_motion",
            "image_topic",
            "obstacle_topic",
            "odom_topic",
            "control_rate",
            "display_rate",
            "show_debug",
            "debug_window_name",
            "camera_timeout",
            "obstacle_timeout",
            "odom_timeout",
            "require_camera",
            "person_confidence",
            "person_nms_threshold",
            "person_min_area_ratio",
            "person_detect_interval",
            "person_confirm_frames",
            "person_smoothing_alpha",
            "person_acquire_hold_seconds",
            "person_lost_timeout",
            "person_resume_delay",
            "person_target_height_ratio",
            "person_distance_deadband",
            "person_center_deadband",
            "person_forward_center_limit",
            "person_linear_kp",
            "person_angular_kp",
            "person_angular_sign",
            "person_max_linear_speed",
            "person_max_reverse_speed",
            "allow_reverse",
            "person_center_stop_exemption",
            "cruise_speed",
            "caution_speed",
            "patrol_steering_kp",
            "patrol_steering_deadband_m",
            "patrol_max_angular_speed",
            "turn_speed",
            "max_angular_speed",
            "max_linear_acceleration",
            "max_angular_acceleration",
            "stop_hold_seconds",
            "verify_hold_seconds",
            "turn_step_rad",
            "max_avoid_steps",
            "turn_timeout_scale",
            "avoid_escape_speed",
            "avoid_escape_seconds",
            "random_turn_enabled",
            "random_turn_min_interval",
            "random_turn_max_interval",
            "random_turn_min_angle",
            "random_turn_max_angle",
            "random_turn_linear_speed",
            "random_seed",
            "status_log_period",
        ):
            setattr(self, name, self.get_parameter(name).value)

        configured_model_dir = str(self.get_parameter("model_directory").value).strip()
        model_dir = Path(configured_model_dir) if configured_model_dir else Path(__file__).resolve().parent / "models"
        self.prototxt_path = model_dir / "deploy.prototxt"
        self.model_path = model_dir / "mobilenet_iter_73000.caffemodel"

        self.person_detect_interval = int(self.person_detect_interval)
        self.person_confirm_frames = int(self.person_confirm_frames)
        self.max_avoid_steps = int(self.max_avoid_steps)
        self.random_seed = int(self.random_seed)

    def _validate_parameters(self) -> None:
        positive = {
            "control_rate": self.control_rate,
            "display_rate": self.display_rate,
            "camera_timeout": self.camera_timeout,
            "obstacle_timeout": self.obstacle_timeout,
            "turn_speed": self.turn_speed,
            "max_angular_speed": self.max_angular_speed,
            "turn_step_rad": self.turn_step_rad,
            "avoid_escape_seconds": self.avoid_escape_seconds,
        }
        invalid = [name for name, value in positive.items() if float(value) <= 0.0]
        if invalid:
            raise ValueError("参数必须为正数: " + ", ".join(invalid))
        if self.person_detect_interval < 1 or self.person_confirm_frames < 1:
            raise ValueError("person_detect_interval/person_confirm_frames 必须 >= 1")
        if self.person_acquire_hold_seconds < 0.0:
            raise ValueError("person_acquire_hold_seconds 不能为负数")
        if not 0.0 < self.person_smoothing_alpha <= 1.0:
            raise ValueError("person_smoothing_alpha 必须在 (0, 1] 内")
        if self.max_avoid_steps < 1:
            raise ValueError("max_avoid_steps 必须 >= 1")
        if not 0.0 < self.person_target_height_ratio < 1.0:
            raise ValueError("person_target_height_ratio 必须在 (0, 1) 内")
        if self.random_turn_max_interval < self.random_turn_min_interval:
            raise ValueError("随机转向最大间隔不能小于最小间隔")
        if self.random_turn_max_angle < self.random_turn_min_angle:
            raise ValueError("随机转向最大角不能小于最小角")
        if self.patrol_max_angular_speed > self.max_angular_speed:
            raise ValueError("patrol_max_angular_speed 不能超过 max_angular_speed")
        if self.caution_speed > self.cruise_speed:
            raise ValueError("caution_speed 不能超过 cruise_speed")
        if self.avoid_escape_speed > self.cruise_speed:
            raise ValueError("avoid_escape_speed 不能超过 cruise_speed")

    @staticmethod
    def _image_to_bgr(message: Image) -> np.ndarray:
        if message.encoding not in ("bgr8", "rgb8", "mono8"):
            raise ValueError(f"不支持的彩色图编码: {message.encoding}")
        channels = 1 if message.encoding == "mono8" else 3
        required_step = int(message.width) * channels
        if int(message.step) < required_step:
            raise ValueError("图像 step 小于有效行宽")
        raw = np.frombuffer(message.data, dtype=np.uint8)
        required_size = int(message.step) * int(message.height)
        if raw.size < required_size:
            raise ValueError("图像数据长度不足")
        rows = raw[:required_size].reshape(int(message.height), int(message.step))
        pixels = rows[:, :required_step]
        if channels == 1:
            return cv2.cvtColor(pixels.reshape(message.height, message.width), cv2.COLOR_GRAY2BGR)
        frame = pixels.reshape(message.height, message.width, 3)
        if message.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame.copy()

    def _image_callback(self, message: Image) -> None:
        try:
            frame = self._image_to_bgr(message)
        except Exception as error:
            self.get_logger().error(f"彩色图转换失败: {error}")
            return
        with self._data_lock:
            self._latest_frame = frame
            self._latest_frame_time = time.monotonic()
            self._frame_sequence += 1

    def _obstacle_callback(self, message: ObstacleStatus) -> None:
        with self._data_lock:
            self._obstacle = message
            self._obstacle_time = time.monotonic()

    def _odom_callback(self, message: Odometry) -> None:
        q = message.pose.pose.orientation
        with self._data_lock:
            self._odom_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self._odom_time = time.monotonic()

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        self.enable_motion = bool(request.data)
        now = time.monotonic()
        if self.enable_motion:
            self._set_state("WAIT_SENSORS", now)
            self._next_random_turn_time = self._schedule_random_turn(now)
            response.message = "自主巡逻已启用，等待传感器安全"
        else:
            self._set_state("IDLE", now)
            self._hard_stop("service_disabled")
            response.message = "自主巡逻已停止"
        response.success = True
        return response

    def _inference_loop(self) -> None:
        previous: Optional[PersonObservation] = None
        while not self._stop_worker.is_set():
            with self._data_lock:
                sequence = self._frame_sequence
                frame = None if self._latest_frame is None else self._latest_frame.copy()
            if frame is None or sequence == self._processed_sequence:
                self._stop_worker.wait(0.01)
                continue
            self._processed_sequence = sequence
            if sequence % self.person_detect_interval != 0:
                continue

            observation = None
            start = time.perf_counter()
            if self.detector is not None:
                try:
                    if previous is not None and time.monotonic() - previous.detected_at > self.person_lost_timeout:
                        previous = None
                    detected = self.detector.detect(frame, previous, time.monotonic())
                    if detected is not None:
                        observation = smooth_person_observation(
                            previous, detected, self.person_smoothing_alpha
                        )
                except Exception as error:
                    self.get_logger().error(f"人体推理失败: {error}")
            inference_ms = (time.perf_counter() - start) * 1000.0

            with self._data_lock:
                self._inference_ms = inference_ms
                if observation is None:
                    self._person_confirm_count = 0
                else:
                    previous = observation
                    self._observation = observation
                    self._person_confirm_count += 1
                    if self._person_confirm_count >= self.person_confirm_frames:
                        self._person_locked = True

    def _snapshot(self):
        with self._data_lock:
            return (
                self._observation,
                self._person_locked,
                self._latest_frame_time,
                self._obstacle,
                self._obstacle_time,
                self._odom_yaw,
                self._odom_time,
            )

    def _person_is_active(
        self, observation: Optional[PersonObservation], locked: bool, now: float
    ) -> bool:
        return bool(
            locked
            and observation is not None
            and now - observation.detected_at <= self.person_lost_timeout
        )

    def _sensor_failure_reason(
        self,
        now: float,
        frame_time: float,
        obstacle: Optional[ObstacleStatus],
        obstacle_time: float,
    ) -> Optional[str]:
        if obstacle is None or now - obstacle_time > self.obstacle_timeout:
            return "obstacle_timeout"
        if int(obstacle.status) == ObstacleStatus.UNKNOWN:
            return "obstacle_unknown"
        if self.require_camera and now - frame_time > self.camera_timeout:
            return "camera_timeout"
        if self.detector is None:
            return "person_detector_unavailable"
        return None

    def _control_loop(self) -> None:
        now = time.monotonic()
        (
            observation,
            person_locked,
            frame_time,
            obstacle,
            obstacle_time,
            odom_yaw,
            odom_time,
        ) = self._snapshot()

        if not self.enable_motion:
            self._set_state("IDLE", now)
            self._hard_stop("motion_disabled")
            self._log_status(now, observation, obstacle)
            return

        failure = self._sensor_failure_reason(
            now,
            frame_time,
            obstacle,
            obstacle_time,
        )
        if failure is not None:
            self._set_state("WAIT_SENSORS", now)
            self._hard_stop(failure)
            self._log_status(now, observation, obstacle)
            return
        assert obstacle is not None

        person_active = self._person_is_active(observation, person_locked, now)
        if not person_active and person_locked:
            with self._data_lock:
                self._person_locked = False
                self._person_confirm_count = 0

        # A STOP in any of the three wide-robot zones has priority.  A centered
        # person is the sole exception: in that case stop rather than turn away.
        avoidance_states = (
            "AVOID_STOP",
            "AVOID_TURN",
            "AVOID_VERIFY",
            "AVOID_ESCAPE",
            "BLOCKED",
        )
        if int(obstacle.status) == ObstacleStatus.STOP:
            centered_person = bool(
                person_active
                and observation is not None
                and abs(observation.center_error) <= self.person_center_stop_exemption
            )
            if centered_person:
                self._was_following_person = True
                self._set_state("PERSON_TOO_CLOSE", now)
                self._hard_stop("person_or_center_obstacle_too_close")
            else:
                if self.state in avoidance_states:
                    self._handle_avoidance(now, obstacle, odom_yaw, odom_time)
                else:
                    self._begin_avoidance(now, obstacle)
            self._log_status(now, observation, obstacle)
            return

        # Once the corridor is safe enough to move slowly, a confirmed person
        # interrupts scanning and becomes the active task.
        if person_active and observation is not None:
            if not self._was_following_person:
                self._was_following_person = True
                self._set_state("PERSON_ACQUIRE_STOP", now)
                self._hard_stop("person_acquired")
                self._log_status(now, observation, obstacle)
                return
            if (
                self.state == "PERSON_ACQUIRE_STOP"
                and now - self.state_since < self.person_acquire_hold_seconds
            ):
                self._hard_stop("person_acquire_hold")
                self._log_status(now, observation, obstacle)
                return
            self._handle_person(now, observation, obstacle)
            self._log_status(now, observation, obstacle)
            return

        if self.state in avoidance_states:
            self._handle_avoidance(now, obstacle, odom_yaw, odom_time)
            self._log_status(now, observation, obstacle)
            return

        if self._was_following_person:
            self._was_following_person = False
            self._last_person_lost_time = now
            self._set_state("PERSON_LOST", now)
            self._hard_stop("person_lost")
            self._log_status(now, observation, obstacle)
            return

        if self.state == "PERSON_LOST":
            if now - self._last_person_lost_time < self.person_resume_delay:
                self._hard_stop("person_lost_wait")
                self._log_status(now, observation, obstacle)
                return
            self._set_state("ROAM", now)
            self._next_random_turn_time = self._schedule_random_turn(now)

        if self.state == "RANDOM_TURN":
            if (
                int(obstacle.status) != ObstacleStatus.CLEAR
                or self._turn_complete(now, odom_yaw, odom_time)
            ):
                self._set_state("ROAM", now)
                self._next_random_turn_time = self._schedule_random_turn(now)
            else:
                self._publish_target(
                    min(self.cruise_speed, self.random_turn_linear_speed),
                    self._turn_direction * self.turn_speed,
                    now,
                )
                self._log_status(now, observation, obstacle)
                return

        if (
            self.random_turn_enabled
            and int(obstacle.status) == ObstacleStatus.CLEAR
            and now >= self._next_random_turn_time
        ):
            direction = self._random.choice((-1, 1))
            angle = self._random.uniform(
                self.random_turn_min_angle, self.random_turn_max_angle
            )
            self._start_turn(now, direction, angle, odom_yaw, "RANDOM_TURN")
            self._publish_target(
                min(self.cruise_speed, self.random_turn_linear_speed),
                direction * self.turn_speed,
                now,
            )
            self._log_status(now, observation, obstacle)
            return

        linear, angular, state = self._patrol_command(obstacle)
        self._set_state(state, now)
        self._publish_target(linear, angular, now)
        self._log_status(now, observation, obstacle)

    def _patrol_command(self, obstacle: ObstacleStatus) -> Tuple[float, float, str]:
        return patrol_command(
            obstacle.left_distance,
            obstacle.center_distance,
            obstacle.right_distance,
            obstacle.left_status,
            obstacle.center_status,
            obstacle.right_status,
            self.cruise_speed,
            self.caution_speed,
            self.patrol_steering_kp,
            self.patrol_steering_deadband_m,
            self.patrol_max_angular_speed,
        )

    def _handle_person(
        self, now: float, observation: PersonObservation, obstacle: ObstacleStatus
    ) -> None:
        linear, angular, state = person_follow_command(
            observation.center_error,
            observation.height_ratio,
            self.person_target_height_ratio,
            self.person_center_deadband,
            self.person_distance_deadband,
            self.person_forward_center_limit,
            self.person_linear_kp,
            self.person_angular_kp,
            self.person_angular_sign,
            self.person_max_linear_speed,
            self.person_max_reverse_speed,
            self.max_angular_speed,
            self.allow_reverse,
        )
        if int(obstacle.status) == ObstacleStatus.CAUTION and linear > 0.0:
            linear = min(linear, self.caution_speed)
        self._set_state(state, now)
        self._publish_target(linear, angular, now)

    def _begin_avoidance(self, now: float, obstacle: ObstacleStatus) -> None:
        self._avoid_direction = choose_turn_direction(
            obstacle.left_distance,
            obstacle.right_distance,
            obstacle.left_status,
            obstacle.right_status,
            self._last_turn_direction,
        )
        self._last_turn_direction = self._avoid_direction
        self._avoid_steps = 0
        self._set_state("AVOID_STOP", now)
        self._hard_stop("obstacle_stop")

    def _handle_avoidance(
        self,
        now: float,
        obstacle: ObstacleStatus,
        odom_yaw: Optional[float],
        odom_time: float,
    ) -> None:
        if self.state == "AVOID_STOP":
            self._hard_stop("avoid_stop_hold")
            if corridor_is_passable(
                obstacle.left_status,
                obstacle.center_status,
                obstacle.right_status,
            ):
                self._set_state("AVOID_ESCAPE", now)
                return
            if now - self.state_since >= self.stop_hold_seconds:
                self._avoid_steps += 1
                self._start_turn(
                    now,
                    self._avoid_direction,
                    self.turn_step_rad,
                    odom_yaw,
                    "AVOID_TURN",
                )
            return

        if self.state == "AVOID_TURN":
            if self._turn_complete(now, odom_yaw, odom_time):
                self._set_state("AVOID_VERIFY", now)
                self._hard_stop("avoid_turn_step_complete")
            else:
                self._publish_target(0.0, self._turn_direction * self.turn_speed, now)
            return

        if self.state == "AVOID_VERIFY":
            self._hard_stop("avoid_verify")
            if now - self.state_since < self.verify_hold_seconds:
                return
            if corridor_is_passable(
                obstacle.left_status,
                obstacle.center_status,
                obstacle.right_status,
            ):
                self._set_state("AVOID_ESCAPE", now)
                return
            if self._avoid_steps >= self.max_avoid_steps:
                self._set_state("BLOCKED", now)
                self._hard_stop("avoidance_exhausted")
                return
            self._avoid_direction = choose_turn_direction(
                obstacle.left_distance,
                obstacle.right_distance,
                obstacle.left_status,
                obstacle.right_status,
                self._avoid_direction,
            )
            self._last_turn_direction = self._avoid_direction
            self._set_state("AVOID_STOP", now)
            return

        if self.state == "AVOID_ESCAPE":
            if not corridor_is_passable(
                obstacle.left_status,
                obstacle.center_status,
                obstacle.right_status,
            ):
                self._begin_avoidance(now, obstacle)
                return
            linear, angular, next_state = self._patrol_command(obstacle)
            linear = min(linear, self.avoid_escape_speed)
            if now - self.state_since >= self.avoid_escape_seconds:
                self._set_state(next_state, now)
                self._next_random_turn_time = self._schedule_random_turn(now)
            self._publish_target(linear, angular, now)
            return

        if self.state == "BLOCKED":
            self._hard_stop("blocked")
            if corridor_is_passable(
                obstacle.left_status,
                obstacle.center_status,
                obstacle.right_status,
            ):
                self._set_state("AVOID_ESCAPE", now)

    def _start_turn(
        self,
        now: float,
        direction: int,
        angle: float,
        odom_yaw: Optional[float],
        state: str,
    ) -> None:
        self._turn_direction = 1 if direction >= 0 else -1
        self._turn_angle = float(angle)
        self._turn_start_yaw = odom_yaw
        self._turn_start_time = now
        self._set_state(state, now)

    def _turn_complete(
        self, now: float, odom_yaw: Optional[float], odom_time: float
    ) -> bool:
        if (
            self._turn_start_yaw is not None
            and odom_yaw is not None
            and now - odom_time <= self.odom_timeout
        ):
            turned = abs(wrap_angle(odom_yaw - self._turn_start_yaw))
            if turned >= 0.92 * self._turn_angle:
                return True
        expected = self._turn_angle / max(0.05, self.turn_speed)
        return now - self._turn_start_time >= expected * self.turn_timeout_scale

    def _schedule_random_turn(self, now: float) -> float:
        if not self.random_turn_enabled:
            return float("inf")
        return now + self._random.uniform(
            self.random_turn_min_interval, self.random_turn_max_interval
        )

    def _set_state(self, state: str, now: float) -> None:
        if self.state != state:
            self.state = state
            self.state_since = now

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
        if abs(self._current_linear) > 1.0e-4 or abs(self._current_angular) > 1.0e-4:
            self._last_stop_reason = "-"
        self._publish_command(self._current_linear, self._current_angular)

    def _publish_command(self, linear: float, angular: float) -> None:
        message = Twist()
        message.linear.x = float(linear)
        message.angular.z = float(angular)
        self.cmd_vel_pub.publish(message)

    def _hard_stop(self, reason: str) -> None:
        self._current_linear = 0.0
        self._current_angular = 0.0
        self._last_command_time = time.monotonic()
        self._last_stop_reason = reason
        try:
            self._publish_command(0.0, 0.0)
        except Exception as error:
            if not self._closing:
                self.get_logger().error(f"停车指令发布失败: {error}")

    @staticmethod
    def _status_name(status: int) -> str:
        return {
            ObstacleStatus.CLEAR: "CLEAR",
            ObstacleStatus.CAUTION: "CAUTION",
            ObstacleStatus.STOP: "STOP",
        }.get(int(status), "UNKNOWN")

    def _log_status(
        self,
        now: float,
        observation: Optional[PersonObservation],
        obstacle: Optional[ObstacleStatus],
    ) -> None:
        if now - self._last_log_time < self.status_log_period:
            return
        self._last_log_time = now
        person_age = 999.0 if observation is None else now - observation.detected_at
        obstacle_name = "NONE" if obstacle is None else self._status_name(obstacle.status)
        self.get_logger().info(
            f"state={self.state} obstacle={obstacle_name} person_age={person_age:.2f}s "
            f"cmd=({self._current_linear:.2f},{self._current_angular:.2f}) "
            f"stop={self._last_stop_reason}"
        )

    def _display_loop(self) -> None:
        if not self.show_debug:
            return
        with self._data_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            observation = self._observation
            person_locked = self._person_locked
            obstacle = self._obstacle
            inference_ms = self._inference_ms
        if frame is None:
            return

        now = time.monotonic()
        if observation is not None and now - observation.detected_at <= self.person_lost_timeout:
            x, y, width, height = observation.bbox
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 3)
            cv2.circle(
                frame,
                (int(x + 0.5 * width), int(y + 0.5 * height)),
                6,
                (0, 0, 255),
                -1,
            )
            cv2.putText(
                frame,
                f"PERSON {observation.confidence:.2f}",
                (x, max(24, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

        obstacle_name = "NONE" if obstacle is None else self._status_name(obstacle.status)
        lines = [
            f"STATE: {self.state}",
            f"MOTION: {'ENABLED' if self.enable_motion else 'STOPPED'}",
            f"OBSTACLE: {obstacle_name}",
            f"CMD vx={self._current_linear:.2f} wz={self._current_angular:.2f}",
            f"PERSON: {'LOCKED' if person_locked else 'SEARCH'}  DNN {inference_ms:.0f} ms",
            "S:Start  SPACE:E-Stop  Q:Quit",
        ]
        if obstacle is not None:
            lines.insert(
                3,
                f"L/C/R {obstacle.left_distance:.2f}/{obstacle.center_distance:.2f}/{obstacle.right_distance:.2f}m",
            )
        for index, line in enumerate(lines):
            color = (0, 255, 0) if index == 0 else (255, 255, 255)
            cv2.putText(
                frame,
                line,
                (12, 28 + 27 * index),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
                cv2.LINE_AA,
            )
        try:
            cv2.imshow(self.debug_window_name, frame)
            key = cv2.waitKey(1) & 0xFF
        except cv2.error as error:
            self.get_logger().error(f"调试窗口不可用，自动关闭显示: {error}")
            self.show_debug = False
            return
        if key in (ord("s"), ord("S")):
            self.enable_motion = True
            self._set_state("WAIT_SENSORS", now)
        elif key == 32:
            self.enable_motion = False
            self._set_state("IDLE", now)
            self._hard_stop("keyboard_emergency_stop")
        elif key in (ord("q"), ord("Q"), 27):
            self.enable_motion = False
            self._hard_stop("keyboard_quit")
            rclpy.shutdown()

    def cleanup(self) -> None:
        self._closing = True
        self.enable_motion = False
        self._stop_worker.set()
        for _ in range(3):
            try:
                self._publish_command(0.0, 0.0)
                time.sleep(0.03)
            except Exception:
                pass
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        try:
            cv2.destroyAllWindows()
        except (cv2.error, KeyboardInterrupt):
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PersonAwarePatrol()
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
