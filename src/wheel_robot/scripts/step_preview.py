#!/usr/bin/env python3
"""HDMI preview for low-step detection and approach-test state."""

from __future__ import annotations

import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Image
from wheel_robot.msg import StepApproachStatus, StepDetection


def image_to_bgr(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"不支持的图像编码 {message.encoding}")
    frame = np.frombuffer(message.data, dtype=np.uint8)
    frame = frame.reshape((message.height, message.width, 3))
    if encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame.copy()


def fmt_m(value: float) -> str:
    if not math.isfinite(float(value)):
        return "--"
    return f"{float(value):.2f}m"


def fmt_h(value: float) -> str:
    if not math.isfinite(float(value)):
        return "--"
    return f"{float(value):.3f}m"


class StepPreview(Node):
    def __init__(self) -> None:
        super().__init__("step_preview")

        self.declare_parameter("show_display", True)
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("window_name", "Step approach preview (Q/Esc to close)")
        self.declare_parameter("debug_image_topic", "/step_detector/debug_image")
        self.declare_parameter("detection_topic", "/step_detector/detection")
        self.declare_parameter("approach_status_topic", "/step_approach/status")
        self.declare_parameter("stale_timeout_s", 1.0)
        self.declare_parameter("panel_height_px", 132)
        self.declare_parameter("fallback_width", 640)
        self.declare_parameter("fallback_height", 480)

        self.show_display = bool(self.get_parameter("show_display").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.approach_status_topic = str(self.get_parameter("approach_status_topic").value)
        self.stale_timeout_s = max(0.1, float(self.get_parameter("stale_timeout_s").value))
        self.panel_height_px = max(90, int(self.get_parameter("panel_height_px").value))
        self.fallback_width = max(320, int(self.get_parameter("fallback_width").value))
        self.fallback_height = max(240, int(self.get_parameter("fallback_height").value))

        self.latest_image: np.ndarray | None = None
        self.latest_image_time = 0.0
        self.latest_detection: StepDetection | None = None
        self.latest_detection_time = 0.0
        self.latest_status: StepApproachStatus | None = None
        self.latest_status_time = 0.0
        self._window_ready = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.debug_image_topic, self._on_image, qos)
        self.create_subscription(StepDetection, self.detection_topic, self._on_detection, 10)
        self.create_subscription(
            StepApproachStatus, self.approach_status_topic, self._on_status, 10
        )

        if self.show_display:
            display = os.environ.get("DISPLAY", "")
            if not display:
                raise RuntimeError("show_display=true 但 DISPLAY 为空，请检查 HDMI 桌面")
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            if self.fullscreen:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
            self._window_ready = True
            self.create_timer(1.0 / 20.0, self._draw)
            self.get_logger().info(
                f"台阶预览窗口已打开 DISPLAY={display}, debug={self.debug_image_topic}"
            )
        else:
            self.get_logger().info("show_display=false，不打开 HDMI 窗口")

    def _on_image(self, message: Image) -> None:
        try:
            self.latest_image = image_to_bgr(message)
            self.latest_image_time = time.monotonic()
        except ValueError as error:
            self.get_logger().error(str(error))

    def _on_detection(self, message: StepDetection) -> None:
        self.latest_detection = message
        self.latest_detection_time = time.monotonic()

    def _on_status(self, message: StepApproachStatus) -> None:
        self.latest_status = message
        self.latest_status_time = time.monotonic()

    def _draw(self) -> None:
        if not self._window_ready:
            return
        now = time.monotonic()
        if self.latest_image is None:
            frame = np.zeros((self.fallback_height, self.fallback_width, 3), dtype=np.uint8)
        else:
            frame = self.latest_image.copy()

        self._draw_panel(frame, now)
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            self.close()

    def _draw_panel(self, frame: np.ndarray, now: float) -> None:
        panel_h = min(self.panel_height_px, max(90, frame.shape[0] - 1))
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0), (frame.shape[1], panel_h), (18, 18, 18), -1)
        cv2.addWeighted(overlay, 0.78, frame, 0.22, 0.0, frame)

        detection = self.latest_detection
        detection_fresh = (
            detection is not None and now - self.latest_detection_time <= self.stale_timeout_s
        )
        status = self.latest_status
        status_fresh = status is not None and now - self.latest_status_time <= self.stale_timeout_s

        step_text = "STEP" if detection_fresh and detection.detected else "NO STEP"
        step_color = (0, 0, 255) if step_text == "STEP" else (180, 180, 180)
        cv2.putText(frame, step_text, (18, 38), cv2.FONT_HERSHEY_SIMPLEX, 1.0, step_color, 2, cv2.LINE_AA)

        distance = detection.distance if detection_fresh else float("nan")
        height = detection.height if detection_fresh else float("nan")
        confidence = detection.confidence if detection_fresh else 0.0
        state = status.state if status_fresh else "--"
        target = status.target_travel if status_fresh else float("nan")
        traveled = status.traveled if status_fresh else float("nan")
        remaining = status.remaining if status_fresh else float("nan")

        line1 = "distance %s   height %s   confidence %.2f" % (
            fmt_m(distance),
            fmt_h(height),
            float(confidence),
        )
        line2 = "state %s   target %s   traveled %s   remaining %s" % (
            state,
            fmt_m(target),
            fmt_m(traveled),
            fmt_m(remaining),
        )
        line3 = "green ground   yellow step area   red/white front edge"
        if self.latest_image is None or now - self.latest_image_time > self.stale_timeout_s:
            line3 = "waiting for detector debug image"

        cv2.putText(frame, line1, (18, 72), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 1, cv2.LINE_AA)
        cv2.putText(frame, line2, (18, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (245, 245, 245), 1, cv2.LINE_AA)
        cv2.putText(frame, line3, (18, 124), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (210, 230, 255), 1, cv2.LINE_AA)

    def close(self) -> None:
        if self._window_ready:
            cv2.destroyWindow(self.window_name)
            self._window_ready = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
