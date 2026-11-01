#!/usr/bin/env python3
"""在车载 HDMI 屏幕显示 Astra 障碍物距离图。"""

import os

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


def _image_to_bgr(message: Image) -> np.ndarray:
    encoding = message.encoding.lower()
    if encoding not in ("bgr8", "rgb8"):
        raise ValueError(f"不支持的图像编码 {message.encoding}")
    frame = np.frombuffer(message.data, dtype=np.uint8)
    frame = frame.reshape((message.height, message.width, 3))
    if encoding == "rgb8":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    # 消息在本次回调结束前一直有效；imshow 后立即 waitKey 完成绘制，
    # 无需再复制一张约 0.9 MB 的 800×480 BGR 图。
    return frame


class DepthPreview(Node):
    """打开 /camera/depth/obstacle_view 的 OpenCV 窗口。"""

    def __init__(self) -> None:
        super().__init__("depth_preview")
        self.declare_parameter("show_display", True)
        self.declare_parameter("fullscreen", True)
        self.declare_parameter("topic", "/camera/depth/obstacle_view")
        self.declare_parameter(
            "window_name", "Astra depth distance (Q/Esc to close)"
        )

        self.show_display = bool(self.get_parameter("show_display").value)
        self.fullscreen = bool(self.get_parameter("fullscreen").value)
        self.topic = str(self.get_parameter("topic").value)
        self.window_name = str(self.get_parameter("window_name").value)
        self._window_ready = False

        if not self.show_display:
            self.get_logger().info("show_display=false，不打开测距窗口")
            return

        display = os.environ.get("DISPLAY", "")
        if not display:
            raise RuntimeError(
                "show_display=true 但 DISPLAY 为空，请检查本地 HDMI 桌面是否已启动"
            )
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            if self.fullscreen:
                cv2.setWindowProperty(
                    self.window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
        except cv2.error as error:
            raise RuntimeError(
                f"无法在 DISPLAY={display} 创建窗口: {error}"
            ) from error
        self._window_ready = True
        latest_frame_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, self.topic, self._on_image, latest_frame_qos)
        self.get_logger().info(
            f"测距窗口已打开 DISPLAY={display} topic={self.topic}；"
            f"窗口应显示在车载 HDMI 屏幕，fullscreen={self.fullscreen}"
        )

    def _on_image(self, message: Image) -> None:
        if not self._window_ready:
            return
        try:
            frame = _image_to_bgr(message)
        except ValueError as error:
            self.get_logger().error(str(error))
            return
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q"), ord("Q")):
            self._window_ready = False
            cv2.destroyWindow(self.window_name)
            self.get_logger().info("已关闭测距窗口，测距节点继续运行")

    def close(self) -> None:
        if self._window_ready:
            cv2.destroyWindow(self.window_name)
            self._window_ready = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthPreview()
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
