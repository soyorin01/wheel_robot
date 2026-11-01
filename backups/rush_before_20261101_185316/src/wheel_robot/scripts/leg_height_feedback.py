#!/usr/bin/env python3
"""Publish actual leg-height feedback estimated from joint angles."""

from __future__ import annotations

import math
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


LEFT_FRONT = "left_front_joint_link"
LEFT_BACK = "left_back_joint_link"
RIGHT_FRONT = "right_front_joint_link"
RIGHT_BACK = "right_back_joint_link"


class LegHeightFeedback(Node):
    def __init__(self) -> None:
        super().__init__("leg_height_feedback")

        self.declare_parameter("thigh_m", 0.15)
        self.declare_parameter("shank_m", 0.24)
        self.declare_parameter("motor_spacing_m", 0.085)
        self.declare_parameter("foot_x_offset_m", 0.0425)
        self.declare_parameter("publish_invalid_as_nan", True)

        self.thigh = float(self.get_parameter("thigh_m").value)
        self.shank = float(self.get_parameter("shank_m").value)
        self.motor_spacing = float(self.get_parameter("motor_spacing_m").value)
        self.foot_x_offset = float(self.get_parameter("foot_x_offset_m").value)
        self.publish_invalid_as_nan = bool(
            self.get_parameter("publish_invalid_as_nan").value
        )

        self.publisher = self.create_publisher(JointState, "/leg_height_feedback", 10)
        self.create_subscription(
            JointState, "/robot_joint_states", self._on_joint_state, 10
        )
        self.get_logger().info(
            "腿高反馈节点已启动: /robot_joint_states -> /leg_height_feedback"
        )

    def _on_joint_state(self, message: JointState) -> None:
        positions = self._position_by_name(message)
        required = [LEFT_FRONT, LEFT_BACK, RIGHT_FRONT, RIGHT_BACK]
        missing = [name for name in required if name not in positions]
        if missing:
            self.get_logger().warn(
                f"/robot_joint_states 缺少关节: {', '.join(missing)}",
                throttle_duration_sec=2.0,
            )
            return

        left_height = self._leg_height(
            alpha_deg=math.degrees(positions[LEFT_BACK]),
            beta_deg=math.degrees(positions[LEFT_FRONT]),
        )
        right_height = self._leg_height(
            alpha_deg=math.degrees(positions[RIGHT_BACK]),
            beta_deg=math.degrees(positions[RIGHT_FRONT]),
        )

        heights = [left_height, right_height]
        valid_heights = [height for height in heights if math.isfinite(height)]
        if valid_heights:
            average_height = sum(valid_heights) / len(valid_heights)
        else:
            average_height = math.nan
            if not self.publish_invalid_as_nan:
                self.get_logger().warn("左右腿高度均无法解算", throttle_duration_sec=2.0)
                return

        output = JointState()
        output.header = message.header
        output.name = ["left_leg_height", "right_leg_height", "average_leg_height"]
        output.position = [left_height, right_height, average_height]
        self.publisher.publish(output)

    @staticmethod
    def _position_by_name(message: JointState) -> dict[str, float]:
        result: dict[str, float] = {}
        count = min(len(message.name), len(message.position))
        for index in range(count):
            result[message.name[index]] = float(message.position[index])
        return result

    def _leg_height(self, alpha_deg: float, beta_deg: float) -> float:
        point = self._forward_kinematics(alpha_deg, beta_deg)
        if point is None:
            return math.nan
        return point[1]

    def _forward_kinematics(
        self, alpha_deg: float, beta_deg: float
    ) -> tuple[float, float] | None:
        l1 = self.thigh
        l2 = self.shank
        l3 = self.shank
        l4 = self.thigh
        l5 = self.motor_spacing

        alpha = math.radians(180.0 - alpha_deg)
        beta = math.radians(beta_deg)

        point_a = (l1 * math.cos(alpha), l1 * math.sin(alpha))
        point_c = (l5 + l4 * math.cos(beta), l4 * math.sin(beta))

        ax_minus_cx = point_a[0] - point_c[0]
        ay_minus_cy = point_a[1] - point_c[1]
        a_value = 2.0 * ax_minus_cx * l2
        b_value = 2.0 * ay_minus_cy * l2
        link_distance = math.hypot(ax_minus_cx, ay_minus_cy)
        c_value = l3 * l3 - l2 * l2 - link_distance * link_distance
        denominator = a_value + c_value
        discriminant = b_value * b_value + a_value * a_value - c_value * c_value

        if discriminant < 0.0 or abs(denominator) < 1e-9:
            self.get_logger().warn(
                (
                    "腿部正解超出结构范围: "
                    f"alpha={alpha_deg:.2f}deg beta={beta_deg:.2f}deg"
                ),
                throttle_duration_sec=2.0,
            )
            return None

        root = math.sqrt(discriminant)
        theta1 = 2.0 * math.atan((b_value + root) / denominator)
        theta2 = 2.0 * math.atan((b_value - root) / denominator)
        theta1 = theta1 if theta1 >= 0.0 else theta1 + 2.0 * math.pi
        theta2 = theta2 if theta2 >= 0.0 else theta2 + 2.0 * math.pi
        selected_theta = theta2 if theta1 >= math.pi / 2.0 else theta1

        foot_x = point_a[0] + l2 * math.cos(selected_theta) - self.foot_x_offset
        foot_y = point_a[1] + l2 * math.sin(selected_theta)
        return foot_x, foot_y


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LegHeightFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
