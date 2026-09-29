#!/usr/bin/env python3

import sys
import time

import rclpy
from rclpy.node import Node

from motor_temperature_pkg.msg import MotorTemperatures


class MotorTemperatureMonitor(Node):
    def __init__(self):
        super().__init__("motor_temperature_monitor")
        self.declare_parameter("refresh_period", 0.5)
        self.refresh_period = float(self.get_parameter("refresh_period").value)
        self.last_print_time = 0.0
        self.subscription = self.create_subscription(
            MotorTemperatures,
            "/motor_temperatures",
            self.temperature_callback,
            10,
        )

    def temperature_callback(self, msg: MotorTemperatures) -> None:
        now = time.monotonic()
        if now - self.last_print_time < self.refresh_period:
            return
        self.last_print_time = now

        overheated = set(int(index) for index in msg.overheated_index)
        rows = []
        for index, (name, temp) in enumerate(zip(msg.name, msg.temperature_c), start=1):
            status = "超温" if (index - 1) in overheated else "正常"
            rows.append(
                f"{index:>1}号电机  {name:<24}  {temp:>5.1f} C  {status}"
            )

        max_temp = max(msg.temperature_c) if msg.temperature_c else 0.0
        min_temp = min(msg.temperature_c) if msg.temperature_c else 0.0
        header = (
            "电机温度监控 /motor_temperatures\n"
            f"刷新周期: {self.refresh_period:.1f}s  "
            f"报警阈值: {msg.warning_threshold_c:.1f} C  "
            f"最高: {max_temp:.1f} C  最低: {min_temp:.1f} C\n"
            "---------------------------------------------------------------"
        )

        output = "\033[2J\033[H" + header + "\n" + "\n".join(rows) + "\n"
        if msg.overheated:
            hot = ", ".join(f"{index + 1}号" for index in overheated)
            output += f"\n警告: {hot} 电机超过温度阈值\n"
        sys.stdout.write(output)
        sys.stdout.flush()


def main(args=None):
    rclpy.init(args=args)
    node = MotorTemperatureMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
