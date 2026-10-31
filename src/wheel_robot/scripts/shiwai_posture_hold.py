#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""保持 shiwai_lane_fixed.py 的「低头」姿态，但不开摄像头、不发车速。

用途
----
你要的是：先用遥控器把车开到低头姿态，再自己起 camera_recorder.py
录原始视频给我分析。之前那个 shiwai_pitch_probe.py 会自己打开
/dev/video0，摄像头上只能有一个进程，所以两个程序会抢摄像头。

这个程序因此只做一件事：按 shiwai_lane_fixed.py 一模一样的姿态逻辑
持续发 /cmd_posture。

* 不打开摄像头（不开 /dev/video0，不占）
* 不创建 /cmd_vel 发布者（方向盘留给遥控器）
* 姿态数值和时间常数全部 import 自 shiwai_lane_fixed.py，
  和跑赛道时完全一致，不会漂

姿态来源
--------
shiwai_lane_fixed.py：
    _current_pitch 从 0 线性加到 PITCH_DEG(=10°)，速率 PITCH_RATE_DEG(=20°/s)
    _current_height 目标 STAND_HEIGHT(=0.20 m)，速率 HEIGHT_RATE(=0.20 m/s)
    roll 平地固定 0（BRIDGE_ROLL_DEG=0）
    _publish_posture() 的裁剪范围：高度 0.14~0.36、roll ±15、pitch ±35
    发布频率 control_rate = 15 Hz
注意 STAND_HEIGHT 会被 _apply_field_tuning() 强制覆盖 yaml 的 stand_height，
所以实际跑起来是 0.20，不是 yaml 里的 0.25。

用法
----
终端 1（一直开着，保持低头）：

    ros2 run wheel_robot shiwai_posture_hold.py

终端 2（录原始视频）：

    ros2 run wheel_robot camera_recorder.py

然后用遥控器开直线。开完 Ctrl-C 停掉 recorder，视频在
~/wheel_robot/recordings/

临时改姿态（一般不用）：

    ros2 run wheel_robot shiwai_posture_hold.py --ros-args \
        -p pitch_deg:=10.0 -p height:=0.2
"""

from __future__ import annotations

import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 复用赛场文件的常量与限幅，保证姿态和跑赛道时一模一样。
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import shiwai_lane_fixed as base  # noqa: E402

# _publish_posture() 里的裁剪范围
HEIGHT_MIN, HEIGHT_MAX = 0.14, 0.36
ROLL_LIMIT, PITCH_LIMIT = 15.0, 35.0


class PostureHold(Node):
    def __init__(self) -> None:
        super().__init__("shiwai_posture_hold")

        self.declare_parameter("height", float(base.STAND_HEIGHT))
        self.declare_parameter("pitch_deg", float(base.PITCH_DEG))
        self.declare_parameter("roll_deg", float(base.BRIDGE_ROLL_DEG))
        self.declare_parameter("height_rate", float(base.HEIGHT_RATE))
        self.declare_parameter("pitch_rate_deg", float(base.PITCH_RATE_DEG))
        self.declare_parameter("control_rate", 15.0)

        self.target_height = float(self.get_parameter("height").value)
        self.target_pitch = float(self.get_parameter("pitch_deg").value)
        self.target_roll = float(self.get_parameter("roll_deg").value)
        self.height_rate = float(self.get_parameter("height_rate").value)
        self.pitch_rate = float(self.get_parameter("pitch_rate_deg").value)
        self.control_rate = max(1.0, float(self.get_parameter("control_rate").value))

        # 和赛场节点一样：腿高从站姿开始，俯仰从 0 开始平滑压下去。
        self.current_height = float(base.STAND_HEIGHT)
        self.current_pitch = 0.0
        self.current_roll = 0.0

        # ---------------------------------------------------------------
        # 只有这一个发布者。不建 /cmd_vel，不开摄像头。
        # ---------------------------------------------------------------
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)

        self._last = time.monotonic()
        self._started = time.monotonic()
        self._last_print = 0.0
        self._count = 0
        self._done_announced = False
        self._timer = self.create_timer(
            1.0 / self.control_rate, self._tick
        )

        log = self.get_logger()
        log.info("=" * 60)
        log.info("低头姿态保持器启动（不占摄像头，不发车速）")
        log.info(
            f"目标姿态：高度 {self.target_height:.3f} m / "
            f"横滚 {self.target_roll:+.1f}° / 俯仰 {self.target_pitch:+.1f}°"
        )
        log.info(
            f"速率：高度 {self.height_rate:.2f} m/s，"
            f"俯仰 {self.pitch_rate:.0f} °/s，发布 {self.control_rate:.0f} Hz"
        )
        log.info("现在可以在另一个终端启动 camera_recorder.py 录原始视频")
        log.info("=" * 60)

    @staticmethod
    def _approach(current: float, target: float, delta: float) -> float:
        return float(min(max(target, current - delta), current + delta))

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(now - self._last, 0.5))
        self._last = now

        self.current_height = self._approach(
            self.current_height, self.target_height, self.height_rate * dt
        )
        self.current_pitch = self._approach(
            self.current_pitch, self.target_pitch, self.pitch_rate * dt
        )
        self.current_roll = self._approach(
            self.current_roll, self.target_roll, base.ROLL_RATE_DEG * dt
        )

        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["joint_height", "joint_roll", "joint_pitching"]
        message.position = [
            float(min(max(self.current_height, HEIGHT_MIN), HEIGHT_MAX)),
            float(min(max(self.current_roll, -ROLL_LIMIT), ROLL_LIMIT)),
            float(min(max(self.current_pitch, -PITCH_LIMIT), PITCH_LIMIT)),
        ]
        self.posture_pub.publish(message)
        self._count += 1

        settled = (
            abs(self.current_height - self.target_height) < 1e-4
            and abs(self.current_pitch - self.target_pitch) < 1e-3
        )
        if settled and not self._done_announced:
            self._done_announced = True
            self.get_logger().info(
                f"姿态已到位：高度 {self.current_height:.3f} m "
                f"俯仰 {self.current_pitch:+.1f}°（用了 "
                f"{now - self._started:.1f} 秒）"
            )
        if now - self._last_print >= 2.0:
            self._last_print = now
            self.get_logger().info(
                f"保持中  h={self.posture_limit_height():.3f} "
                f"roll={self.current_roll:+.1f} pitch={self.current_pitch:+.1f} "
                f"已发 {self._count} 帧"
            )

    def posture_limit_height(self) -> float:
        return float(min(max(self.current_height, HEIGHT_MIN), HEIGHT_MAX))

    def announce_exit(self) -> None:
        self.get_logger().info(
            "退出：姿态停在最后的值。下位机姿态超时后会自己回默认值，"
            "车可能会动一下，注意周围。"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PostureHold()

    def _handler(signum, _frame) -> None:
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.announce_exit()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("低头姿态保持器已退出")


if __name__ == "__main__":
    main()
