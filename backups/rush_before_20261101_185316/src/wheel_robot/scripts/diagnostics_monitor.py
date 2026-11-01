#!/usr/bin/env python3
"""Terminal dashboard for the ESP32 structured self-test status."""

import os
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from rclpy.node import Node


LEVEL_TEXT = {
    DiagnosticStatus.OK: "正常",
    DiagnosticStatus.WARN: "等待/警告",
    DiagnosticStatus.ERROR: "故障",
    DiagnosticStatus.STALE: "无数据",
}


class DiagnosticsMonitor(Node):
    def __init__(self) -> None:
        super().__init__("diagnostics_monitor")
        self._statuses = {}
        self._last_update = 0.0
        self.create_subscription(
            DiagnosticArray, "/robot_diagnostics", self._on_diagnostics, 10
        )
        self.create_timer(1.0, self._on_timer)
        self._render()

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        self._statuses = {status.name: status for status in message.status}
        self._last_update = time.monotonic()
        self._render()

    def _on_timer(self) -> None:
        if not self._last_update or time.monotonic() - self._last_update > 2.0:
            self._render()

    @staticmethod
    def _values(status: DiagnosticStatus):
        return {item.key: item.value for item in status.values}

    @staticmethod
    def _line(label: str, status: DiagnosticStatus | None) -> str:
        if status is None:
            return f"{label:<12} [无数据]"
        level = LEVEL_TEXT.get(status.level, f"未知({status.level})")
        return f"{label:<12} [{level}] {status.message}"

    def _render(self) -> None:
        os.system("clear")
        print("轮腿机器人开机自检")
        print("=" * 56)

        age = time.monotonic() - self._last_update if self._last_update else None
        if age is None:
            print("Type-C通信   [等待] 正在等待 ESP32 自检状态包…")
        elif age > 2.0:
            print(f"Type-C通信   [故障] 超过 {age:.1f} 秒未收到状态包")
        else:
            print(f"Type-C通信   [正常] 状态包更新于 {age:.1f} 秒前")

        overall = self._statuses.get("wheel_robot/self_test")
        remote = self._statuses.get("wheel_robot/remote")
        battery = self._statuses.get("wheel_robot/battery")
        can_imu = self._statuses.get("wheel_robot/can_imu")

        print(self._line("整机自检", overall))
        print(self._line("遥控器", remote))
        print(self._line("电池", battery))
        print(self._line("CAN / IMU", can_imu))

        if overall is not None:
            values = self._values(overall)
            print(
                f"  启动阶段={values.get('boot_stage', '?')}  "
                f"故障位={values.get('fault_bits', '?')}  "
                f"运行时间={values.get('uptime_ms', '?')} ms"
            )
        if battery is not None:
            print(f"  电压={self._values(battery).get('voltage_v', '?')} V")
        if remote is not None:
            values = self._values(remote)
            channels = " ".join(values.get(f"channel_{i}", "?") for i in range(1, 11))
            print(
                f"  SBUS周期={values.get('frame_period_ms', '?')} ms  "
                f"帧年龄={values.get('frame_age_ms', '?')} ms  "
                f"复位掩码={values.get('reset_mask', '?')}/63"
            )
            print(f"  通道1~10: {channels}")
        if can_imu is not None:
            values = self._values(can_imu)
            hold_text = "已启用" if values.get("yaw_hold_active") == "1" else "未启用"
            print(
                f"  航向保持={hold_text}  "
                f"Z轴零偏={values.get('yaw_bias_rad_s', '?')} rad/s  "
                f"校正角速度={values.get('corrected_yaw_rate_rad_s', '?')} rad/s"
            )
            print(f"  航向误差={values.get('yaw_hold_error_deg', '?')} deg")

        print("-" * 56)
        for motor_id in range(1, 7):
            status = self._statuses.get(f"wheel_robot/motor_{motor_id}")
            if status is None:
                print(f"电机 {motor_id}       [无数据]")
                continue
            values = self._values(status)
            level = LEVEL_TEXT.get(status.level, str(status.level))
            print(
                f"电机 {motor_id}       [{level}] {status.message}  "
                f"seen={values.get('seen', '?')} fresh={values.get('fresh', '?')} "
                f"enabled={values.get('enabled', '?')} error={values.get('error_code', '?')}"
            )

        print("=" * 56)
        print("Ctrl+C 退出显示；本程序只读取状态，不会发送运动指令。")


def main() -> None:
    rclpy.init()
    node = DiagnosticsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
