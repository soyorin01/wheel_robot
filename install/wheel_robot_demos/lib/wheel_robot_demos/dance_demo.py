#!/usr/bin/env python3
"""Safe scripted dance demo for the wheel-legged robot."""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    import yaml
except ImportError as exc:  # pragma: no cover - depends on system package state
    raise RuntimeError("缺少 python3-yaml，请先安装或用 rosdep 安装依赖") from exc


POSTURE_NAMES = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]
DEFAULT_STATE = {
    "height": 0.25,
    "roll": 0.0,
    "pitch": 0.0,
    "slide": 0.0,
    "yaw": 0.0,
    "linear": 0.0,
}
DEFAULT_LIMITS = {
    "height_min": 0.20,
    "height_max": 0.28,
    "roll_abs_deg": 6.0,
    "pitch_abs_deg": 5.0,
    "slide_abs_m": 0.03,
    "yaw_abs_rad_s": 0.25,
    "linear_abs_m_s": 0.0,
}
REMOTE_FAULT_BITS = (1 << 7) | (1 << 8)


@dataclass
class DanceStep:
    name: str
    duration: float
    hold: float
    target: dict[str, float]
    jump: bool = False


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def smoothstep(x: float) -> float:
    x = clamp(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def value_map(status: DiagnosticStatus | None) -> dict[str, str]:
    if status is None:
        return {}
    return {item.key: item.value for item in status.values}


def as_int(values: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(values.get(key, str(default)))
    except ValueError:
        return default


def parameter_bool(value: Any) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def describe_fault_bits(fault_bits: int) -> str:
    names = [
        (1 << 0, "CAN初始化故障"),
        (1 << 1, "电机停止故障"),
        (1 << 2, "电机零点设置故障"),
        (1 << 3, "电机方向设置故障"),
        (1 << 4, "电机模式设置故障"),
        (1 << 5, "电机使能故障"),
        (1 << 6, "IMU故障"),
        (1 << 7, "SBUS遥控信号故障"),
        (1 << 8, "遥控器通道未复位"),
        (1 << 9, "电池电压过低"),
        (1 << 10, "电机离线"),
        (1 << 11, "电机报错"),
    ]
    active = [name for bit, name in names if fault_bits & bit]
    if not active:
        return "无故障"
    return "、".join(active)


class DanceDemo(Node):
    def __init__(self) -> None:
        super().__init__("wheel_robot_dance_demo")

        self.declare_parameter("sequence_file", "")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("repeat", 0)
        self.declare_parameter("with_jump", False)
        self.declare_parameter("wait_for_safety", True)
        self.declare_parameter("safety_timeout_sec", 5.0)
        self.declare_parameter("max_diag_age_sec", 2.0)

        self.dry_run = parameter_bool(self.get_parameter("dry_run").value)
        self.speed_scale = max(0.1, float(self.get_parameter("speed_scale").value))
        self.repeat_override = int(self.get_parameter("repeat").value)
        self.with_jump = parameter_bool(self.get_parameter("with_jump").value)
        self.wait_for_safety = parameter_bool(
            self.get_parameter("wait_for_safety").value
        )
        self.safety_timeout_sec = float(self.get_parameter("safety_timeout_sec").value)
        self.max_diag_age_sec = float(self.get_parameter("max_diag_age_sec").value)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self.jump_pub = self.create_publisher(Bool, "/cmd_jump", 10)
        self.create_subscription(
            DiagnosticArray, "/robot_diagnostics", self._on_diagnostics, 10
        )

        self._diagnostics: dict[str, DiagnosticStatus] = {}
        self._diagnostics_time = 0.0
        self._last_progress = 0.0

        config = self._load_config()
        self.rate_hz = max(10.0, float(config.get("rate_hz", 40.0)))
        self.period = 1.0 / self.rate_hz
        self.defaults = DEFAULT_STATE | self._float_dict(config.get("defaults", {}))
        self.limits = DEFAULT_LIMITS | self._float_dict(config.get("limits", {}))
        yaml_repeat = max(1, int(config.get("repeat", 1)))
        self.repeat_count = self.repeat_override if self.repeat_override > 0 else yaml_repeat
        self.steps = self._parse_steps(config.get("sequence", []))
        self.state = dict(self.defaults)

        if not self.steps:
            raise RuntimeError("舞蹈序列为空，请检查 dance_sequence.yaml")

        mode = "干跑，不发布运动指令" if self.dry_run else "真实发布运动指令"
        self.get_logger().info(
            f"跳舞演示已加载: {len(self.steps)} 个动作, repeat={self.repeat_count}, "
            f"speed_scale={self.speed_scale:.2f}, {mode}"
        )

    def _load_config(self) -> dict[str, Any]:
        sequence_file = str(self.get_parameter("sequence_file").value).strip()
        if not sequence_file:
            sequence_file = str(
                Path(get_package_share_directory("wheel_robot_demos"))
                / "config"
                / "dance_sequence.yaml"
            )

        path = Path(sequence_file).expanduser()
        if not path.exists():
            raise FileNotFoundError(f"找不到舞蹈序列文件: {path}")

        with path.open("r", encoding="utf-8") as file:
            config = yaml.safe_load(file) or {}
        if not isinstance(config, dict):
            raise RuntimeError(f"舞蹈序列格式错误: {path}")
        self.get_logger().info(f"使用舞蹈序列: {path}")
        return config

    @staticmethod
    def _float_dict(data: Any) -> dict[str, float]:
        if not isinstance(data, dict):
            return {}
        result: dict[str, float] = {}
        for key, value in data.items():
            if key in DEFAULT_STATE or key in DEFAULT_LIMITS:
                result[str(key)] = float(value)
        return result

    def _parse_steps(self, raw_steps: Any) -> list[DanceStep]:
        if not isinstance(raw_steps, list):
            raise RuntimeError("sequence 必须是列表")

        steps: list[DanceStep] = []
        for index, raw in enumerate(raw_steps, start=1):
            if not isinstance(raw, dict):
                raise RuntimeError(f"第 {index} 个动作不是字典")
            target = self._float_dict(raw.get("target", {}))
            steps.append(
                DanceStep(
                    name=str(raw.get("name", f"step_{index}")),
                    duration=max(0.05, float(raw.get("duration", 1.0))),
                    hold=max(0.0, float(raw.get("hold", 0.0))),
                    target=target,
                    jump=bool(raw.get("jump", False)),
                )
            )
        return steps

    def _on_diagnostics(self, message: DiagnosticArray) -> None:
        self._diagnostics = {status.name: status for status in message.status}
        self._diagnostics_time = time.monotonic()

    def _safety_error(self) -> str | None:
        now = time.monotonic()
        if self.cmd_vel_pub.get_subscription_count() == 0:
            return "/cmd_vel 没有订阅者，请先启动 base.launch.py"
        if self.posture_pub.get_subscription_count() == 0:
            return "/cmd_posture 没有订阅者，请先启动 base.launch.py"

        if self.wait_for_safety:
            if not self._diagnostics_time:
                return "还没有收到 /robot_diagnostics"
            if now - self._diagnostics_time > self.max_diag_age_sec:
                return "/robot_diagnostics 超时"

            overall = self._diagnostics.get("wheel_robot/self_test")
            if overall is None:
                return "缺少整机自检状态 wheel_robot/self_test"

            overall_values = value_map(overall)
            boot_stage = as_int(overall_values, "boot_stage", 0)
            fault_bits = as_int(overall_values, "fault_bits", 0)
            blocking_fault_bits = fault_bits & ~REMOTE_FAULT_BITS
            robot_mode = as_int(overall_values, "robot_mode", 0)
            if boot_stage != 5:
                return f"启动阶段 boot_stage={boot_stage}，尚未初始化完成"
            if blocking_fault_bits != 0:
                return (
                    f"fault_bits={fault_bits}，"
                    f"{describe_fault_bits(blocking_fault_bits)}"
                )
            if overall.level == DiagnosticStatus.ERROR:
                remote_only_fault = fault_bits != 0 and blocking_fault_bits == 0
                if not remote_only_fault:
                    return f"整机自检故障: {overall.message}"
            if robot_mode != 0:
                return f"robot_mode={robot_mode}，当前不是移动底盘模式"

        return None

    def wait_until_safe(self) -> bool:
        if self.dry_run:
            self.get_logger().info("干跑模式跳过安全闭锁，只打印动作序列")
            return True

        deadline = time.monotonic() + self.safety_timeout_sec
        last_error = ""
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            error = self._safety_error()
            if error is None:
                self.get_logger().info("安全检查通过，开始舞蹈")
                return True
            if error != last_error:
                self.get_logger().warn(f"等待安全条件: {error}")
                last_error = error

        self.get_logger().error(f"安全检查未通过: {last_error or '未知原因'}")
        return False

    def _limited_state(self, state: dict[str, float]) -> dict[str, float]:
        limited = dict(state)
        limited["height"] = clamp(
            limited.get("height", self.defaults["height"]),
            self.limits["height_min"],
            self.limits["height_max"],
        )
        limited["roll"] = clamp(
            limited.get("roll", 0.0),
            -self.limits["roll_abs_deg"],
            self.limits["roll_abs_deg"],
        )
        limited["pitch"] = clamp(
            limited.get("pitch", 0.0),
            -self.limits["pitch_abs_deg"],
            self.limits["pitch_abs_deg"],
        )
        limited["slide"] = clamp(
            limited.get("slide", 0.0),
            -self.limits["slide_abs_m"],
            self.limits["slide_abs_m"],
        )
        limited["yaw"] = clamp(
            limited.get("yaw", 0.0),
            -self.limits["yaw_abs_rad_s"],
            self.limits["yaw_abs_rad_s"],
        )
        limited["linear"] = clamp(
            limited.get("linear", 0.0),
            -self.limits["linear_abs_m_s"],
            self.limits["linear_abs_m_s"],
        )
        return limited

    def _publish_state(self, state: dict[str, float], jump: bool = False) -> None:
        state = self._limited_state(state)
        if self.dry_run:
            return

        velocity = Twist()
        velocity.linear.x = state["linear"]
        velocity.angular.z = state["yaw"]
        self.cmd_vel_pub.publish(velocity)

        posture = JointState()
        posture.header.stamp = self.get_clock().now().to_msg()
        posture.name = POSTURE_NAMES
        posture.position = [
            state["height"],
            state["roll"],
            state["pitch"],
            state["slide"],
        ]
        self.posture_pub.publish(posture)

        jump_msg = Bool()
        jump_msg.data = bool(jump)
        self.jump_pub.publish(jump_msg)

    def _log_progress(self, name: str, state: dict[str, float]) -> None:
        now = time.monotonic()
        if now - self._last_progress < 0.4:
            return
        self._last_progress = now
        self.get_logger().info(
            f"{name}: h={state['height']:.3f}m roll={state['roll']:+.1f}deg "
            f"pitch={state['pitch']:+.1f}deg slide={state['slide']:+.3f}m "
            f"yaw={state['yaw']:+.2f}rad/s"
        )

    def _send_jump_pulse(self) -> None:
        if self.dry_run or not self.with_jump:
            return
        stationary = dict(self.state)
        stationary["linear"] = 0.0
        stationary["yaw"] = 0.0
        for _ in range(max(1, int(0.3 * self.rate_hz))):
            self._publish_state(stationary, jump=False)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)
        for _ in range(max(1, int(0.2 * self.rate_hz))):
            self._publish_state(stationary, jump=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)
        self._publish_state(stationary, jump=False)

    def _run_step(self, step: DanceStep) -> bool:
        target = self._limited_state(dict(self.state) | step.target)
        start = dict(self.state)
        duration = step.duration / self.speed_scale
        hold = step.hold / self.speed_scale
        step_start = time.monotonic()

        self.get_logger().info(f"动作: {step.name}")
        if step.jump and self.with_jump:
            self._send_jump_pulse()

        while rclpy.ok():
            elapsed = time.monotonic() - step_start
            if elapsed >= duration:
                break
            alpha = smoothstep(elapsed / duration)
            current = {
                key: start.get(key, 0.0) + (target.get(key, 0.0) - start.get(key, 0.0)) * alpha
                for key in DEFAULT_STATE
            }
            if not self.dry_run:
                error = self._safety_error()
                if error is not None:
                    self.get_logger().error(f"舞蹈中止: {error}")
                    return False
            self._publish_state(current)
            self._log_progress(step.name, current)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)

        self.state = target
        hold_end = time.monotonic() + hold
        while rclpy.ok() and time.monotonic() < hold_end:
            if not self.dry_run:
                error = self._safety_error()
                if error is not None:
                    self.get_logger().error(f"舞蹈中止: {error}")
                    return False
            self._publish_state(self.state)
            self._log_progress(step.name, self.state)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(self.period)
        return True

    def publish_final_stop(self) -> None:
        self.state = self._limited_state(dict(self.defaults))
        self.state["yaw"] = 0.0
        self.state["linear"] = 0.0
        for _ in range(max(10, int(0.4 * self.rate_hz))):
            try:
                self._publish_state(self.state, jump=False)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(self.period)
            except Exception as exc:  # noqa: BLE001 - keep shutdown robust
                print(f"退出时发送停止指令失败: {exc}", file=sys.stderr)
                break

    def run(self) -> bool:
        if not self.wait_until_safe():
            return False

        for loop_index in range(self.repeat_count):
            self.get_logger().info(f"开始第 {loop_index + 1}/{self.repeat_count} 遍")
            for step in self.steps:
                if not self._run_step(step):
                    return False

        self.get_logger().info("舞蹈完成，正在归零停止")
        return True


def main(args=None) -> None:
    rclpy.init(args=args)
    node: DanceDemo | None = None
    try:
        node = DanceDemo()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - show launch-friendly error
        print(f"跳舞演示启动失败: {exc}", file=sys.stderr)
    finally:
        if node is not None:
            node.publish_final_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
