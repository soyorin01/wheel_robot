#!/usr/bin/env python3
"""Approach a confirmed low step and jump at the learned takeoff distance."""

from __future__ import annotations

import csv
import datetime as dt
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray
from wheel_robot.msg import StepApproachStatus, StepDetection


POSTURE_NAMES = ["joint_height", "joint_roll", "joint_pitching", "joint_slide"]


def parameter_bool(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class StepApproachTest(Node):
    STATE_SEARCH = "SEARCH"
    STATE_LOCKED = "LOCKED"
    STATE_APPROACH = "APPROACH"
    STATE_JUMP = "JUMP"
    STATE_LAND = "LAND"
    STATE_STOPPED = "STOPPED"
    STATE_DONE = "DONE"

    def __init__(self) -> None:
        super().__init__("step_approach_test")

        self.declare_parameter("detection_topic", "/step_detector/detection")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("cmd_posture_topic", "/cmd_posture")
        self.declare_parameter("cmd_jump_topic", "/cmd_jump")
        self.declare_parameter("cmd_jump_profile_topic", "/cmd_jump_profile")
        self.declare_parameter("status_topic", "/step_approach/status")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("detection_timeout_s", 0.50)
        self.declare_parameter("odom_timeout_s", 0.50)
        self.declare_parameter("stop_distance_m", 0.20)
        self.declare_parameter("position_tolerance_m", 0.015)
        self.declare_parameter("search_speed_m_s", 0.06)
        self.declare_parameter("approach_speed_m_s", 0.50)
        self.declare_parameter("trigger_distance_source", "commanded")
        self.declare_parameter("max_approach_seconds", 12.0)
        self.declare_parameter("lock_hold_seconds", 0.0)
        self.declare_parameter("jump_enabled", True)
        self.declare_parameter("use_low_jump_profile", True)
        self.declare_parameter("jump_min_height_m", 0.18)
        self.declare_parameter("jump_max_height_m", 0.30)
        self.declare_parameter("stand_height_m", 0.25)
        self.declare_parameter("forward_speed_m_s", 0.50)
        self.declare_parameter("landing_forward_seconds", 2.0)
        self.declare_parameter("landing_speed_scale", 0.60)
        self.declare_parameter("jump_pulse_seconds", 0.20)
        self.declare_parameter("recover_seconds", 3.0)
        self.declare_parameter("terminal_log_period_s", 0.5)
        self.declare_parameter("enable_file_log", True)
        self.declare_parameter("log_dir", "/home/orangepi/wheel_robot/log/step_approach")

        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.cmd_posture_topic = str(self.get_parameter("cmd_posture_topic").value)
        self.cmd_jump_topic = str(self.get_parameter("cmd_jump_topic").value)
        self.cmd_jump_profile_topic = str(
            self.get_parameter("cmd_jump_profile_topic").value
        )
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.control_rate_hz = max(5.0, float(self.get_parameter("control_rate_hz").value))
        self.detection_timeout_s = max(0.05, float(self.get_parameter("detection_timeout_s").value))
        self.odom_timeout_s = max(0.05, float(self.get_parameter("odom_timeout_s").value))
        self.stop_distance_m = max(0.0, float(self.get_parameter("stop_distance_m").value))
        self.position_tolerance_m = max(0.001, float(self.get_parameter("position_tolerance_m").value))
        self.search_speed_m_s = clamp(
            abs(float(self.get_parameter("search_speed_m_s").value)), 0.0, 0.20
        )
        self.approach_speed_m_s = clamp(
            abs(float(self.get_parameter("approach_speed_m_s").value)), 0.0, 0.60
        )
        self.trigger_distance_source = str(
            self.get_parameter("trigger_distance_source").value
        ).strip().lower()
        if self.trigger_distance_source not in ("commanded", "odom"):
            self.get_logger().warn(
                "trigger_distance_source 只能是 commanded/odom，已回退到 commanded"
            )
            self.trigger_distance_source = "commanded"
        self.max_approach_seconds = max(0.0, float(self.get_parameter("max_approach_seconds").value))
        self.lock_hold_seconds = max(0.0, float(self.get_parameter("lock_hold_seconds").value))
        self.jump_enabled = parameter_bool(self.get_parameter("jump_enabled").value)
        self.use_low_jump_profile = parameter_bool(
            self.get_parameter("use_low_jump_profile").value
        )
        self.jump_min_height = clamp(
            float(self.get_parameter("jump_min_height_m").value), 0.16, 0.24
        )
        self.jump_max_height = clamp(
            float(self.get_parameter("jump_max_height_m").value), 0.24, 0.34
        )
        if self.jump_max_height < self.jump_min_height + 0.04:
            self.jump_max_height = self.jump_min_height + 0.04
        self.stand_height = clamp(
            float(self.get_parameter("stand_height_m").value), 0.20, 0.30
        )
        self.jump_forward_speed_m_s = clamp(
            abs(float(self.get_parameter("forward_speed_m_s").value)), 0.0, 0.60
        )
        self.landing_forward_seconds = max(
            0.0, float(self.get_parameter("landing_forward_seconds").value)
        )
        self.landing_speed_scale = clamp(
            float(self.get_parameter("landing_speed_scale").value), 0.0, 1.0
        )
        self.jump_pulse_seconds = max(
            0.01, float(self.get_parameter("jump_pulse_seconds").value)
        )
        self.recover_seconds = max(0.0, float(self.get_parameter("recover_seconds").value))
        self.terminal_log_period_s = max(
            0.1, float(self.get_parameter("terminal_log_period_s").value)
        )
        self.enable_file_log = parameter_bool(self.get_parameter("enable_file_log").value)
        self.log_dir = str(self.get_parameter("log_dir").value)

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.posture_pub = self.create_publisher(JointState, self.cmd_posture_topic, 10)
        self.cmd_jump_pub = self.create_publisher(Bool, self.cmd_jump_topic, 10)
        self.profile_pub = self.create_publisher(
            Float32MultiArray, self.cmd_jump_profile_topic, 10
        )
        self.status_pub = self.create_publisher(StepApproachStatus, self.status_topic, 10)
        self.create_subscription(StepDetection, self.detection_topic, self._on_detection, 10)
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, 10)

        self.state = self.STATE_SEARCH
        self.latest_detection: StepDetection | None = None
        self.latest_detection_time = 0.0
        self.odom_xy: tuple[float, float] | None = None
        self.odom_yaw = 0.0
        self.last_odom_time = 0.0

        self.locked = False
        self.detected_distance = float("nan")
        self.locked_height = float("nan")
        self.locked_confidence = 0.0
        self.lock_x = 0.0
        self.lock_y = 0.0
        self.lock_yaw = 0.0
        self.lock_time = 0.0
        self.approach_start_time = 0.0
        self.last_approach_time = 0.0
        self.commanded_traveled = 0.0
        self.state_deadline = 0.0
        self.target_travel = 0.0
        self.traveled = 0.0
        self.trigger_traveled = 0.0
        self.remaining = 0.0
        self.last_terminal_log_time = 0.0
        self.last_command_speed = 0.0
        self.last_jump_command = False
        self.stop_reason = "等待台阶检测"
        self.run_started_mono = time.monotonic()
        self.log_file = None
        self.log_writer = None
        self.log_path = ""
        self._open_file_log()

        self.create_timer(1.0 / self.control_rate_hz, self._on_timer)
        jump_mode = "自动跳跃" if self.jump_enabled else "到起跳点停车"
        self.get_logger().warn(
            "5cm 台阶接近启动: SEARCH->LOCKED->APPROACH->JUMP->LAND->DONE；"
            f"{jump_mode}, stop_distance={self.stop_distance_m:.2f}m, "
            f"approach_v={self.approach_speed_m_s:.2f}m/s, "
            f"trigger_source={self.trigger_distance_source}, "
            f"jump_v={self.jump_forward_speed_m_s:.2f}m/s, "
            f"profile={self.jump_min_height:.2f}->{self.jump_max_height:.2f}m"
        )

    def _on_detection(self, message: StepDetection) -> None:
        self.latest_detection = message
        self.latest_detection_time = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.odom_xy = (float(position.x), float(position.y))
        self.odom_yaw = yaw_from_odom(message)
        self.last_odom_time = time.monotonic()

    def _on_timer(self) -> None:
        now = time.monotonic()

        if self.state == self.STATE_SEARCH:
            self._handle_search(now)
        elif self.state == self.STATE_LOCKED:
            self._handle_locked(now)
        elif self.state == self.STATE_APPROACH:
            self._handle_approach(now)
        elif self.state == self.STATE_JUMP:
            self._handle_jump(now)
        elif self.state == self.STATE_LAND:
            self._handle_land(now)
        else:
            self._publish_common(0.0, jump=False)

        self._log_runtime_status(now)
        self._write_file_log(now)
        self._publish_status()

    def _handle_search(self, now: float) -> None:
        self._publish_common(self.search_speed_m_s, jump=False)
        detection = self.latest_detection
        if detection is None or now - self.latest_detection_time > self.detection_timeout_s:
            self.stop_reason = "SEARCH 慢速前进，等待台阶检测"
            return
        if not detection.detected:
            self.stop_reason = (
                "SEARCH 慢速前进，候选台阶确认 %d/%d"
                % (detection.confirm_frames, detection.required_confirm_frames)
            )
            return
        if self.odom_xy is None or now - self.last_odom_time > self.odom_timeout_s:
            self._publish_common(0.0, jump=False)
            self.stop_reason = "检测到台阶但 /odom 不新鲜"
            self.get_logger().warn("已确认台阶，但正在等待新鲜 /odom，暂不锁定", throttle_duration_sec=1.0)
            return
        if not math.isfinite(float(detection.distance)):
            self._publish_common(0.0, jump=False)
            self.stop_reason = "检测距离无效"
            self.get_logger().warn("检测距离无效，等待下一次确认", throttle_duration_sec=1.0)
            return

        self.locked = True
        self.detected_distance = float(detection.distance)
        self.locked_height = float(detection.height)
        self.locked_confidence = float(detection.confidence)
        self.lock_x, self.lock_y = self.odom_xy
        self.lock_yaw = self.odom_yaw
        self.lock_time = now
        self.target_travel = max(0.0, self.detected_distance - self.stop_distance_m)
        self.traveled = 0.0
        self.trigger_traveled = 0.0
        self.commanded_traveled = 0.0
        self.remaining = self.target_travel
        self._set_state(self.STATE_LOCKED, "台阶距离已锁定")
        self.get_logger().info(
            "锁定台阶: distance=%.3fm height=%.3fm confidence=%.2f odom=(%.3f, %.3f) yaw=%.3f target=%.3fm"
            % (
                self.detected_distance,
                self.locked_height,
                self.locked_confidence,
                self.lock_x,
                self.lock_y,
                self.lock_yaw,
                self.target_travel,
            )
        )

    def _handle_locked(self, now: float) -> None:
        if self.target_travel <= self.position_tolerance_m:
            self._begin_jump(now)
            return
        if now - self.lock_time < self.lock_hold_seconds:
            self._publish_common(0.0, jump=False)
            self.stop_reason = "LOCKED 短暂停稳"
            return
        self.approach_start_time = now
        self.last_approach_time = now
        self._set_state(self.STATE_APPROACH, "开始按里程计前进到起跳点")
        self.get_logger().info("进入 APPROACH；之后不再因台阶离开视野而取消任务")

    def _handle_approach(self, now: float) -> None:
        if self.odom_xy is None or now - self.last_odom_time > self.odom_timeout_s:
            self._publish_common(0.0, jump=False)
            self.stop_reason = "APPROACH 等待新鲜 /odom"
            self.get_logger().warn("APPROACH 中等待新鲜 /odom，任务保持锁定", throttle_duration_sec=1.0)
            return

        self.traveled = self._forward_travel()
        dt = max(0.0, min(now - self.last_approach_time, 0.20))
        self.last_approach_time = now
        self.commanded_traveled += self.approach_speed_m_s * dt
        trigger_traveled = (
            self.commanded_traveled
            if self.trigger_distance_source == "commanded"
            else self.traveled
        )
        self.trigger_traveled = trigger_traveled
        self.remaining = self.target_travel - trigger_traveled
        if self.remaining <= self.position_tolerance_m:
            self._begin_jump(now)
            return

        if self.max_approach_seconds > 0.0 and now - self.approach_start_time > self.max_approach_seconds:
            self._publish_common(0.0, jump=False)
            self._set_state(self.STATE_STOPPED, "APPROACH 超时")
            self.stop_reason = "APPROACH 超时"
            self.get_logger().error("APPROACH 超时，停车并保持 STOPPED")
            return

        self.stop_reason = (
            "APPROACH 恒速前进 source=%s cmd_dist=%.3f odom_dist=%.3f"
            % (self.trigger_distance_source, self.commanded_traveled, self.traveled)
        )
        self._publish_common(self.approach_speed_m_s, jump=False)

    def _begin_jump(self, now: float) -> None:
        if not self.jump_enabled:
            self._publish_common(0.0, jump=False)
            self._set_state(self.STATE_STOPPED, "jump_enabled=false")
            self.stop_reason = "到达起跳点但跳跃关闭"
            self.get_logger().warn("到达起跳点；jump_enabled=false，停车保持 STOPPED")
            return

        self._set_state(self.STATE_JUMP, "到达起跳点，开始跳跃脉冲")
        self.state_deadline = now + self.jump_pulse_seconds
        self.stop_reason = "JUMP 脉冲中"
        self.get_logger().warn(
            "到达起跳点，触发 /cmd_jump: forward=%.2fm/s pulse=%.2fs"
            % (self.jump_forward_speed_m_s, self.jump_pulse_seconds)
        )

    def _handle_jump(self, now: float) -> None:
        if now >= self.state_deadline:
            self._set_state(self.STATE_LAND, "跳跃脉冲结束")
            self.state_deadline = now + self.landing_forward_seconds
            self._publish_common(self.jump_forward_speed_m_s * self.landing_speed_scale, jump=False)
            self.stop_reason = "LAND 落地前行"
            self.get_logger().info(
                "跳跃脉冲结束，落地前行 %.2fs" % self.landing_forward_seconds
            )
            return
        self.stop_reason = "JUMP 脉冲中"
        self._publish_common(self.jump_forward_speed_m_s, jump=True)

    def _handle_land(self, now: float) -> None:
        if now >= self.state_deadline:
            self._set_state(self.STATE_DONE, "落地前行结束")
            self.state_deadline = now + self.recover_seconds
            self._publish_common(0.0, jump=False)
            self.stop_reason = "DONE 停车"
            self.get_logger().info("落地前行结束，停车恢复")
            return
        self.stop_reason = "LAND 落地前行"
        self._publish_common(self.jump_forward_speed_m_s * self.landing_speed_scale, jump=False)

    def _set_state(self, state: str, reason: str) -> None:
        if self.state == state:
            return
        previous = self.state
        self.state = state
        self.last_terminal_log_time = 0.0
        self.get_logger().warn(f"STATE {previous} -> {state}: {reason}")

    def _forward_travel(self) -> float:
        if self.odom_xy is None:
            return 0.0
        x, y = self.odom_xy
        return (x - self.lock_x) * math.cos(self.lock_yaw) + (y - self.lock_y) * math.sin(self.lock_yaw)

    def _publish_common(self, speed: float, jump: bool = False) -> None:
        self.last_command_speed = float(max(0.0, speed))
        self.last_jump_command = bool(jump)

        profile = Float32MultiArray()
        if self.use_low_jump_profile:
            profile.data = [self.jump_min_height, self.jump_max_height]
        else:
            profile.data = [0.0, 0.0]
        self.profile_pub.publish(profile)

        posture = JointState()
        posture.header.stamp = self.get_clock().now().to_msg()
        posture.name = POSTURE_NAMES
        posture.position = [self.stand_height, 0.0, 0.0, 0.0]
        self.posture_pub.publish(posture)

        twist = Twist()
        twist.linear.x = self.last_command_speed
        self.cmd_vel_pub.publish(twist)

        message = Bool()
        message.data = bool(jump)
        self.cmd_jump_pub.publish(message)

    def _log_runtime_status(self, now: float) -> None:
        if now - self.last_terminal_log_time < self.terminal_log_period_s:
            return
        self.last_terminal_log_time = now

        detection = self.latest_detection
        if detection is None:
            det_text = "det=none"
        else:
            age = now - self.latest_detection_time
            det_text = (
                "det=%d age=%.2fs d=%.3fm h=%.3fm conf=%.2f confirm=%d/%d"
                % (
                    1 if detection.detected else 0,
                    age,
                    float(detection.distance),
                    float(detection.height),
                    float(detection.confidence),
                    detection.confirm_frames,
                    detection.required_confirm_frames,
                )
            )

        if self.odom_xy is None:
            odom_text = "odom=none"
        else:
            odom_text = "odom_age=%.2fs yaw=%.2f" % (
                now - self.last_odom_time,
                self.odom_yaw,
            )

        self.get_logger().info(
            "APPROACH state=%s reason=%s cmd_v=%.3fm/s jump=%d target=%.3fm "
            "traveled=%.3fm cmd_dist=%.3fm remaining=%.3fm locked=%d %s %s"
            % (
                self.state,
                self.stop_reason,
                self.last_command_speed,
                1 if self.last_jump_command else 0,
                self.target_travel,
                self.trigger_traveled,
                self.commanded_traveled,
                self.remaining,
                1 if self.locked else 0,
                det_text,
                odom_text,
            )
        )

    def _open_file_log(self) -> None:
        if not self.enable_file_log:
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_path = os.path.join(self.log_dir, f"{stamp}_approach.csv")
            self.log_file = open(self.log_path, "w", newline="", buffering=1)
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(
                [
                    "wall_time_s",
                    "elapsed_s",
                    "state",
                    "reason",
                    "cmd_v_m_s",
                    "cmd_jump",
                    "locked",
                    "jump_enabled",
                    "detected_distance_m",
                    "stop_distance_m",
                    "target_travel_m",
                    "trigger_distance_source",
                    "trigger_traveled_m",
                    "cmd_dist_m",
                    "odom_dist_m",
                    "remaining_m",
                    "odom_x_m",
                    "odom_y_m",
                    "odom_yaw_rad",
                    "odom_age_s",
                    "detected",
                    "detection_age_s",
                    "detection_distance_m",
                    "detection_height_m",
                    "detection_confidence",
                    "detection_confirm_frames",
                    "detection_required_frames",
                    "approach_speed_m_s",
                    "jump_forward_speed_m_s",
                    "jump_min_height_m",
                    "jump_max_height_m",
                    "jump_pulse_seconds",
                    "landing_forward_seconds",
                ]
            )
            self.get_logger().warn(f"APPROACH CSV 日志: {self.log_path}")
        except OSError as error:
            self.log_file = None
            self.log_writer = None
            self.get_logger().error(f"无法创建 APPROACH CSV 日志: {error}")

    def _write_file_log(self, now: float) -> None:
        if self.log_writer is None:
            return

        detection = self.latest_detection
        if detection is None:
            det_detected = 0
            det_age = float("nan")
            det_distance = float("nan")
            det_height = float("nan")
            det_confidence = 0.0
            det_confirm = 0
            det_required = 0
        else:
            det_detected = 1 if detection.detected else 0
            det_age = now - self.latest_detection_time
            det_distance = float(detection.distance)
            det_height = float(detection.height)
            det_confidence = float(detection.confidence)
            det_confirm = int(detection.confirm_frames)
            det_required = int(detection.required_confirm_frames)

        if self.odom_xy is None:
            odom_x = float("nan")
            odom_y = float("nan")
            odom_age = float("nan")
        else:
            odom_x, odom_y = self.odom_xy
            odom_age = now - self.last_odom_time

        self.log_writer.writerow(
            [
                f"{time.time():.6f}",
                f"{now - self.run_started_mono:.6f}",
                self.state,
                self.stop_reason,
                f"{self.last_command_speed:.6f}",
                1 if self.last_jump_command else 0,
                1 if self.locked else 0,
                1 if self.jump_enabled else 0,
                f"{self.detected_distance:.6f}",
                f"{self.stop_distance_m:.6f}",
                f"{self.target_travel:.6f}",
                self.trigger_distance_source,
                f"{self.trigger_traveled:.6f}",
                f"{self.commanded_traveled:.6f}",
                f"{self.traveled:.6f}",
                f"{self.remaining:.6f}",
                f"{odom_x:.6f}",
                f"{odom_y:.6f}",
                f"{self.odom_yaw:.6f}",
                f"{odom_age:.6f}",
                det_detected,
                f"{det_age:.6f}",
                f"{det_distance:.6f}",
                f"{det_height:.6f}",
                f"{det_confidence:.6f}",
                det_confirm,
                det_required,
                f"{self.approach_speed_m_s:.6f}",
                f"{self.jump_forward_speed_m_s:.6f}",
                f"{self.jump_min_height:.6f}",
                f"{self.jump_max_height:.6f}",
                f"{self.jump_pulse_seconds:.6f}",
                f"{self.landing_forward_seconds:.6f}",
            ]
        )

    def close_log(self) -> None:
        if self.log_file is not None:
            self.get_logger().warn(f"APPROACH CSV 日志已保存: {self.log_path}")
            self.log_file.close()
            self.log_file = None
            self.log_writer = None

    def _publish_status(self) -> None:
        msg = StepApproachStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.state = self.state
        msg.locked = self.locked
        msg.step_detected = bool(
            self.latest_detection is not None and self.latest_detection.detected
        )
        msg.detected_distance = float(self.detected_distance)
        msg.height = float(self.locked_height)
        msg.confidence = float(self.locked_confidence)
        msg.stop_distance = float(self.stop_distance_m)
        msg.target_travel = float(self.target_travel)
        msg.traveled = float(self.trigger_traveled)
        msg.remaining = float(self.remaining)
        self.status_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepApproachTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            node._publish_common(0.0, jump=False)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(1.0 / node.control_rate_hz)
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
