#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""低头姿态下的「横向偏置」标定探针。

为什么要有这个文件
------------------
之前的录像都是「车自己在按 err 走」，所以从视频里永远分不清
到底是「车真的偏」还是「检测器偏」。这个探针把两件事拆开：

  1. 车由人用遥控器开一条真正的直线（车不会自己修方向）
  2. 程序只负责把跑赛道时的姿态摆出来，并把检测到的 err 记下来

车被摆成跑赛道时的姿态（压低 + 低头），人沿直线开，如果
err 的中位数稳定不为 0，那就是相机/车体的横向安装偏置，
直接填进 TARGET_OFFSET_PIXELS 就行。

和 shiwai_lane_fixed.py 的关系
------------------------------
* 检测像素逻辑「完全复用」shiwai_lane_fixed.py 里的
  DualYellowLineDetector —— 本文件不复制掩码/成对逻辑，
  保证两边算出来的 err 是同一个 err。
* 本文件 **不创建 /cmd_vel 发布者**（一个都没有）。按 readme：
  SWA=2（ROS 模式）时「上位机没有发布数据，遥控器会接管控制权」，
  所以这里不发速度，方向盘归遥控器。
* /cmd_posture 单独发，用来维持低头姿态。

怎么跑
------
    # 车摆直道正中、车头对正，SWA=2，遥控器先别动
    ros2 run wheel_robot shiwai_pitch_probe.py

    # 屏幕/终端确认姿态已经是 [0.20, 0.0, 10.0] 后，用遥控器开直线
    # 眼睛盯着车道正中修方向（不要按 err 修，按你自己看到的修）
    # 开完按 Q 或 Esc，终端打印统计和推荐值

如果发现遥控器推不动（下位机认为上位机还在发数据），退回：

    ros2 run wheel_robot shiwai_pitch_probe.py \
        --ros-args -p publish_posture:=false

  然后 SWA=1 / SWB=0 / SWD=1（遥控模式2），右摇杆手动压高度和俯仰。

输出
----
* 终端：实时 err / 中位数 / 推荐 TARGET_OFFSET_PIXELS
* CSV：~/wheel_robot/outdoor_check/pitch_probe_<时间>.csv
* 视频：~/wheel_robot/recordings/pitch_probe_<时间>.avi（可用 -p record:=false 关掉）
"""

from __future__ import annotations

import os
import signal
import sys
import threading
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 复用同一个检测器，避免两处算法跑偏。
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import shiwai_lane_fixed as base  # noqa: E402

OUT_DIR = Path.home() / "wheel_robot" / "outdoor_check"
REC_DIR = Path(base.RECORDINGS_DIR)


class PitchProbe(Node):
    def __init__(self) -> None:
        super().__init__("shiwai_pitch_probe")
        self._declare_parameters()
        self._load_parameters()

        # ---------------------------------------------------------------
        # 关键：这里 deliberately 不创建 /cmd_vel 发布者。
        # 只要不发速度，下位机在 Cmd_Vel_Timeout_Threshold(0.30s) 之后
        # 就会把控制权交回遥控器。
        # ---------------------------------------------------------------
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)

        self.detector = base.DualYellowLineDetector(
            search_top_ratio=self.search_top_ratio,
            search_bottom_ratio=self.search_bottom_ratio,
            search_left_ratio=self.search_left_ratio,
            search_right_ratio=self.search_right_ratio,
            yellow_h_min=self.lane_yellow_h_min,
            yellow_h_max=self.lane_yellow_h_max,
            yellow_s_min=self.lane_yellow_s_min,
            yellow_v_min=self.lane_yellow_v_min,
            black_value_max=self.black_value_max,
            black_saturation_max=self.black_saturation_max,
            min_boundary_lane_contrast=self.min_boundary_lane_contrast,
            morphology_kernel=self.morphology_kernel,
            min_line_width_ratio=self.min_line_width_ratio,
            max_line_width_ratio=self.max_line_width_ratio,
            expected_line_width_ratio=self.expected_line_width_ratio,
            width_tolerance_ratio=self.width_tolerance_ratio,
            min_lane_width_ratio=self.min_lane_width_ratio,
            max_lane_width_ratio=self.max_lane_width_ratio,
            expected_lane_width_ratio=self.expected_lane_width_ratio,
            lane_width_tolerance_ratio=self.lane_width_tolerance_ratio,
            max_center_jump_ratio=self.max_center_jump_ratio,
            min_valid_row_fraction=self.min_valid_row_fraction,
            min_confidence=self.min_confidence,
            target_offset_pixels=self.target_offset_pixels,
        )

        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._frame_time = 0.0
        self._sequence = 0
        self._camera_ok = False
        self._capture: Optional[cv2.VideoCapture] = None

        # 统计
        self._errors: List[float] = []
        self._widths: List[float] = []
        self._single_ok = 0
        self._invalid = 0
        self._reason_count = {}
        self._csv_rows: List[str] = []
        self._t0 = time.monotonic()

        self._writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None
        self._window_ready = False

        if self.record:
            REC_DIR.mkdir(parents=True, exist_ok=True)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self._video_path = REC_DIR / f"pitch_probe_{stamp}.avi"
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self._writer = cv2.VideoWriter(
                str(self._video_path), fourcc, 15.0, (self.image_width, self.image_height)
            )
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        self._csv_path = OUT_DIR / f"pitch_probe_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        self._csv_rows.append(
            "t,err,valid,reason,lane_width,left_rows,right_rows,left_edge_x,conf"
        )

        if self.show_debug:
            try:
                cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.debug_window_name, 1280, 960)
                cv2.setWindowProperty(
                    self.debug_window_name,
                    cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN,
                )
                self._window_ready = True
            except cv2.error as error:
                self.get_logger().error(f"调试窗口创建失败: {error}")

        self._posture_thread = threading.Thread(
            target=self._posture_loop, name="pitch_probe_posture", daemon=True
        )
        self._posture_thread.start()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="pitch_probe_camera", daemon=True
        )
        self._capture_thread.start()
        self._timer = self.create_timer(1.0 / self.control_rate, self._cycle)

        self.get_logger().info("=" * 62)
        self.get_logger().info("低头标定探针启动")
        if self.publish_posture:
            self.get_logger().info(
                f"姿态：高度 {self.height:.2f} m / 横滚 0.0° / 俯仰 {self.pitch_deg:.1f}°"
            )
        else:
            self.get_logger().info("publish_posture=false：请用遥控器自己摆姿态")
        self.get_logger().info(
            "★ 本程序不发 /cmd_vel。遥控器 SWA 打到 2（ROS 模式），"
            "0.3 秒后遥控器接管，可用左摇杆前后+航向。"
        )
        self.get_logger().info(
            "★ 用眼睛盯着车道正中把车开成一条直线（不要按 err 修方向）。"
            "开完按 Q 或 Esc。"
        )
        self.get_logger().info("=" * 62)

    # ------------------------------------------------------------------
    # 参数
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        defaults = {
            # 姿态（沿用巡线时的 stand_height + PITCH_DEG，不是下蹲高度）
            "pitch_deg": float(base.PITCH_DEG),
            "height": float(base.STAND_HEIGHT),
            "publish_posture": True,
            "posture_rate": 20.0,
            # 主循环
            "control_rate": 15.0,
            "show_debug": True,
            "record": True,
            "debug_window_name": "Pitch Probe (Q/Esc to stop)",
            # 摄像头（与 shiwai.yaml 一致）
            "camera_backend": "v4l2",
            "camera_id": 0,
            "camera_device": "/dev/video0",
            "image_width": 640,
            "image_height": 480,
            "capture_frame_rate": 30.0,
            "pixel_format": "YUYV",
            "rotation_degrees": 0,
            "mirror_mode": "none",
            "camera_retry_seconds": 1.0,
            # 检测（与 shiwai.yaml 一致；lane_yellow_* 用代码默认值）
            "search_top_ratio": 0.24,
            "search_bottom_ratio": 0.86,
            "search_left_ratio": 0.0,
            "search_right_ratio": 1.0,
            "lane_yellow_h_min": 24,
            "lane_yellow_h_max": 40,
            "lane_yellow_s_min": 35,
            "lane_yellow_v_min": 70,
            "black_value_max": 95,
            "black_saturation_max": 110,
            "min_boundary_lane_contrast": 18.0,
            "morphology_kernel": 3,
            "min_line_width_ratio": 0.012,
            "max_line_width_ratio": 0.50,
            "expected_line_width_ratio": 0.08,
            "width_tolerance_ratio": 0.08,
            "min_lane_width_ratio": 0.30,
            "max_lane_width_ratio": 0.82,
            "expected_lane_width_ratio": 0.59,
            "lane_width_tolerance_ratio": 0.30,
            "max_center_jump_ratio": 0.04,
            "min_valid_row_fraction": 0.12,
            "min_confidence": 0.38,
            # 标定期间必须保持 0，否则测的是「已经补偿过」的残差
            "target_offset_pixels": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        names = [
            "pitch_deg",
            "height",
            "publish_posture",
            "posture_rate",
            "control_rate",
            "show_debug",
            "record",
            "debug_window_name",
            "camera_backend",
            "camera_id",
            "camera_device",
            "image_width",
            "image_height",
            "capture_frame_rate",
            "pixel_format",
            "rotation_degrees",
            "mirror_mode",
            "camera_retry_seconds",
            "search_top_ratio",
            "search_bottom_ratio",
            "search_left_ratio",
            "search_right_ratio",
            "lane_yellow_h_min",
            "lane_yellow_h_max",
            "lane_yellow_s_min",
            "lane_yellow_v_min",
            "black_value_max",
            "black_saturation_max",
            "min_boundary_lane_contrast",
            "morphology_kernel",
            "min_line_width_ratio",
            "max_line_width_ratio",
            "expected_line_width_ratio",
            "width_tolerance_ratio",
            "min_lane_width_ratio",
            "max_lane_width_ratio",
            "expected_lane_width_ratio",
            "lane_width_tolerance_ratio",
            "max_center_jump_ratio",
            "min_valid_row_fraction",
            "min_confidence",
            "target_offset_pixels",
        ]
        int_names = (
            "camera_id",
            "image_width",
            "image_height",
            "morphology_kernel",
            "lane_yellow_h_min",
            "lane_yellow_h_max",
            "lane_yellow_s_min",
            "lane_yellow_v_min",
            "black_value_max",
            "black_saturation_max",
            "rotation_degrees",
        )
        bool_names = ("show_debug", "record", "publish_posture")
        str_names = (
            "camera_backend",
            "camera_device",
            "pixel_format",
            "mirror_mode",
            "debug_window_name",
        )
        for name in names:
            value = self.get_parameter(name).value
            if name in str_names:
                value = str(value)
            elif name in int_names:
                value = int(value)
            elif name in bool_names:
                value = bool(value)
            else:
                value = float(value)
            setattr(self, name, value)

    # ------------------------------------------------------------------
    # 姿态
    # ------------------------------------------------------------------
    def _posture_loop(self) -> None:
        period = 1.0 / max(1.0, float(self.posture_rate))
        while not self._stop.is_set():
            if self.publish_posture:
                message = JointState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.name = ["joint_height", "joint_roll", "joint_pitching"]
                message.position = [
                    float(np.clip(self.height, 0.14, 0.36)),
                    0.0,
                    float(np.clip(self.pitch_deg, -35.0, 35.0)),
                ]
                try:
                    self.posture_pub.publish(message)
                except Exception:
                    pass
            self._stop.wait(period)

    # ------------------------------------------------------------------
    # 摄像头
    # ------------------------------------------------------------------
    def _open_camera(self) -> Optional[cv2.VideoCapture]:
        if self.camera_backend == "v4l2":
            capture = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(self.camera_id)
        if not capture.isOpened():
            capture.release()
            return None
        if self.camera_backend == "v4l2":
            capture.set(
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.pixel_format)
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        capture.set(cv2.CAP_PROP_FPS, self.capture_frame_rate)
        self.get_logger().info(
            f"摄像头: {int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
            f"{capture.get(cv2.CAP_PROP_FPS):.1f} fps"
        )
        return capture

    def _capture_loop(self) -> None:
        while not self._stop.is_set():
            if self._capture is None or not self._capture.isOpened():
                self._capture = self._open_camera()
                if self._capture is None:
                    with self._lock:
                        self._camera_ok = False
                        self._frame = None
                    self.get_logger().warn("摄像头打不开，重试中…")
                    self._stop.wait(self.camera_retry_seconds)
                    continue
            ok, frame = self._capture.read()
            if not ok or frame is None:
                with self._lock:
                    self._camera_ok = False
                    self._frame = None
                self._capture.release()
                self._capture = None
                self._stop.wait(self.camera_retry_seconds)
                continue
            frame = self._transform(frame)
            with self._lock:
                self._frame = frame
                self._frame_time = time.monotonic()
                self._sequence += 1
                self._camera_ok = True

    def _transform(self, frame: np.ndarray) -> np.ndarray:
        if self.rotation_degrees == 90:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        elif self.rotation_degrees == 180:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        elif self.rotation_degrees == 270:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        if self.mirror_mode == "horizontal":
            frame = cv2.flip(frame, 1)
        elif self.mirror_mode == "vertical":
            frame = cv2.flip(frame, 0)
        elif self.mirror_mode == "both":
            frame = cv2.flip(frame, -1)
        return frame

    # ------------------------------------------------------------------
    # 主循环
    # ------------------------------------------------------------------
    def _cycle(self) -> None:
        with self._lock:
            frame = self._frame.copy() if self._frame is not None else None
            ok = self._camera_ok
            frame_time = self._frame_time
        if not ok or frame is None:
            return
        if time.monotonic() - frame_time > 1.0:
            return

        result = self.detector.detect(frame)
        now = time.monotonic() - self._t0
        if result.valid:
            self._errors.append(result.error)
            if result.lane_width_pixels > 0:
                self._widths.append(result.lane_width_pixels)
            if result.left_line_rows > 0 or result.right_line_rows > 0:
                self._single_ok += 1
        else:
            self._invalid += 1
            self._reason_count[result.reason] = self._reason_count.get(result.reason, 0) + 1

        self._csv_rows.append(
            f"{now:.3f},{result.error:.4f},{int(result.valid)},{result.reason},"
            f"{result.lane_width_pixels:.1f},{result.left_line_rows},"
            f"{result.right_line_rows},{result.left_edge_x:.1f},{result.confidence:.3f}"
        )

        debug = self._annotate(result, now)
        if self._writer is not None:
            try:
                self._writer.write(debug)
            except Exception:
                pass
        if self._window_ready:
            try:
                cv2.imshow(self.debug_window_name, debug)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q"), ord("Q")):
                    self.request_stop("按键退出")
            except cv2.error:
                self._window_ready = False

        if len(self._csv_rows) % 60 == 0:
            self._print_live(result)

    def _annotate(self, result, now: float) -> np.ndarray:
        image = result.debug_frame.copy()
        h, w = image.shape[:2]

        med = float(np.median(self._errors)) if self._errors else 0.0
        n = len(self._errors)
        offset = int(round(med * 0.5 * w))

        cv2.rectangle(image, (0, 0), (w, 96), (0, 0, 0), -1)
        if result.valid:
            color = (0, 255, 0) if abs(result.error) < 0.06 else (0, 200, 255)
        else:
            color = (0, 0, 255)
        cv2.putText(
            image,
            f"err {result.error:+.3f}",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.1,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"median {med:+.3f}  n={n}",
            (10, 72),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (200, 255, 200),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"lane {result.lane_width_pixels:.0f}px  -> offset {offset:+d}px",
            (10, 92),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (180, 220, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            f"pitch {self.pitch_deg:.0f}  h {self.height:.2f}  "
            f"valid {n}  bad {self._invalid}  {now:.0f}s",
            (w - 400, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )
        if not result.valid:
            cv2.putText(
                image,
                result.reason,
                (10, h - 14),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        # 目标线（画面正中）
        cv2.line(image, (w // 2, 96), (w // 2, h), (140, 140, 140), 1, cv2.LINE_AA)
        return image

    def _print_live(self, result) -> None:
        n = len(self._errors)
        if n == 0:
            return
        med = float(np.median(self._errors))
        self.get_logger().info(
            f"[{n:4d} 帧] err={result.error:+.3f} "
            f"中位数={med:+.3f} → 偏移 {int(round(med * 320.0)):+d}px"
        )

    # ------------------------------------------------------------------
    # 结束与统计
    # ------------------------------------------------------------------
    def request_stop(self, reason: str) -> None:
        if not self._stop.is_set():
            self.get_logger().info(f"准备结束：{reason}")
            self._stop.set()

    def close(self) -> None:
        self._stop.set()
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception:
                pass
            self._writer = None
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
            self._capture = None
        try:
            self._csv_path.write_text("\n".join(self._csv_rows) + "\n", encoding="utf-8")
            wrote_csv = True
        except Exception as error:
            wrote_csv = False
            self.get_logger().error(f"CSV 写入失败: {error}")
        self._print_summary(wrote_csv)

    def _print_summary(self, wrote_csv: bool) -> None:
        log = self.get_logger()
        n = len(self._errors)
        log.info("")
        log.info("=" * 62)
        log.info("低头标定结果")
        log.info("-" * 62)
        log.info(f"有效帧 {n} / 无效帧 {self._invalid}")
        if self._invalid and self._reason_count:
            detail = " ".join(f"{k}={v}" for k, v in sorted(self._reason_count.items()))
            log.info(f"无效原因 {detail}")
        if n < 30:
            log.warn(f"有效帧只有 {n} 帧，太少了（建议 >200）。再开一趟直线试试。")
            log.info("=" * 62)
            return

        arr = np.asarray(self._errors, dtype=float)
        med = float(np.median(arr))
        mean = float(np.mean(arr))
        std = float(np.std(arr))
        p10, p90 = (float(v) for v in np.percentile(arr, [10, 90]))
        half = 0.5 * float(self.image_width)
        offset = int(round(med * half))

        log.info(
            f"err 中位数 {med:+.4f}   均值 {mean:+.4f}   标准差 {std:.4f}"
        )
        log.info(f"err 10%~90% 分位 [{p10:+.4f}, {p90:+.4f}]")
        log.info(
            f"推荐  TARGET_OFFSET_PIXELS = {offset:+d}"
            f"   （= 中位数 {med:+.4f} × {half:.0f}）"
        )
        if self._widths:
            wmed = float(np.median(self._widths))
            wratio = wmed / float(self.image_width)
            log.info(
                f"车道宽度中位数 {wmed:.0f}px = 比例 {wratio:.3f}"
                f"（配置 {self.expected_lane_width_ratio:.3f}）"
            )
            if abs(wratio - self.expected_lane_width_ratio) > 0.04:
                log.warn(
                    "  车道宽度比例与配置差得多，"
                    "expected_lane_width_ratio 也要一起改。"
                )
        log.info("-" * 62)
        if std > 0.20:
            log.warn(
                f"标准差 {std:.3f} 偏大：这一趟开得不直，"
                "中位数可能被蛇行带偏，建议再开一趟更直的。"
            )
        elif abs(med) < 0.03:
            log.info(
                "中位数接近 0：检测器没有横向偏置。"
                "如果车看起来还是偏，那是机械/安装问题，不是 err 问题。"
            )
        else:
            direction = "左" if med < 0 else "右"
            log.info(
                f"中位数稳定在 {med:+.3f}（虚拟中点偏{direction} "
                f"{abs(med) * half:.0f}px）→ 存在横向安装偏置。"
            )
            log.info(
                "把这行填进 shiwai_lane_fixed.py 开头："
                f"TARGET_OFFSET_PIXELS = {offset}"
            )
        log.info("-" * 62)
        if wrote_csv:
            log.info(f"CSV: {self._csv_path}")
        if self._video_path is not None:
            log.info(f"视频: {self._video_path}")
        log.info("=" * 62)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PitchProbe()

    def _handler(signum, _frame) -> None:
        node.request_stop("收到信号")

    signal.signal(signal.SIGINT, _handler)
    signal.signal(signal.SIGTERM, _handler)
    try:
        while rclpy.ok() and not node._stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("低头标定探针已退出")


if __name__ == "__main__":
    main()
