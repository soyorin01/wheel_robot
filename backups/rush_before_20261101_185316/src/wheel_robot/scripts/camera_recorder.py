#!/usr/bin/env python3
"""Record clean camera frames while the robot is driven by remote control.

This node never creates a control publisher.  The saved video contains no text
overlay, so it can later be passed frame-by-frame through the ICN detector.
"""

from datetime import datetime
import json
from pathlib import Path
import shutil
import time
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node


class CameraRecorder(Node):
    VALID_ROTATIONS = (0, 90, 180, 270)
    VALID_MIRROR_MODES = ("none", "horizontal", "vertical", "both")

    def __init__(self) -> None:
        super().__init__("camera_recorder")
        self._declare_parameters()
        self._load_parameters()
        self._validate_parameters()

        self.capture: Optional[cv2.VideoCapture] = None
        self.writer: Optional[cv2.VideoWriter] = None
        self.output_path: Optional[Path] = None
        self.metadata_path: Optional[Path] = None
        self.stop_requested = False
        self.frame_count = 0
        self.read_failures = 0
        self.started_monotonic = 0.0
        self.started_wall_time = ""
        self.last_status_time = 0.0
        self.actual_width = 0
        self.actual_height = 0
        self.actual_fps = 0.0
        self.backend_name = "unknown"

    def _declare_parameters(self) -> None:
        self.declare_parameter("camera_backend", "v4l2")
        self.declare_parameter("camera_id", 0)
        self.declare_parameter("camera_device", "/dev/video0")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("pixel_format", "YUYV")
        self.declare_parameter("rotation_degrees", 0)
        self.declare_parameter("mirror_mode", "none")
        self.declare_parameter("codec", "MJPG")
        self.declare_parameter(
            "output_directory", str(Path.home() / "wheel_robot" / "recordings")
        )
        self.declare_parameter("output_filename", "")
        # X11/remote-desktop preview reduced the measured capture rate from
        # 30 fps to about 3.6 fps on this robot.  Record at full speed by
        # default; preview can still be enabled explicitly when needed.
        self.declare_parameter("show_preview", False)
        self.declare_parameter("max_duration_seconds", 0.0)
        self.declare_parameter("minimum_free_space_mb", 512)

    def _load_parameters(self) -> None:
        self.camera_backend = str(
            self.get_parameter("camera_backend").value
        ).lower()
        self.camera_id = int(self.get_parameter("camera_id").value)
        self.camera_device = str(self.get_parameter("camera_device").value)
        self.image_width = int(self.get_parameter("image_width").value)
        self.image_height = int(self.get_parameter("image_height").value)
        self.frame_rate = float(self.get_parameter("frame_rate").value)
        self.pixel_format = str(self.get_parameter("pixel_format").value).upper()
        self.rotation_degrees = int(
            self.get_parameter("rotation_degrees").value
        )
        self.mirror_mode = str(self.get_parameter("mirror_mode").value).lower()
        self.codec = str(self.get_parameter("codec").value).upper()
        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        ).expanduser()
        self.output_filename = str(self.get_parameter("output_filename").value)
        self.show_preview = bool(self.get_parameter("show_preview").value)
        self.max_duration_seconds = float(
            self.get_parameter("max_duration_seconds").value
        )
        self.minimum_free_space_mb = int(
            self.get_parameter("minimum_free_space_mb").value
        )

    def _validate_parameters(self) -> None:
        if self.camera_backend not in ("auto", "v4l2"):
            raise ValueError("camera_backend 只能是 auto 或 v4l2")
        if self.camera_id < 0 or self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("摄像头编号和图像尺寸无效")
        if self.frame_rate <= 0.0:
            raise ValueError("frame_rate 必须大于 0")
        if len(self.pixel_format) != 4 or len(self.codec) != 4:
            raise ValueError("pixel_format 和 codec 必须是四字符格式")
        if self.rotation_degrees not in self.VALID_ROTATIONS:
            raise ValueError("rotation_degrees 只能是 0、90、180 或 270")
        if self.mirror_mode not in self.VALID_MIRROR_MODES:
            raise ValueError("mirror_mode 只能是 none、horizontal、vertical 或 both")
        if self.max_duration_seconds < 0.0:
            raise ValueError("max_duration_seconds 不能为负数")
        if self.minimum_free_space_mb < 64:
            raise ValueError("minimum_free_space_mb 不能小于 64")
        if self.output_filename and Path(self.output_filename).name != self.output_filename:
            raise ValueError("output_filename 只能是文件名，不能包含目录")

    def _make_output_paths(self) -> None:
        self.output_directory.mkdir(parents=True, exist_ok=True)
        free_mb = shutil.disk_usage(self.output_directory).free / (1024 * 1024)
        if free_mb < self.minimum_free_space_mb:
            raise RuntimeError(
                f"录像目录剩余空间只有 {free_mb:.0f} MB，低于安全值 "
                f"{self.minimum_free_space_mb} MB"
            )

        if self.output_filename:
            filename = self.output_filename
            if not Path(filename).suffix:
                filename += ".avi"
        else:
            timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
            filename = f"course_{timestamp}.avi"
        self.output_path = self.output_directory / filename
        if self.output_path.exists():
            raise RuntimeError(f"输出文件已存在，不覆盖: {self.output_path}")
        self.metadata_path = self.output_path.with_suffix(
            self.output_path.suffix + ".json"
        )

    def _open_camera(self) -> None:
        if self.camera_backend == "v4l2":
            source = self.camera_device
            capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
        else:
            source = self.camera_id
            capture = cv2.VideoCapture(source)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(
                f"无法打开摄像头 {source}；请先关闭 icn.py 和 camera_viewer.py"
            )

        if self.camera_backend == "v4l2":
            capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*self.pixel_format),
            )
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        capture.set(cv2.CAP_PROP_FPS, self.frame_rate)
        self.capture = capture
        self.actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        reported_fps = float(capture.get(cv2.CAP_PROP_FPS))
        self.actual_fps = reported_fps if reported_fps > 1.0 else self.frame_rate
        self.backend_name = capture.getBackendName()
        self.get_logger().info(
            f"摄像头已打开: {source}, 后端={self.backend_name}, "
            f"{self.actual_width}x{self.actual_height} @ {self.actual_fps:.1f} fps"
        )

    def _transform_frame(self, frame):
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

    def _open_writer(self, frame) -> None:
        assert self.output_path is not None
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(
            str(self.output_path),
            cv2.VideoWriter_fourcc(*self.codec),
            self.actual_fps,
            (width, height),
        )
        if not writer.isOpened():
            writer.release()
            raise RuntimeError(
                f"无法创建录像 {self.output_path}，编码器={self.codec}"
            )
        self.writer = writer
        self.actual_width = width
        self.actual_height = height

    def _check_disk_space(self) -> bool:
        free_mb = shutil.disk_usage(self.output_directory).free / (1024 * 1024)
        if free_mb >= self.minimum_free_space_mb:
            return True
        self.get_logger().error(
            f"磁盘剩余 {free_mb:.0f} MB，录像自动停止，防止写满磁盘"
        )
        return False

    def _show_preview(self, frame, elapsed: float, measured_fps: float) -> None:
        preview = frame.copy()
        cv2.putText(
            preview,
            f"REC {elapsed:06.1f}s  frames={self.frame_count}  {measured_fps:.1f} fps",
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            "Q / Esc: stop and save",
            (12, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("Course Camera Recorder", preview)
        if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
            self.stop_requested = True

    def run(self) -> None:
        self._make_output_paths()
        self._open_camera()
        if self.show_preview:
            try:
                cv2.namedWindow("Course Camera Recorder", cv2.WINDOW_NORMAL)
            except cv2.error as error:
                raise RuntimeError("无法创建录像预览窗口") from error

        self.started_monotonic = time.monotonic()
        self.started_wall_time = datetime.now().astimezone().isoformat()
        self.last_status_time = self.started_monotonic
        self.get_logger().warn("录像即将开始；本程序不发布任何底盘控制话题")

        while rclpy.ok() and not self.stop_requested:
            assert self.capture is not None
            ok, frame = self.capture.read()
            if not ok or frame is None:
                self.read_failures += 1
                if self.read_failures >= 30:
                    raise RuntimeError("摄像头连续读取失败 30 帧，停止录像")
                continue
            self.read_failures = 0
            frame = self._transform_frame(frame)
            if self.writer is None:
                self._open_writer(frame)
                self.get_logger().info(f"开始录制原始画面: {self.output_path}")

            self.writer.write(frame)
            self.frame_count += 1
            now = time.monotonic()
            elapsed = now - self.started_monotonic
            measured_fps = self.frame_count / max(elapsed, 1e-6)
            if self.show_preview:
                self._show_preview(frame, elapsed, measured_fps)
            if self.max_duration_seconds > 0.0 and elapsed >= self.max_duration_seconds:
                self.get_logger().info("达到最大录像时长，自动停止")
                break
            if now - self.last_status_time >= 5.0:
                if not self._check_disk_space():
                    break
                file_mb = (
                    self.output_path.stat().st_size / (1024 * 1024)
                    if self.output_path is not None and self.output_path.exists()
                    else 0.0
                )
                self.get_logger().info(
                    f"REC {elapsed:.1f}s, {self.frame_count} 帧, "
                    f"采集={measured_fps:.1f} fps, 文件约 {file_mb:.1f} MB"
                )
                self.last_status_time = now

    def close(self) -> None:
        duration = (
            time.monotonic() - self.started_monotonic
            if self.started_monotonic > 0.0
            else 0.0
        )
        if self.writer is not None:
            self.writer.release()
            self.writer = None
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        if self.show_preview:
            cv2.destroyAllWindows()

        if self.output_path is None or self.frame_count == 0:
            return
        metadata = {
            "video_path": str(self.output_path.resolve()),
            "started_at": self.started_wall_time,
            "duration_seconds": duration,
            "frame_count": self.frame_count,
            "measured_capture_fps": self.frame_count / max(duration, 1e-6),
            "video_fps": self.actual_fps,
            "width": self.actual_width,
            "height": self.actual_height,
            "codec": self.codec,
            "camera_backend": self.backend_name,
            "camera_device": self.camera_device,
            "pixel_format": self.pixel_format,
            "rotation_degrees": self.rotation_degrees,
            "mirror_mode": self.mirror_mode,
        }
        assert self.metadata_path is not None
        self.metadata_path.write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        self.get_logger().info(
            f"录像已保存: {self.output_path} ({duration:.1f}s, "
            f"{self.frame_count} 帧)"
        )
        self.get_logger().info(f"录像元数据: {self.metadata_path}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CameraRecorder] = None
    try:
        node = CameraRecorder()
        node.run()
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"摄像头录像程序启动失败: {error}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
