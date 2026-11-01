#!/usr/bin/env python3
"""Detect a low step from Astra depth images using a fitted ground plane."""

from __future__ import annotations

import csv
import datetime as dt
import math
import os
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from wheel_robot.msg import StepDetection


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def parameter_bool(value: object) -> bool:
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


@dataclass
class Plane:
    normal: np.ndarray
    offset: float
    inliers: int
    mean_error: float


@dataclass
class StepCandidate:
    distance: float
    height: float
    confidence: float
    pose_valid: bool
    edge_angle_rad: float
    lateral_offset_m: float
    edge_fit_rmse_m: float
    mask: np.ndarray
    ground_mask: np.ndarray
    edge_mask: np.ndarray
    pixels_u: np.ndarray
    pixels_v: np.ndarray
    selected_indices: np.ndarray


class StepDetector(Node):
    """Find 3-7.5 cm step tops above the fitted ground plane."""

    def __init__(self) -> None:
        super().__init__("step_detector")

        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("detection_topic", "/step_detector/detection")
        self.declare_parameter("debug_image_topic", "/step_detector/debug_image")
        self.declare_parameter("depth_scale_m", 0.001)
        self.declare_parameter("float_depth_scale_m", 1.0)
        self.declare_parameter("fallback_horizontal_fov_deg", 58.0)
        self.declare_parameter("fallback_vertical_fov_deg", 45.0)

        self.declare_parameter("roi_x_min", 0.12)
        self.declare_parameter("roi_x_max", 0.88)
        self.declare_parameter("roi_y_min", 0.18)
        self.declare_parameter("roi_y_max", 0.90)
        self.declare_parameter("ground_roi_y_min", 0.58)
        self.declare_parameter("ground_roi_y_max", 0.98)
        self.declare_parameter("min_distance_m", 0.15)
        self.declare_parameter("max_distance_m", 1.60)
        self.declare_parameter("sample_stride", 4)

        self.declare_parameter("ransac_iterations", 80)
        self.declare_parameter("ransac_sample_points", 2500)
        self.declare_parameter("ground_distance_threshold_m", 0.012)
        self.declare_parameter("min_ground_points", 120)
        self.declare_parameter("min_ground_normal_y_abs", 0.45)

        self.declare_parameter("step_height_min_m", 0.03)
        self.declare_parameter("step_height_max_m", 0.075)
        self.declare_parameter("target_step_height_m", 0.05)
        self.declare_parameter("min_step_pixels", 35)
        self.declare_parameter("min_step_width_px", 7)
        self.declare_parameter("min_step_height_px", 4)
        self.declare_parameter("min_step_width_m", 0.10)
        self.declare_parameter("edge_band_depth_m", 0.025)
        self.declare_parameter("front_edge_percentile", 0.12)
        self.declare_parameter("min_edge_pose_points", 12)
        self.declare_parameter("edge_pose_max_rmse_m", 0.020)
        self.declare_parameter("morph_kernel_size", 3)
        self.declare_parameter("min_confidence", 0.50)
        self.declare_parameter("confirm_frames", 5)

        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("visualization_max_distance_m", 1.40)
        self.declare_parameter("terminal_log_period_s", 1.0)
        self.declare_parameter("enable_file_log", True)
        self.declare_parameter("log_dir", "/home/orangepi/wheel_robot/log/step_approach")

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value)
        self.depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        self.float_depth_scale_m = float(self.get_parameter("float_depth_scale_m").value)
        self.fallback_horizontal_fov = math.radians(
            float(self.get_parameter("fallback_horizontal_fov_deg").value)
        )
        self.fallback_vertical_fov = math.radians(
            float(self.get_parameter("fallback_vertical_fov_deg").value)
        )

        self.roi_x_min = float(self.get_parameter("roi_x_min").value)
        self.roi_x_max = float(self.get_parameter("roi_x_max").value)
        self.roi_y_min = float(self.get_parameter("roi_y_min").value)
        self.roi_y_max = float(self.get_parameter("roi_y_max").value)
        self.ground_roi_y_min = float(self.get_parameter("ground_roi_y_min").value)
        self.ground_roi_y_max = float(self.get_parameter("ground_roi_y_max").value)
        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.max_distance_m = float(self.get_parameter("max_distance_m").value)
        self.sample_stride = max(1, int(self.get_parameter("sample_stride").value))

        self.ransac_iterations = max(1, int(self.get_parameter("ransac_iterations").value))
        self.ransac_sample_points = max(
            50, int(self.get_parameter("ransac_sample_points").value)
        )
        self.ground_distance_threshold_m = float(
            self.get_parameter("ground_distance_threshold_m").value
        )
        self.min_ground_points = max(3, int(self.get_parameter("min_ground_points").value))
        self.min_ground_normal_y_abs = clamp(
            float(self.get_parameter("min_ground_normal_y_abs").value), 0.0, 1.0
        )

        self.step_height_min_m = float(self.get_parameter("step_height_min_m").value)
        self.step_height_max_m = float(self.get_parameter("step_height_max_m").value)
        self.target_step_height_m = float(self.get_parameter("target_step_height_m").value)
        self.min_step_pixels = max(1, int(self.get_parameter("min_step_pixels").value))
        self.min_step_width_px = max(1, int(self.get_parameter("min_step_width_px").value))
        self.min_step_height_px = max(1, int(self.get_parameter("min_step_height_px").value))
        self.min_step_width_m = max(0.0, float(self.get_parameter("min_step_width_m").value))
        self.edge_band_depth_m = max(0.001, float(self.get_parameter("edge_band_depth_m").value))
        self.front_edge_percentile = clamp(
            float(self.get_parameter("front_edge_percentile").value), 0.0, 0.5
        )
        self.min_edge_pose_points = max(
            6, int(self.get_parameter("min_edge_pose_points").value)
        )
        self.edge_pose_max_rmse_m = max(
            0.003, float(self.get_parameter("edge_pose_max_rmse_m").value)
        )
        self.morph_kernel_size = max(1, int(self.get_parameter("morph_kernel_size").value))
        self.min_confidence = clamp(float(self.get_parameter("min_confidence").value), 0.0, 1.0)
        self.confirm_frames = max(1, int(self.get_parameter("confirm_frames").value))

        self.publish_debug_image = parameter_bool(self.get_parameter("publish_debug_image").value)
        self.visualization_max_distance_m = max(
            self.min_distance_m + 0.1,
            float(self.get_parameter("visualization_max_distance_m").value),
        )
        self.terminal_log_period_s = max(
            0.1, float(self.get_parameter("terminal_log_period_s").value)
        )
        self.enable_file_log = parameter_bool(self.get_parameter("enable_file_log").value)
        self.log_dir = str(self.get_parameter("log_dir").value)

        self.camera_info: CameraInfo | None = None
        self.confirm_count = 0
        self.last_terminal_log_time = 0.0
        self.rng = np.random.default_rng()
        self.run_started_mono = time.monotonic()
        self.log_file = None
        self.log_writer = None
        self.log_path = ""
        self._open_file_log()

        self.detection_pub = self.create_publisher(StepDetection, self.detection_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 1)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, self.depth_topic, self._on_depth_image, qos_profile_sensor_data
        )

        self.get_logger().info(
            "台阶检测启动: 高度范围 %.3f~%.3fm, 连续确认 %d 帧, depth=%s, info=%s"
            % (
                self.step_height_min_m,
                self.step_height_max_m,
                self.confirm_frames,
                self.depth_topic,
                self.camera_info_topic,
            )
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _on_depth_image(self, message: Image) -> None:
        depth = self._depth_to_meters(message)
        if depth is None:
            return

        intrinsics = self._camera_intrinsics(message.width, message.height)
        if intrinsics is None:
            self._publish_detection(message, None, 0)
            return

        candidate, plane = self._detect_step(depth, intrinsics)
        if candidate is not None and candidate.confidence >= self.min_confidence:
            self.confirm_count += 1
        else:
            self.confirm_count = 0

        detected = self.confirm_count >= self.confirm_frames
        self._publish_detection(message, candidate, self.confirm_count if detected else min(self.confirm_count, self.confirm_frames))
        self._log_detection(candidate, plane, detected)
        self._write_file_log(candidate, plane, detected)
        if self.publish_debug_image:
            self._publish_debug_image(message, depth, intrinsics, candidate, plane, detected)

    def _camera_intrinsics(self, width: int, height: int) -> tuple[float, float, float, float] | None:
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )

        if self.fallback_horizontal_fov <= 0.0 or self.fallback_vertical_fov <= 0.0:
            self.get_logger().warn("CameraInfo 未标定且 fallback FOV 无效", throttle_duration_sec=2.0)
            return None
        fx = width / (2.0 * math.tan(self.fallback_horizontal_fov * 0.5))
        fy = height / (2.0 * math.tan(self.fallback_vertical_fov * 0.5))
        cx = (width - 1.0) * 0.5
        cy = (height - 1.0) * 0.5
        self.get_logger().warn(
            "CameraInfo 未标定，使用 fallback FOV 估算内参", throttle_duration_sec=5.0
        )
        return fx, fy, cx, cy

    def _depth_to_meters(self, message: Image) -> np.ndarray | None:
        encoding = message.encoding.lower()
        if encoding in ("16uc1", "mono16"):
            row_values = message.step // 2
            expected_values = row_values * int(message.height)
            raw = np.frombuffer(message.data, dtype=np.uint16, count=expected_values)
            if raw.size != expected_values:
                self.get_logger().warn("深度图数据长度异常", throttle_duration_sec=2.0)
                return None
            if message.is_bigendian != (sys.byteorder == "big"):
                raw = raw.byteswap()
            image = raw.reshape((int(message.height), row_values))[:, : int(message.width)]
            depth = image.astype(np.float32) * self.depth_scale_m
            depth[image == 0] = np.nan
            return depth

        if encoding == "32fc1":
            row_values = message.step // 4
            expected_values = row_values * int(message.height)
            raw = np.frombuffer(message.data, dtype=np.float32, count=expected_values)
            if raw.size != expected_values:
                self.get_logger().warn("32FC1 深度图数据长度异常", throttle_duration_sec=2.0)
                return None
            if message.is_bigendian != (sys.byteorder == "big"):
                raw = raw.byteswap()
            depth = raw.reshape((int(message.height), row_values))[:, : int(message.width)]
            depth = depth.astype(np.float32, copy=True) * self.float_depth_scale_m
            depth[depth <= 0.0] = np.nan
            return depth

        self.get_logger().warn(
            f"暂不支持深度图编码 {message.encoding}，需要 16UC1/mono16/32FC1",
            throttle_duration_sec=2.0,
        )
        return None

    def _sample_points(
        self,
        depth: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        x_min_ratio: float,
        x_max_ratio: float,
        y_min_ratio: float,
        y_max_ratio: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, tuple[int, int, int, int], tuple[int, int]]:
        height, width = depth.shape
        x0 = int(clamp(x_min_ratio, 0.0, 1.0) * width)
        x1 = int(clamp(x_max_ratio, 0.0, 1.0) * width)
        y0 = int(clamp(y_min_ratio, 0.0, 1.0) * height)
        y1 = int(clamp(y_max_ratio, 0.0, 1.0) * height)
        x1 = max(x0 + 1, min(width, x1))
        y1 = max(y0 + 1, min(height, y1))

        xs = np.arange(x0, x1, self.sample_stride, dtype=np.int32)
        ys = np.arange(y0, y1, self.sample_stride, dtype=np.int32)
        uu, vv = np.meshgrid(xs, ys)
        zz = depth[np.ix_(ys, xs)]
        valid = np.isfinite(zz) & (zz >= self.min_distance_m) & (zz <= self.max_distance_m)

        fx, fy, cx, cy = intrinsics
        z = zz[valid].astype(np.float32)
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points = np.column_stack((x, y, z)).astype(np.float32)
        return points, uu[valid], vv[valid], (x0, y0, x1, y1), uu.shape

    def _detect_step(
        self, depth: np.ndarray, intrinsics: tuple[float, float, float, float]
    ) -> tuple[StepCandidate | None, Plane | None]:
        ground_points, _, _, _, _ = self._sample_points(
            depth,
            intrinsics,
            self.roi_x_min,
            self.roi_x_max,
            self.ground_roi_y_min,
            self.ground_roi_y_max,
        )
        plane = self._fit_ground_plane(ground_points)
        if plane is None:
            return None, None

        points, pixels_u, pixels_v, _, grid_shape = self._sample_points(
            depth,
            intrinsics,
            self.roi_x_min,
            self.roi_x_max,
            self.roi_y_min,
            self.roi_y_max,
        )
        if points.size == 0:
            return None, plane

        signed_height = points @ plane.normal + plane.offset
        ground_mask_flat = np.abs(signed_height) <= self.ground_distance_threshold_m
        step_mask_flat = (
            (signed_height >= self.step_height_min_m)
            & (signed_height <= self.step_height_max_m)
        )

        mask = np.zeros(grid_shape, dtype=np.uint8)
        valid_rows = ((pixels_v - int(clamp(self.roi_y_min, 0.0, 1.0) * depth.shape[0])) // self.sample_stride).astype(np.int32)
        valid_cols = ((pixels_u - int(clamp(self.roi_x_min, 0.0, 1.0) * depth.shape[1])) // self.sample_stride).astype(np.int32)
        mask[valid_rows[step_mask_flat], valid_cols[step_mask_flat]] = 255
        if self.morph_kernel_size > 1:
            kernel = np.ones((self.morph_kernel_size, self.morph_kernel_size), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        if component_count <= 1:
            return None, plane

        best: StepCandidate | None = None
        best_score = -1.0
        for label in range(1, component_count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            width_px = int(stats[label, cv2.CC_STAT_WIDTH])
            height_px = int(stats[label, cv2.CC_STAT_HEIGHT])
            if (
                area < self.min_step_pixels
                or width_px < self.min_step_width_px
                or height_px < self.min_step_height_px
            ):
                continue

            in_component_grid = labels[valid_rows, valid_cols] == label
            indices = np.flatnonzero(in_component_grid & step_mask_flat)
            if indices.size < self.min_step_pixels:
                continue

            component_points = points[indices]
            width_m = float(np.nanpercentile(component_points[:, 0], 95) - np.nanpercentile(component_points[:, 0], 5))
            if width_m < self.min_step_width_m:
                continue

            # 不能用整个台阶的全局最近深度取前沿：车身斜对台阶时，那样只会
            # 留下较近的一端，恰好丢失我们最需要的横边角度。这里按图像列分别
            # 取台阶顶面的最近点，再把左右各列的局部前沿合并起来。
            component_cols = valid_cols[indices]
            edge_chunks: list[np.ndarray] = []
            for grid_col in np.unique(component_cols):
                column_indices = indices[component_cols == grid_col]
                column_z = points[column_indices, 2]
                column_front_z = float(
                    np.nanquantile(column_z, self.front_edge_percentile)
                )
                selected = column_indices[
                    column_z <= column_front_z + self.edge_band_depth_m
                ]
                if selected.size:
                    edge_chunks.append(selected)
            edge_indices = (
                np.unique(np.concatenate(edge_chunks))
                if edge_chunks
                else np.empty(0, dtype=np.int64)
            )
            if edge_indices.size < max(3, self.min_step_width_px):
                continue

            edge_points = points[edge_indices]
            distance = float(np.nanmedian(edge_points[:, 2]))
            height_m = float(np.nanmedian(signed_height[indices]))
            confidence = self._confidence(area, width_m, height_m, edge_indices.size, plane)
            pose_valid, edge_angle, edge_rmse = self._fit_front_edge_pose(edge_points)
            lateral_offset = 0.5 * float(
                np.nanpercentile(component_points[:, 0], 5)
                + np.nanpercentile(component_points[:, 0], 95)
            )

            edge_mask = np.zeros_like(mask)
            edge_rows = valid_rows[edge_indices]
            edge_cols = valid_cols[edge_indices]
            edge_mask[edge_rows, edge_cols] = 255

            candidate = StepCandidate(
                distance=distance,
                height=height_m,
                confidence=confidence,
                pose_valid=pose_valid,
                edge_angle_rad=edge_angle,
                lateral_offset_m=lateral_offset,
                edge_fit_rmse_m=edge_rmse,
                mask=(labels == label).astype(np.uint8) * 255,
                ground_mask=ground_mask_flat,
                edge_mask=edge_mask,
                pixels_u=pixels_u,
                pixels_v=pixels_v,
                selected_indices=indices,
            )
            if confidence > best_score:
                best = candidate
                best_score = confidence

        return best, plane

    def _fit_front_edge_pose(
        self, edge_points: np.ndarray
    ) -> tuple[bool, float, float]:
        """Robustly fit the physical step edge as z=a*x+b in camera coordinates."""
        if edge_points.shape[0] < self.min_edge_pose_points:
            return False, float("nan"), float("nan")

        x = edge_points[:, 0].astype(np.float64)
        z = edge_points[:, 2].astype(np.float64)
        keep = np.isfinite(x) & np.isfinite(z)
        if np.count_nonzero(keep) < self.min_edge_pose_points:
            return False, float("nan"), float("nan")

        # 两轮 MAD 剔除深度飞点；保留至少一半点，避免单侧噪声把横边拉斜。
        for _ in range(2):
            fit_x = x[keep]
            fit_z = z[keep]
            if fit_x.size < self.min_edge_pose_points:
                return False, float("nan"), float("nan")
            design = np.column_stack((fit_x, np.ones_like(fit_x)))
            slope, intercept = np.linalg.lstsq(design, fit_z, rcond=None)[0]
            residual = z - (slope * x + intercept)
            center = float(np.nanmedian(residual[keep]))
            mad = float(np.nanmedian(np.abs(residual[keep] - center)))
            limit = max(0.006, 3.5 * 1.4826 * mad)
            next_keep = keep & (np.abs(residual - center) <= limit)
            if np.count_nonzero(next_keep) < self.min_edge_pose_points:
                break
            keep = next_keep

        fit_x = x[keep]
        fit_z = z[keep]
        if (
            fit_x.size < self.min_edge_pose_points
            or float(np.ptp(fit_x)) < max(0.06, self.min_step_width_m * 0.60)
        ):
            return False, float("nan"), float("nan")
        design = np.column_stack((fit_x, np.ones_like(fit_x)))
        slope, intercept = np.linalg.lstsq(design, fit_z, rcond=None)[0]
        residual = fit_z - (slope * fit_x + intercept)
        rmse = float(math.sqrt(float(np.mean(residual * residual))))
        if not math.isfinite(rmse) or rmse > self.edge_pose_max_rmse_m:
            return False, float("nan"), rmse
        return True, float(math.atan(float(slope))), rmse

    def _fit_ground_plane(self, points: np.ndarray) -> Plane | None:
        if points.shape[0] < self.min_ground_points:
            return None

        sample_count = min(points.shape[0], self.ransac_sample_points)
        if sample_count < points.shape[0]:
            sample_indices = self.rng.choice(points.shape[0], sample_count, replace=False)
            samples = points[sample_indices]
        else:
            samples = points

        best_inliers: np.ndarray | None = None
        best_count = 0
        for _ in range(self.ransac_iterations):
            triple = samples[self.rng.choice(samples.shape[0], 3, replace=False)]
            normal = np.cross(triple[1] - triple[0], triple[2] - triple[0])
            norm = float(np.linalg.norm(normal))
            if norm < 1.0e-6:
                continue
            normal = normal / norm
            if normal[1] > 0.0:
                normal = -normal
            if abs(float(normal[1])) < self.min_ground_normal_y_abs:
                continue
            offset = -float(np.dot(normal, triple[0]))
            distances = np.abs(samples @ normal + offset)
            inliers = distances <= self.ground_distance_threshold_m
            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_inliers = inliers

        if best_inliers is None or best_count < self.min_ground_points:
            return None

        inlier_points = samples[best_inliers]
        centroid = np.mean(inlier_points, axis=0)
        _, _, vt = np.linalg.svd(inlier_points - centroid, full_matrices=False)
        normal = vt[-1].astype(np.float32)
        normal /= max(1.0e-6, float(np.linalg.norm(normal)))
        if normal[1] > 0.0:
            normal = -normal
        offset = -float(np.dot(normal, centroid))
        errors = np.abs(inlier_points @ normal + offset)
        return Plane(
            normal=normal,
            offset=offset,
            inliers=int(inlier_points.shape[0]),
            mean_error=float(np.mean(errors)),
        )

    def _confidence(
        self, area: int, width_m: float, height_m: float, edge_points: int, plane: Plane
    ) -> float:
        height_half_range = max(
            0.005, (self.step_height_max_m - self.step_height_min_m) * 0.5
        )
        height_score = 1.0 - abs(height_m - self.target_step_height_m) / height_half_range
        height_score = clamp(height_score, 0.0, 1.0)
        area_score = clamp(float(area) / float(max(1, self.min_step_pixels * 3)), 0.0, 1.0)
        width_score = clamp(width_m / max(0.01, self.min_step_width_m * 1.5), 0.0, 1.0)
        edge_score = clamp(float(edge_points) / float(max(1, self.min_step_width_px * 3)), 0.0, 1.0)
        plane_score = clamp(1.0 - plane.mean_error / max(0.002, self.ground_distance_threshold_m), 0.0, 1.0)
        return float(
            0.30 * height_score
            + 0.20 * area_score
            + 0.20 * width_score
            + 0.15 * edge_score
            + 0.15 * plane_score
        )

    def _publish_detection(
        self, source: Image, candidate: StepCandidate | None, confirm_count: int
    ) -> None:
        msg = StepDetection()
        msg.header = source.header
        msg.detected = candidate is not None and confirm_count >= self.confirm_frames
        msg.distance = float(candidate.distance) if candidate is not None else float("nan")
        msg.height = float(candidate.height) if candidate is not None else float("nan")
        msg.confidence = float(candidate.confidence) if candidate is not None else 0.0
        msg.pose_valid = bool(candidate.pose_valid) if candidate is not None else False
        msg.edge_angle_rad = (
            float(candidate.edge_angle_rad) if candidate is not None else float("nan")
        )
        msg.lateral_offset_m = (
            float(candidate.lateral_offset_m) if candidate is not None else float("nan")
        )
        msg.confirm_frames = int(confirm_count)
        msg.required_confirm_frames = int(self.confirm_frames)
        self.detection_pub.publish(msg)

    def _log_detection(
        self, candidate: StepCandidate | None, plane: Plane | None, detected: bool
    ) -> None:
        now = self.get_clock().now().nanoseconds * 1.0e-9
        if now - self.last_terminal_log_time < self.terminal_log_period_s:
            return
        self.last_terminal_log_time = now

        if plane is None:
            self.get_logger().info(
                "DETECTOR no-plane: 地面拟合失败，等待更清晰的地面深度点"
            )
            return

        if candidate is None:
            self.get_logger().info(
                "DETECTOR no-step: ground_inliers=%d mean_err=%.3fm confirm=%d/%d"
                % (
                    plane.inliers,
                    plane.mean_error,
                    min(self.confirm_count, self.confirm_frames),
                    self.confirm_frames,
                )
            )
            return

        state = "STEP" if detected else "candidate"
        self.get_logger().info(
            "DETECTOR %s: d=%.3fm h=%.3fm conf=%.2f edge=%s lat=%+.3fm confirm=%d/%d ground_err=%.3fm"
            % (
                state,
                candidate.distance,
                candidate.height,
                candidate.confidence,
                (
                    "%.1fdeg" % math.degrees(candidate.edge_angle_rad)
                    if candidate.pose_valid
                    else "invalid"
                ),
                candidate.lateral_offset_m,
                min(self.confirm_count, self.confirm_frames),
                self.confirm_frames,
                plane.mean_error,
            )
        )

    def _open_file_log(self) -> None:
        if not self.enable_file_log:
            return
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.log_path = os.path.join(self.log_dir, f"{stamp}_detector.csv")
            self.log_file = open(self.log_path, "w", newline="", buffering=1)
            self.log_writer = csv.writer(self.log_file)
            self.log_writer.writerow(
                [
                    "wall_time_s",
                    "elapsed_s",
                    "detector_state",
                    "detected",
                    "candidate",
                    "distance_m",
                    "height_m",
                    "confidence",
                    "pose_valid",
                    "edge_angle_rad",
                    "lateral_offset_m",
                    "edge_fit_rmse_m",
                    "confirm_frames",
                    "required_confirm_frames",
                    "plane_ok",
                    "ground_inliers",
                    "ground_mean_error_m",
                    "ground_nx",
                    "ground_ny",
                    "ground_nz",
                    "step_height_min_m",
                    "step_height_max_m",
                    "min_confidence",
                ]
            )
            self.get_logger().warn(f"DETECTOR CSV 日志: {self.log_path}")
        except OSError as error:
            self.log_file = None
            self.log_writer = None
            self.get_logger().error(f"无法创建 DETECTOR CSV 日志: {error}")

    def _write_file_log(
        self, candidate: StepCandidate | None, plane: Plane | None, detected: bool
    ) -> None:
        if self.log_writer is None:
            return

        if plane is None:
            state = "no-plane"
            plane_ok = 0
            ground_inliers = 0
            ground_mean_error = float("nan")
            nx = ny = nz = float("nan")
        else:
            state = "step" if detected else ("candidate" if candidate is not None else "no-step")
            plane_ok = 1
            ground_inliers = int(plane.inliers)
            ground_mean_error = float(plane.mean_error)
            nx = float(plane.normal[0])
            ny = float(plane.normal[1])
            nz = float(plane.normal[2])

        if candidate is None:
            distance = float("nan")
            height = float("nan")
            confidence = 0.0
            candidate_flag = 0
            pose_valid = 0
            edge_angle = lateral_offset = edge_rmse = float("nan")
        else:
            distance = float(candidate.distance)
            height = float(candidate.height)
            confidence = float(candidate.confidence)
            candidate_flag = 1
            pose_valid = 1 if candidate.pose_valid else 0
            edge_angle = float(candidate.edge_angle_rad)
            lateral_offset = float(candidate.lateral_offset_m)
            edge_rmse = float(candidate.edge_fit_rmse_m)

        now = time.monotonic()
        self.log_writer.writerow(
            [
                f"{time.time():.6f}",
                f"{now - self.run_started_mono:.6f}",
                state,
                1 if detected else 0,
                candidate_flag,
                f"{distance:.6f}",
                f"{height:.6f}",
                f"{confidence:.6f}",
                pose_valid,
                f"{edge_angle:.6f}",
                f"{lateral_offset:.6f}",
                f"{edge_rmse:.6f}",
                int(min(self.confirm_count, self.confirm_frames)),
                int(self.confirm_frames),
                plane_ok,
                ground_inliers,
                f"{ground_mean_error:.6f}",
                f"{nx:.6f}",
                f"{ny:.6f}",
                f"{nz:.6f}",
                f"{self.step_height_min_m:.6f}",
                f"{self.step_height_max_m:.6f}",
                f"{self.min_confidence:.6f}",
            ]
        )

    def close_log(self) -> None:
        if self.log_file is not None:
            self.get_logger().warn(f"DETECTOR CSV 日志已保存: {self.log_path}")
            self.log_file.close()
            self.log_file = None
            self.log_writer = None

    def _publish_debug_image(
        self,
        source: Image,
        depth: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        candidate: StepCandidate | None,
        plane: Plane | None,
        detected: bool,
    ) -> None:
        frame = self._depth_visualization(depth)
        if plane is not None:
            points, pixels_u, pixels_v, _, _ = self._sample_points(
                depth,
                intrinsics,
                self.roi_x_min,
                self.roi_x_max,
                self.roi_y_min,
                self.roi_y_max,
            )
            if points.size > 0:
                signed_height = points @ plane.normal + plane.offset
                ground_mask = np.abs(signed_height) <= self.ground_distance_threshold_m
                self._paint_sample_mask(
                    frame,
                    pixels_u[ground_mask],
                    pixels_v[ground_mask],
                    (30, 180, 30),
                    1,
                )

        if candidate is not None:
            x0 = int(clamp(self.roi_x_min, 0.0, 1.0) * depth.shape[1])
            y0 = int(clamp(self.roi_y_min, 0.0, 1.0) * depth.shape[0])
            selected_grid = np.argwhere(candidate.mask > 0)
            for row, col in selected_grid:
                u = x0 + int(col) * self.sample_stride
                v = y0 + int(row) * self.sample_stride
                cv2.circle(frame, (u, v), 2, (0, 210, 255), -1)

            edge_grid = np.argwhere(candidate.edge_mask > 0)
            edge_pixels: list[tuple[int, int]] = []
            for row, col in edge_grid:
                u = x0 + int(col) * self.sample_stride
                v = y0 + int(row) * self.sample_stride
                edge_pixels.append((u, v))
                cv2.circle(frame, (u, v), 3, (0, 0, 255), -1)
            if edge_pixels:
                us = [p[0] for p in edge_pixels]
                vs = [p[1] for p in edge_pixels]
                cv2.line(
                    frame,
                    (min(us), int(np.median(vs))),
                    (max(us), int(np.median(vs))),
                    (255, 255, 255),
                    2,
                )

        self._draw_roi(frame, depth.shape)
        if plane is not None:
            normal = plane.normal
            text = "ground n=[%.2f %.2f %.2f] inliers=%d" % (
                normal[0],
                normal[1],
                normal[2],
                plane.inliers,
            )
            cv2.putText(frame, text, (12, frame.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 255, 220), 1, cv2.LINE_AA)

        state = "STEP" if detected else "NO STEP"
        color = (0, 0, 255) if detected else (190, 190, 190)
        cv2.putText(frame, state, (14, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
        if candidate is not None:
            cv2.putText(
                frame,
                "d=%.2fm h=%.3fm conf=%.2f %d/%d"
                % (
                    candidate.distance,
                    candidate.height,
                    candidate.confidence,
                    min(self.confirm_count, self.confirm_frames),
                    self.confirm_frames,
                ),
                (14, 62),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            pose_text = (
                "edge=%+.1fdeg lateral=%+.2fm rmse=%.3fm"
                % (
                    math.degrees(candidate.edge_angle_rad),
                    candidate.lateral_offset_m,
                    candidate.edge_fit_rmse_m,
                )
                if candidate.pose_valid
                else "edge pose invalid"
            )
            cv2.putText(
                frame,
                pose_text,
                (14, 86),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        image = self._bgr_to_image(frame, source.header)
        self.debug_pub.publish(image)

    def _depth_visualization(self, depth: np.ndarray) -> np.ndarray:
        clipped = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        clipped = np.clip(clipped, self.min_distance_m, self.visualization_max_distance_m)
        scaled = ((self.visualization_max_distance_m - clipped) / (self.visualization_max_distance_m - self.min_distance_m) * 255.0).astype(np.uint8)
        scaled[~np.isfinite(depth)] = 0
        return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)

    def _paint_sample_mask(
        self, frame: np.ndarray, pixels_u: np.ndarray, pixels_v: np.ndarray, color: tuple[int, int, int], radius: int
    ) -> None:
        for u, v in zip(pixels_u.astype(np.int32), pixels_v.astype(np.int32)):
            cv2.circle(frame, (int(u), int(v)), radius, color, -1)

    def _draw_roi(self, frame: np.ndarray, shape: tuple[int, int]) -> None:
        height, width = shape
        x0 = int(clamp(self.roi_x_min, 0.0, 1.0) * width)
        x1 = int(clamp(self.roi_x_max, 0.0, 1.0) * width)
        y0 = int(clamp(self.roi_y_min, 0.0, 1.0) * height)
        y1 = int(clamp(self.roi_y_max, 0.0, 1.0) * height)
        gy0 = int(clamp(self.ground_roi_y_min, 0.0, 1.0) * height)
        gy1 = int(clamp(self.ground_roi_y_max, 0.0, 1.0) * height)
        cv2.rectangle(frame, (x0, y0), (x1, y1), (255, 255, 255), 1)
        cv2.rectangle(frame, (x0, gy0), (x1, gy1), (30, 180, 30), 1)

    def _bgr_to_image(self, frame: np.ndarray, header) -> Image:
        if not frame.flags["C_CONTIGUOUS"]:
            frame = np.ascontiguousarray(frame)
        msg = Image()
        msg.header = header
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * 3)
        msg.data = frame.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StepDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_log()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
