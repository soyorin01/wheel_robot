#!/usr/bin/env python3
"""室外赛道：双边黄线巡线、人行道停车、隧道下蹲、终点停车。"""

from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState


# ======================== 现场调参（优先改这里） ========================
# 改完保存后重新启动 shiwai 即可，不必改 yaml。
# 赛道两道黄泡沫内侧间距约 84 cm，轮足左右轮距 43 cm。
# 车走正中时：左轮在中线左侧 21.5 cm，右轮在中线右侧 21.5 cm。
# 单边桥在右侧，右轮会压上桥面，左轮仍在赛道地面，不要再往桥上偏。

TRACK_WIDTH_CM = 84.0
WHEEL_TRACK_CM = 43.0

# 过单边桥时相对黄泡沫中点的偏移（厘米）。
# 正数往左，负数往右。桥在右侧：右轮上桥、左轮在地面。
# 从左侧棱滑下去就把负数再加大（更往右），例如 -12。
BRIDGE_OFFSET_CM = -10.0

# 过单边桥时的横滚（度）。负数为左倾（左腿压低、右腿抬高），正数为右倾。
# 桥在右侧：左横滚让右轮更容易上到桥面。下桥后自动回到 0。
BRIDGE_ROLL_DEG = 15.0
# 横滚变化速度（度/秒）。从 0 到 15° 大约 0.5 秒。
ROLL_RATE_DEG = 30.0
# 程序启动后这么多秒才允许左横滚，避免一出发就歪。
BRIDGE_ROLL_DELAY_SECONDS = 4.0

# 只看到一侧黄线时，虚拟中点相对那条内沿的假定半宽（× 画面宽度）。
# 左：虚拟中点 = 左内沿 + 半宽。太小贴左，太大贴右。
# 右：虚拟中点 = 右内沿 − 半宽。太小贴右，太大贴左。
SINGLE_LEFT_HALF_RATIO = 0.38
SINGLE_RIGHT_HALF_RATIO = 0.38
TUNNEL_SINGLE_LEFT_HALF_RATIO = 0.50
# 隧道里现在只能看到右侧海绵：半宽太小会贴右。还贴右就再加大，例如 0.52、0.55。
TUNNEL_SINGLE_RIGHT_HALF_RATIO = 0.50
# 过完单边桥进大弯：只顺着左侧外沿海绵走。
# 半宽太小贴左海绵，太大会往内圈偏。
TURN_SINGLE_LEFT_HALF_RATIO = 0.22
TURN_SINGLE_RIGHT_HALF_RATIO = 0.42

# 下桥后相对中点的偏移。大弯改跟左线，不再额外左拉。
TURN_OFFSET_CM = 0.0

# 下桥后先停稳，再向左慢转找左侧海绵。正数为左转（w>0）。
# 停车后必须先左转够时间，不能马上把内圈黄块当成左线锁住。
TURN_STOP_SECONDS = 0.6
TURN_MIN_SPIN_SECONDS = 1.6
TURN_SEARCH_SECONDS = 5.0
TURN_LOST_GRACE_SECONDS = 9.0
TURN_LOST_SPEED = 0.0
TURN_LOST_ANGULAR = 0.22
# 锁左海绵时，其内沿必须在画面左侧。内圈黄块在右边，不能当左线。
TURN_LEFT_MAX_X_RATIO = 0.42

# 锁到左线后的速度比例（相对 FORWARD_SPEED），顺着左海绵慢速过弯。
TURN_SPEED_SCALE = 0.55

# 过完桥允许用更远处的黄线重新锁线（0=搜索框顶，1=框底）。
TURN_NEAR_LOCK_RATIO = 0.08

# 巡线搜索框（0=画面顶，1=画面底）。
# 框顶要能看到弯道远处黄线，框底要能看到脚下海绵。
SEARCH_TOP_RATIO = 0.24
SEARCH_BOTTOM_RATIO = 0.86
# 搜索框内：只有靠近车的这一段允许“新锁线”（0=框顶远处，1=框底近处）。
NEAR_LOCK_RATIO = 0.35

# ---------- 速度 ----------
# 直线巡线目标速度（m/s）。整体加快或减慢主要改这个。
FORWARD_SPEED = 0.30

# 转弯自动降速后的最低前进速度（m/s）。
MINIMUM_FORWARD_SPEED = 0.20

# 最大转向角速度（rad/s）。弯道转不过去就略增大。
MAX_ANGULAR_SPEED = 0.40

# 转弯减速比例，范围 0~1。越大，转弯时越慢。
CURVE_SLOWDOWN = 0.60

# 看见单边桥黄黑胶带后冲上桥面的速度（m/s），应比 FORWARD_SPEED 大。
BRIDGE_CLIMB_SPEED = 0.50

# 冲桥加速时长，从“真正靠近桥面”重新计时，不是从远处看见胶带开始。
# 只加总时长没用：远处看见就会倒计时，走到桥头刚好加速结束，没动力上桥。
BRIDGE_CLIMB_SECONDS = 7.0

# 看见桥之后至少这么多秒才允许判定“桥已离开”，避免胶带闪一下就取消右偏。
BRIDGE_MIN_HOLD_SECONDS = 2.5

# 靠近桥面后再计时，到点才取消右偏并停车找弯道。加速和右偏分开。
# 6 秒会带着右偏进弯压内圈；太短会从桥左边掉下去。
BRIDGE_PASS_SECONDS = 3.2

# 黄黑胶带进入画面这个高度范围，才算“靠近桥面”（0=画面顶，1=画面底）。
# 框顶太靠上会过早开始计时，走到桥头加速/右偏已经结束。
BRIDGE_NEAR_TOP_RATIO = 0.52
BRIDGE_NEAR_BOTTOM_RATIO = 0.92

# 冲桥加速结束后、右偏还没取消时的速度比例（相对 FORWARD_SPEED）。
# 防止 6 秒加速用完人还在桥上，掉回 0.20 又没动力。
BRIDGE_ON_RAMP_SPEED_SCALE = 1.50

# 已经取消右偏、到达人行道之前的速度比例。
BRIDGE_SPEED_SCALE = 1.00

# 隧道下蹲通过时的速度比例。
TUNNEL_SPEED_SCALE = 2.0

# ---------- 人行道 ----------
# 停稳后再计时的秒数。规则要求停车 5 秒。
SIDEWALK_STOP_SECONDS = 5.0

# 斑马线搜索框（0=画面顶远处，1=画面底近处）。
# TOP 越大，越要斑马线靠近车才停车。太小会在进入停车区之前就停。
CROSSWALK_TOP_RATIO = 0.72
CROSSWALK_BOTTOM_RATIO = 0.94
CROSSWALK_CONFIRM_FRAMES = 6.0
MIN_CROSSWALK_STRIPES = 5

# ---------- 腿高 / 隧道下蹲 ----------
# 正常站立腿高（米）。底盘默认约 0.25，允许范围大约 0.14~0.36。
STAND_HEIGHT = 0.25

# 过隧道时的下蹲腿高（米）。越小蹲得越明显，不要低于 0.14。
CROUCH_HEIGHT = 0.16

# 升降腿速度（m/s）。太大容易前后晃。
HEIGHT_RATE = 0.20

# 从开始下蹲算起，至少蹲这么多秒才允许起身。
MIN_CROUCH_SECONDS = 4.0

# 隧道入口黄黑胶带离开画面后，再继续蹲行这么多秒，避免洞里起身。
TUNNEL_PASS_SECONDS =0.1

# 最长下蹲时间（秒）。到点强制起身，防止一直蹲着。
MAX_CROUCH_SECONDS = 14.0

# 出隧道后丢线宽限（秒）。洞口反光容易丢黄线，3 秒后再停车。
POST_TUNNEL_LOST_GRACE_SECONDS = 1.5

# ---------- 终点 ----------
# 终点是地上三个大黑字，不是斑马线。过完隧道后认到就停车。
# TOP 越大越靠近车才停。太小会在终点前就停，太大可能压过终点才停。
FINISH_TOP_RATIO = 0.80
FINISH_BOTTOM_RATIO = 0.96
FINISH_CONFIRM_FRAMES = 6.0
# 黑字必须落到搜索框下部（靠近车）才算到达，只看见远处大字不算。
FINISH_NEAR_RATIO = 0.40
# 出隧道后再走这么多秒才允许认终点，避免把隧道口虚线/远处大字当成终点。
FINISH_AFTER_TUNNEL_SECONDS = 8.0
# 认出终点后再往前拱这么久再停，把车身开进终点带。太短会停在字前，太长会冲出垫子。
FINISH_DRIVE_IN_SECONDS = 2.0
FINISH_DRIVE_IN_SPEED = 0.14

# ---------- 识别画面录像 ----------
# 默认把带框的调试画面录下来，跑完一圈后把程序停掉即可。
RECORD_DEBUG_VIDEO = True
RECORDINGS_DIR = str(Path.home() / "wheel_robot" / "recordings")
# HDMI 本机屏：True=调试窗全屏。录像仍是原始 640x480，不受影响。
DEBUG_FULLSCREEN = True


@dataclass
class DetectionResult:
    valid: bool
    error: float
    confidence: float
    reason: str
    debug_frame: np.ndarray
    lane_width_pixels: float = 0.0
    left_line_rows: int = 0
    right_line_rows: int = 0
    left_edge_x: float = 0.0


@dataclass
class SceneObservation:
    crosswalk: bool
    hazard: bool
    hazard_near: bool
    finish: bool
    crosswalk_score: float
    hazard_score: float
    finish_score: float
    stripe_count: int
    finish_blobs: int


class DualYellowLineDetector:
    """在画面下部寻找成对的黄色泡沫边界，并跟随其中点。"""

    def __init__(
        self,
        search_top_ratio: float,
        search_bottom_ratio: float,
        search_left_ratio: float,
        search_right_ratio: float,
        yellow_h_min: int,
        yellow_h_max: int,
        yellow_s_min: int,
        yellow_v_min: int,
        black_value_max: int,
        black_saturation_max: int,
        min_boundary_lane_contrast: float,
        morphology_kernel: int,
        min_line_width_ratio: float,
        max_line_width_ratio: float,
        expected_line_width_ratio: float,
        width_tolerance_ratio: float,
        min_lane_width_ratio: float,
        max_lane_width_ratio: float,
        expected_lane_width_ratio: float,
        lane_width_tolerance_ratio: float,
        max_center_jump_ratio: float,
        min_valid_row_fraction: float,
        min_confidence: float,
        target_offset_pixels: float,
    ) -> None:
        self.search_top_ratio = search_top_ratio
        self.search_bottom_ratio = search_bottom_ratio
        self.search_left_ratio = search_left_ratio
        self.search_right_ratio = search_right_ratio
        self.yellow_h_min = yellow_h_min
        self.yellow_h_max = yellow_h_max
        self.yellow_s_min = yellow_s_min
        self.yellow_v_min = yellow_v_min
        self.black_value_max = black_value_max
        self.black_saturation_max = black_saturation_max
        self.min_boundary_lane_contrast = min_boundary_lane_contrast
        self.morphology_kernel = morphology_kernel
        self.min_line_width_ratio = min_line_width_ratio
        self.max_line_width_ratio = max_line_width_ratio
        self.expected_line_width_ratio = expected_line_width_ratio
        self.width_tolerance_ratio = width_tolerance_ratio
        self.min_lane_width_ratio = min_lane_width_ratio
        self.max_lane_width_ratio = max_lane_width_ratio
        self.expected_lane_width_ratio = expected_lane_width_ratio
        self.lane_width_tolerance_ratio = lane_width_tolerance_ratio
        self.max_center_jump_ratio = max_center_jump_ratio
        self.min_valid_row_fraction = min_valid_row_fraction
        self.min_confidence = min_confidence
        self.target_offset_pixels = target_offset_pixels
        self.single_left_half_ratio = SINGLE_LEFT_HALF_RATIO
        self.single_right_half_ratio = SINGLE_RIGHT_HALF_RATIO
        self.near_lock_ratio = NEAR_LOCK_RATIO
        self.min_single_coverage = 0.22
        self.base_min_valid_row_fraction = min_valid_row_fraction
        self.base_min_single_coverage = 0.22
        self.base_near_lock_ratio = NEAR_LOCK_RATIO
        self.force_left_line = False
        self._hud_cx: Optional[float] = None
        self._hud_cy: Optional[float] = None

    @staticmethod
    def _runs(row: np.ndarray) -> List[Tuple[int, int]]:
        padded = np.pad((row > 0).astype(np.int8), (1, 1))
        changes = np.diff(padded)
        return list(
            zip(
                np.flatnonzero(changes == 1).tolist(),
                np.flatnonzero(changes == -1).tolist(),
            )
        )

    @staticmethod
    def _smooth_xs(xs: List[float], window: int = 7) -> List[float]:
        if len(xs) < 3:
            return xs
        radius = max(1, window // 2)
        smoothed: List[float] = []
        for index in range(len(xs)):
            lo = max(0, index - radius)
            hi = min(len(xs), index + radius + 1)
            smoothed.append(float(np.mean(xs[lo:hi])))
        return smoothed

    @staticmethod
    def _mask_inner_points(
        mask: np.ndarray, x0: int, y0: int, inner: str, step: int = 5
    ) -> List[Tuple[int, int]]:
        points: List[Tuple[int, int]] = []
        height = mask.shape[0]
        for row_y in range(height - 1, -1, -step):
            cols = np.flatnonzero(mask[row_y] > 0)
            if cols.size == 0:
                continue
            col = int(cols[-1] if inner == "right" else cols[0])
            points.append((x0 + col, y0 + row_y))
        return points

    def _draw_path(
        self,
        debug: np.ndarray,
        points: List[Tuple[float, int]],
        y0: int,
        color: Tuple[int, int, int],
        thickness: int = 2,
    ) -> None:
        if len(points) < 2:
            return
        ordered = sorted(points, key=lambda item: item[1])
        step = max(1, len(ordered) // 28)
        sampled = ordered[::step]
        if sampled[-1] != ordered[-1]:
            sampled.append(ordered[-1])
        xs = self._smooth_xs([item[0] for item in sampled], 5)
        polyline = np.array(
            [(int(x), y0 + int(y)) for x, (_, y) in zip(xs, sampled)],
            dtype=np.int32,
        )
        cv2.polylines(debug, [polyline], False, color, thickness, cv2.LINE_AA)

    def _yellow_mask(self, hsv: np.ndarray) -> np.ndarray:
        yellow = cv2.inRange(
            hsv,
            np.array(
                (self.yellow_h_min, self.yellow_s_min, self.yellow_v_min),
                dtype=np.uint8,
            ),
            np.array((self.yellow_h_max, 255, 255), dtype=np.uint8),
        )
        # 黄黑相间胶带会把巡线中点拉向单边桥，必须从黄线掩膜里去掉。
        # 下桥后侧面胶带仍贴在画面右侧，核太小会把胶带黄条当成右黄线，中点被拉去内圈。
        black = cv2.inRange(
            hsv,
            np.array((0, 0, 0), dtype=np.uint8),
            np.array(
                (180, self.black_saturation_max, self.black_value_max),
                dtype=np.uint8,
            ),
        )
        near_black = cv2.dilate(black, np.ones((15, 15), dtype=np.uint8))
        striped = cv2.dilate(
            cv2.bitwise_and(yellow, near_black),
            np.ones((21, 21), dtype=np.uint8),
        )
        return cv2.bitwise_and(yellow, cv2.bitwise_not(striped))

    def detect(self, frame: np.ndarray) -> DetectionResult:
        height, width = frame.shape[:2]
        debug = np.zeros((height, width, 3), dtype=np.uint8)
        x0 = int(width * self.search_left_ratio)
        x1 = int(width * self.search_right_ratio)
        y0 = int(height * self.search_top_ratio)
        y1 = int(height * self.search_bottom_ratio)
        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            return DetectionResult(False, 0.0, 0.0, "empty_roi", debug)

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = self._yellow_mask(hsv)
        if self.morphology_kernel > 1:
            kernel = np.ones(
                (self.morphology_kernel, self.morphology_kernel), dtype=np.uint8
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        roi_height, roi_width = mask.shape
        min_line_width = max(2, int(roi_width * self.min_line_width_ratio))
        max_line_width = max(
            min_line_width + 1, int(roi_width * self.max_line_width_ratio)
        )
        expected_line_width = roi_width * self.expected_line_width_ratio
        line_tolerance = max(1.0, roi_width * self.width_tolerance_ratio)
        min_lane_width = roi_width * self.min_lane_width_ratio
        max_lane_width = roi_width * self.max_lane_width_ratio
        expected_lane_width = roi_width * self.expected_lane_width_ratio
        lane_tolerance = max(1.0, roi_width * self.lane_width_tolerance_ratio)
        max_jump_per_row = max(1.0, roi_width * self.max_center_jump_ratio)
        target_x = 0.5 * width + self.target_offset_pixels

        left_mask = np.zeros_like(mask)
        right_mask = np.zeros_like(mask)
        lane_centers: List[float] = []
        lane_weights: List[float] = []
        lane_center_points: List[Tuple[float, int]] = []
        lane_widths: List[float] = []
        boundary_widths: List[float] = []
        boundary_lane_contrasts: List[float] = []
        previous_center: Optional[float] = None
        previous_row: Optional[int] = None
        pair_rows = 0
        single_rows = 0

        for row_y in range(roi_height - 1, -1, -1):
            runs = []
            for start, end in self._runs(mask[row_y]):
                run_width = end - start
                if min_line_width <= run_width <= max_line_width:
                    runs.append((start, end, run_width))
            if not runs:
                continue
            # 近处还没锁到线时，不要用画面上部的场外黄带当赛道。
            if previous_center is None and row_y < int(
                roi_height * self.near_lock_ratio
            ):
                continue

            reference = previous_center if previous_center is not None else target_x
            row_gap = 1 if previous_row is None else max(1, previous_row - row_y)
            allowed_jump = min(0.38 * roi_width, max_jump_per_row * row_gap)
            center = None
            lane_width = 0.0
            contrast = 0.0
            weight = 1.0 + 1.4 * (row_y / max(1, roi_height - 1))
            used_pair = False

            if self.force_left_line:
                start, end, run_width = min(runs, key=lambda item: item[0])
                half_lane = self.single_left_half_ratio * width
                candidate = x0 + float(end - 1) + half_lane
                candidate = float(np.clip(candidate, x0 + 8, x1 - 8))
                if previous_center is None or abs(candidate - previous_center) <= (
                    1.35 * allowed_jump
                ):
                    center = candidate
                    lane_width = expected_lane_width
                    weight *= 0.70
                    left_mask[row_y, start:end] = 255
                    boundary_widths.append(run_width)
            elif len(runs) >= 2:
                best_pair = None
                for left_idx, left in enumerate(runs[:-1]):
                    for right in runs[left_idx + 1 :]:
                        left_inner = x0 + float(left[1] - 1)
                        right_inner = x0 + float(right[0])
                        pair_width = right_inner - left_inner
                        if not (min_lane_width <= pair_width <= max_lane_width):
                            continue
                        candidate = 0.5 * (left_inner + right_inner)
                        if (
                            previous_center is not None
                            and abs(candidate - previous_center) > allowed_jump
                        ):
                            continue
                        pair_contrast = 0.0
                        corridor = hsv[row_y, left[1] : right[0], 1]
                        if corridor.size > 0:
                            boundary_sat = 0.5 * (
                                float(np.mean(hsv[row_y, left[0] : left[1], 1]))
                                + float(np.mean(hsv[row_y, right[0] : right[1], 1]))
                            )
                            pair_contrast = boundary_sat - float(
                                np.median(corridor)
                            )
                        if pair_contrast < self.min_boundary_lane_contrast:
                            continue
                        width_err = abs(pair_width - expected_lane_width)
                        if best_pair is None or width_err < best_pair[0]:
                            best_pair = (
                                width_err,
                                left,
                                right,
                                pair_width,
                                pair_contrast,
                                candidate,
                            )
                if best_pair is not None:
                    (
                        _width_err,
                        left,
                        right,
                        lane_width,
                        contrast,
                        candidate,
                    ) = best_pair
                    center = candidate
                    used_pair = True
                    left_mask[row_y, left[0] : left[1]] = 255
                    right_mask[row_y, right[0] : right[1]] = 255
                    boundary_widths.extend((left[2], right[2]))

            if center is None and not self.force_left_line and len(runs) >= 1:
                start, end, run_width = (
                    min(runs, key=lambda item: abs(x0 + 0.5 * (item[0] + item[1]) - reference))
                )
                run_center = x0 + 0.5 * (start + end - 1)
                if run_center <= target_x:
                    # 只看到左黄线：用左内沿加上假定半宽，得到虚拟赛道中点。
                    half_lane = self.single_left_half_ratio * width
                    candidate = x0 + float(end - 1) + half_lane
                    left_mask[row_y, start:end] = 255
                else:
                    # 只看到右黄线：用右内沿减去假定半宽。
                    half_lane = self.single_right_half_ratio * width
                    candidate = x0 + float(start) - half_lane
                    right_mask[row_y, start:end] = 255
                candidate = float(np.clip(candidate, x0 + 8, x1 - 8))
                if previous_center is None or abs(candidate - previous_center) <= (
                    1.35 * allowed_jump
                ):
                    center = candidate
                    lane_width = expected_lane_width
                    contrast = max(contrast, 0.0)
                    weight *= 0.55
                    boundary_widths.append(run_width)

            if center is None:
                continue
            if used_pair:
                pair_rows += 1
            else:
                single_rows += 1
            lane_centers.append(center)
            lane_weights.append(weight)
            lane_center_points.append((center, row_y))
            lane_widths.append(lane_width)
            boundary_lane_contrasts.append(contrast)
            previous_center = center
            previous_row = row_y

        debug[y0:y1, x0:x1] = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(debug, (x0, y0), (x1, y1), (80, 80, 80), 1)
        cv2.line(
            debug,
            (int(target_x), y0),
            (int(target_x), y1),
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )

        mean_lane_width = float(np.mean(lane_widths)) if lane_widths else 0.0
        left_line_rows = int(np.count_nonzero(np.any(left_mask > 0, axis=1)))
        right_line_rows = int(np.count_nonzero(np.any(right_mask > 0, axis=1)))
        left_edge_xs = []
        for row_y in range(left_mask.shape[0]):
            cols = np.flatnonzero(left_mask[row_y] > 0)
            if cols.size:
                left_edge_xs.append(x0 + float(cols[-1]))
        left_edge_x = float(np.mean(left_edge_xs)) if left_edge_xs else 0.0
        mean_contrast = (
            float(np.mean(boundary_lane_contrasts))
            if boundary_lane_contrasts
            else 0.0
        )
        cv2.putText(
            debug,
            f"L{left_line_rows} R{right_line_rows}  "
            f"lx={left_edge_x:.0f}  "
            f"pairs={pair_rows} single={single_rows}  "
            f"lane={mean_lane_width:.0f}px",
            (max(8, x0 + 6), min(debug.shape[0] - 10, y1 + 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (180, 180, 180),
            1,
            cv2.LINE_AA,
        )

        if not lane_centers:
            return DetectionResult(
                False,
                0.0,
                0.0,
                "no_yellow_lines",
                debug,
                0.0,
                left_line_rows,
                right_line_rows,
                left_edge_x,
            )

        pair_coverage = pair_rows / roi_height
        single_coverage = single_rows / roi_height
        coverage = pair_coverage + 0.55 * single_coverage
        line_quality = float(
            np.mean(
                [
                    max(0.0, 1.0 - abs(value - expected_line_width) / line_tolerance)
                    for value in boundary_widths
                ]
            )
        ) if boundary_widths else 0.0
        lane_quality = float(
            np.mean(
                [
                    max(0.0, 1.0 - abs(value - expected_lane_width) / lane_tolerance)
                    for value in lane_widths
                ]
            )
        )
        if len(lane_centers) > 1:
            continuity = float(
                np.mean(
                    np.maximum(
                        0.0,
                        1.0 - np.abs(np.diff(lane_centers)) / max_jump_per_row,
                    )
                )
            )
        else:
            continuity = 0.0
        confidence = (
            0.45 * min(1.0, coverage / max(self.min_valid_row_fraction, 0.12))
            + 0.20 * line_quality
            + 0.20 * lane_quality
            + 0.15 * continuity
        )
        lane_center = float(
            np.average(lane_centers, weights=np.array(lane_weights, dtype=float))
        )
        error = float(
            np.clip((lane_center - target_x) / (0.5 * width), -1.0, 1.0)
        )

        look_y = y0 + int(0.62 * roi_height)
        if self._hud_cx is None:
            self._hud_cx = lane_center
            self._hud_cy = float(look_y)
        else:
            self._hud_cx = 0.78 * self._hud_cx + 0.22 * lane_center
            self._hud_cy = 0.78 * self._hud_cy + 0.22 * look_y
        cx, cy = int(self._hud_cx), int(self._hud_cy)
        cv2.line(debug, (cx - 14, cy), (cx + 14, cy), (220, 220, 220), 1, cv2.LINE_AA)
        cv2.line(debug, (cx, cy - 14), (cx, cy + 14), (220, 220, 220), 1, cv2.LINE_AA)
        cv2.circle(debug, (cx, cy), 8, (255, 255, 255), 2, cv2.LINE_AA)

        enough_pair = pair_coverage >= self.min_valid_row_fraction
        enough_single = single_coverage >= self.min_single_coverage
        enough_mix = coverage >= self.min_valid_row_fraction
        if not (enough_pair or enough_single or enough_mix):
            return DetectionResult(
                False,
                error,
                confidence,
                "insufficient_double_line",
                debug,
                mean_lane_width,
                left_line_rows,
                right_line_rows,
                left_edge_x,
            )
        if confidence < self.min_confidence:
            return DetectionResult(
                False,
                error,
                confidence,
                "low_confidence",
                debug,
                mean_lane_width,
                left_line_rows,
                right_line_rows,
                left_edge_x,
            )
        reason = "left_line" if self.force_left_line else (
            "tracking" if pair_rows > 0 else "single_line"
        )
        return DetectionResult(
            True,
            error,
            confidence,
            reason,
            debug,
            mean_lane_width,
            left_line_rows,
            right_line_rows,
            left_edge_x,
        )


class TrackSceneDetector:
    """识别人行道斑马线和黄黑相间警戒胶带。"""

    def __init__(
        self,
        yellow_h_min: int,
        yellow_h_max: int,
        yellow_s_min: int,
        yellow_v_min: int,
        black_value_max: int,
        black_saturation_max: int,
        crosswalk_top_ratio: float,
        crosswalk_bottom_ratio: float,
        hazard_top_ratio: float,
        hazard_bottom_ratio: float,
        side_margin_ratio: float,
        min_crosswalk_stripes: int,
        min_hazard_transitions: float,
    ) -> None:
        self.yellow_h_min = yellow_h_min
        self.yellow_h_max = yellow_h_max
        self.yellow_s_min = yellow_s_min
        self.yellow_v_min = yellow_v_min
        self.black_value_max = black_value_max
        self.black_saturation_max = black_saturation_max
        self.crosswalk_top_ratio = crosswalk_top_ratio
        self.crosswalk_bottom_ratio = crosswalk_bottom_ratio
        self.hazard_top_ratio = hazard_top_ratio
        self.hazard_bottom_ratio = hazard_bottom_ratio
        self.side_margin_ratio = side_margin_ratio
        self.min_crosswalk_stripes = min_crosswalk_stripes
        self.min_hazard_transitions = min_hazard_transitions

    def _masks(self, hsv: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        yellow = cv2.inRange(
            hsv,
            np.array(
                (self.yellow_h_min, self.yellow_s_min, self.yellow_v_min),
                dtype=np.uint8,
            ),
            np.array((self.yellow_h_max, 255, 255), dtype=np.uint8),
        )
        black = cv2.inRange(
            hsv,
            np.array((0, 0, 0), dtype=np.uint8),
            np.array(
                (180, self.black_saturation_max, self.black_value_max),
                dtype=np.uint8,
            ),
        )
        black = cv2.bitwise_and(black, cv2.bitwise_not(yellow))
        return yellow, black

    @staticmethod
    def _peak_count(signal: np.ndarray, threshold: float, min_gap: int) -> int:
        above = signal >= threshold
        padded = np.pad(above.astype(np.int8), (1, 1))
        changes = np.diff(padded)
        starts = np.flatnonzero(changes == 1)
        ends = np.flatnonzero(changes == -1)
        count = 0
        last_center = -1e9
        for start, end in zip(starts.tolist(), ends.tolist()):
            if end - start < 2:
                continue
            center = 0.5 * (start + end)
            if center - last_center >= min_gap:
                count += 1
                last_center = center
        return count

    def detect(
        self,
        frame: np.ndarray,
        debug: np.ndarray,
        prefer_finish: bool = False,
    ) -> SceneObservation:
        height, width = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        margin = int(width * self.side_margin_ratio)

        cw_y0 = int(height * self.crosswalk_top_ratio)
        cw_y1 = int(height * self.crosswalk_bottom_ratio)
        hz_y0 = int(height * self.hazard_top_ratio)
        hz_y1 = int(height * self.hazard_bottom_ratio)
        x0 = margin
        x1 = max(x0 + 8, width - margin)

        crosswalk, stripe_count, cw_score, cw_mask = self._detect_crosswalk(
            hsv[cw_y0:cw_y1, x0:x1]
        )
        fn_y0 = int(height * FINISH_TOP_RATIO)
        fn_y1 = int(height * FINISH_BOTTOM_RATIO)
        finish, blob_count, fn_score, blob_boxes = self._detect_finish(
            hsv[fn_y0:fn_y1, x0:x1]
        )
        # 过完隧道才认终点。之前斑马线会被大字逻辑抢走，人行道就停不了。
        if not prefer_finish:
            finish = False
        elif finish:
            crosswalk = False
        hazard, hz_score = self._detect_hazard(hsv[hz_y0:hz_y1, x0:x1])
        near_y0 = int(height * BRIDGE_NEAR_TOP_RATIO)
        near_y1 = int(height * BRIDGE_NEAR_BOTTOM_RATIO)
        hazard_near, _ = self._detect_hazard(hsv[near_y0:near_y1, x0:x1])

        roi = debug[cw_y0:cw_y1, x0:x1]
        if cw_mask.size > 1 and roi.size > 0 and cw_mask.shape[:2] == roi.shape[:2]:
            stripe = cv2.cvtColor(cw_mask, cv2.COLOR_GRAY2BGR)
            debug[cw_y0:cw_y1, x0:x1] = np.maximum(roi, stripe)
        box_tone = 220 if crosswalk else 90
        cv2.rectangle(debug, (x0, cw_y0), (x1, cw_y1), (box_tone, box_tone, box_tone), 1)
        cv2.putText(
            debug,
            f"CW {stripe_count}" + (" HIT" if crosswalk else ""),
            (x0 + 8, min(height - 12, cw_y0 + 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255) if crosswalk else (170, 170, 170),
            1,
            cv2.LINE_AA,
        )

        if prefer_finish:
            fn_tone = 220 if finish else 90
            cv2.rectangle(
                debug, (x0, fn_y0), (x1, fn_y1), (fn_tone, fn_tone, fn_tone), 1
            )
            for bx, by, bw, bh in blob_boxes:
                cv2.rectangle(
                    debug,
                    (x0 + bx, fn_y0 + by),
                    (x0 + bx + bw, fn_y0 + by + bh),
                    (220, 220, 220),
                    1,
                )
            cv2.putText(
                debug,
                f"FN {blob_count}" + (" HIT" if finish else ""),
                (x0 + 8, min(height - 12, fn_y0 + 22)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (255, 255, 255) if finish else (170, 170, 170),
                1,
                cv2.LINE_AA,
            )
        if hazard:
            cv2.putText(
                debug,
                "HAZARD",
                (width - 110, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if hazard_near:
            cv2.putText(
                debug,
                "NEAR",
                (width - 110, 52),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (220, 220, 220),
                1,
                cv2.LINE_AA,
            )
        return SceneObservation(
            crosswalk=crosswalk,
            hazard=hazard,
            hazard_near=hazard_near,
            finish=finish,
            crosswalk_score=cw_score,
            hazard_score=hz_score,
            finish_score=fn_score,
            stripe_count=stripe_count,
            finish_blobs=blob_count,
        )

    def _detect_finish(
        self, hsv_roi: np.ndarray
    ) -> Tuple[bool, int, float, List[Tuple[int, int, int, int]]]:
        empty: List[Tuple[int, int, int, int]] = []
        if hsv_roi.size == 0:
            return False, 0, 0.0, empty
        yellow, black = self._masks(hsv_roi)
        close_kernel = np.ones((11, 11), dtype=np.uint8)
        open_kernel = np.ones((5, 5), dtype=np.uint8)
        black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, close_kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, open_kernel)
        yellow_frac = float(np.mean(yellow > 0))
        black_frac = float(np.mean(black > 0))
        if black_frac < 0.07 or black_frac > 0.62 or yellow_frac > 0.20:
            return False, 0, 0.0, empty

        binary = (black > 0).astype(np.uint8)
        roi_h, roi_w = binary.shape
        near = binary[int(FINISH_NEAR_RATIO * roi_h) :, :]
        if near.size == 0 or float(np.mean(near)) < 0.08:
            return False, 0, 0.0, empty
        num, _labels, stats, _centroids = cv2.connectedComponentsWithStats(
            binary, connectivity=8
        )
        min_area = 0.012 * roi_h * roi_w
        blobs: List[Tuple[int, int, int, int, int]] = []
        for index in range(1, num):
            x = int(stats[index, cv2.CC_STAT_LEFT])
            y = int(stats[index, cv2.CC_STAT_TOP])
            w = int(stats[index, cv2.CC_STAT_WIDTH])
            h = int(stats[index, cv2.CC_STAT_HEIGHT])
            area = int(stats[index, cv2.CC_STAT_AREA])
            if area < min_area or h < 0.20 * roi_h or w < 0.06 * roi_w:
                continue
            if w > 3.2 * max(h, 1):
                continue
            blobs.append((x, y, w, h, area))

        blob_count = len(blobs)
        boxes = [(item[0], item[1], item[2], item[3]) for item in blobs]
        if blob_count == 1:
            _x, _y, w, h, area = blobs[0]
            if w < 0.42 * roi_w or h < 0.26 * roi_h or area < 0.08 * roi_h * roi_w:
                return False, blob_count, 0.0, boxes
        elif blob_count < 2 or blob_count > 6:
            return False, blob_count, 0.0, boxes

        left = min(item[0] for item in blobs)
        right = max(item[0] + item[2] for item in blobs)
        span = (right - left) / max(1, roi_w)
        if span < 0.38:
            return False, blob_count, 0.0, boxes
        bottoms = [item[1] + item[3] for item in blobs]
        if max(bottoms) < FINISH_NEAR_RATIO * roi_h:
            return False, blob_count, 0.0, boxes

        aspects = [item[2] / max(item[3], 1) for item in blobs]
        median_aspect = float(np.median(np.array(aspects, dtype=np.float32)))
        if median_aspect > 2.4:
            return False, blob_count, 0.0, boxes

        score = float(
            np.clip(
                0.40 * min(1.0, blob_count / 3.0)
                + 0.35 * min(1.0, span / 0.70)
                + 0.25 * min(1.0, black_frac / 0.28),
                0.0,
                1.0,
            )
        )
        if blob_count == 1:
            score = min(1.0, score + 0.08)
        return score >= 0.40, blob_count, score, boxes

    def _detect_crosswalk(
        self, hsv_roi: np.ndarray
    ) -> Tuple[bool, int, float, np.ndarray]:
        empty = np.zeros((1, 1), dtype=np.uint8)
        if hsv_roi.size == 0:
            return False, 0, 0.0, empty
        yellow, black = self._masks(hsv_roi)
        kernel = np.ones((3, 5), dtype=np.uint8)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_CLOSE, kernel)
        black_bin = black > 0
        yellow_frac = float(np.mean(yellow > 0))
        black_frac = float(np.mean(black_bin))
        if black_frac < 0.06 or black_frac > 0.58 or yellow_frac > 0.16:
            return False, 0, 0.0, black
        # 斑马线必须落到画面下部（靠近车），只看见远处条纹不算进入停车区。
        roi_h, roi_w = black_bin.shape
        near = black_bin[int(0.55 * roi_h) :, :]
        if near.size == 0 or float(np.mean(near)) < 0.10:
            return False, 0, 0.0, black

        row_fill = black_bin.mean(axis=1)
        col_fill = black_bin.mean(axis=0)
        row_peaks = self._peak_count(row_fill, 0.18, max(3, roi_h // 18))
        col_peaks = self._peak_count(col_fill, 0.16, max(4, roi_w // 16))
        stripe_count = max(row_peaks, col_peaks)
        if stripe_count < self.min_crosswalk_stripes:
            return False, stripe_count, 0.0, black

        span_ok = False
        if row_peaks >= self.min_crosswalk_stripes:
            active = np.flatnonzero(row_fill >= 0.18)
            if (
                active.size
                and (active[-1] - active[0]) >= 0.18 * roi_h
                and active[-1] >= 0.62 * roi_h
            ):
                span_ok = True
        if col_peaks >= self.min_crosswalk_stripes:
            active = np.flatnonzero(col_fill >= 0.16)
            if active.size and (active[-1] - active[0]) >= 0.28 * roi_w:
                span_ok = True
        if not span_ok:
            return False, stripe_count, 0.0, black

        score = float(
            np.clip(
                0.45 * min(1.0, stripe_count / 6.0)
                + 0.35 * min(1.0, black_frac / 0.28)
                + 0.20 * max(0.0, 1.0 - yellow_frac / 0.16),
                0.0,
                1.0,
            )
        )
        return score >= 0.42, stripe_count, score, black

    def _yellow_black_transitions(
        self, yellow_bin: np.ndarray, black_bin: np.ndarray, axis: int
    ) -> List[float]:
        length = yellow_bin.shape[axis]
        width = yellow_bin.shape[1 - axis]
        step = max(1, length // 28)
        window = max(24, int(0.30 * width))
        transitions: List[float] = []
        for index in range(0, length, step):
            if axis == 0:
                y_line = yellow_bin[index]
                b_line = black_bin[index]
            else:
                y_line = yellow_bin[:, index]
                b_line = black_bin[:, index]
            state = np.where(y_line, 1, np.where(b_line, 2, 0)).astype(np.int8)
            compact_idx = np.flatnonzero(state)
            if compact_idx.size < 4:
                continue
            compact = state[compact_idx]
            pair = compact[:-1] + compact[1:]
            changed = compact[1:] != compact[:-1]
            flips = changed & (pair == 3)
            if not np.any(flips):
                continue
            flip_x = compact_idx[1:][flips]
            local = 1
            start = 0
            for end in range(len(flip_x)):
                while int(flip_x[end] - flip_x[start]) > window:
                    start += 1
                local = max(local, end - start + 1)
            if local >= 2:
                transitions.append(float(local))
        return transitions

    def _detect_hazard(self, hsv_roi: np.ndarray) -> Tuple[bool, float]:
        if hsv_roi.size == 0:
            return False, 0.0
        yellow, black = self._masks(hsv_roi)
        kernel = np.ones((3, 3), dtype=np.uint8)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
        black = cv2.morphologyEx(black, cv2.MORPH_OPEN, kernel)
        yellow_bin = yellow > 0
        black_bin = black > 0
        yellow_frac = float(np.mean(yellow_bin))
        black_frac = float(np.mean(black_bin))
        if yellow_frac < 0.018 or black_frac < 0.010:
            return False, 0.0

        dilated_yellow = cv2.dilate(yellow, np.ones((7, 7), dtype=np.uint8))
        adjacent = float(np.mean((dilated_yellow > 0) & black_bin))
        if adjacent < 0.004:
            return False, 0.0

        row_transitions = self._yellow_black_transitions(yellow_bin, black_bin, 0)
        col_transitions = self._yellow_black_transitions(yellow_bin, black_bin, 1)
        mean_row = float(np.mean(row_transitions)) if row_transitions else 0.0
        mean_col = float(np.mean(col_transitions)) if col_transitions else 0.0
        mean_transitions = max(mean_row, mean_col)
        support = max(len(row_transitions), len(col_transitions))
        if mean_transitions < self.min_hazard_transitions or support < 6:
            return False, 0.0

        score = float(
            np.clip(
                0.40 * min(1.0, mean_transitions / 6.0)
                + 0.35 * min(1.0, adjacent / 0.06)
                + 0.25 * min(1.0, (yellow_frac + black_frac) / 0.20),
                0.0,
                1.0,
            )
        )
        return score >= 0.32, score


class ShiWaiFollower(Node):
    VALID_ROTATIONS = (0, 90, 180, 270)
    VALID_MIRRORS = ("none", "horizontal", "vertical", "both")
    MODE_FOLLOW = "FOLLOW"
    MODE_STOP = "SIDEWALK_STOP"
    MODE_CROUCH = "TUNNEL_CROUCH"
    MODE_FINISH = "FINISH_STOP"

    def __init__(self) -> None:
        super().__init__("shiwai")
        self._declare_parameters()
        self._load_parameters()
        self._apply_field_tuning()
        self._validate_parameters()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.posture_pub = self.create_publisher(JointState, "/cmd_posture", 10)
        self._frame_lock = threading.Lock()
        self._capture_lock = threading.Lock()
        self._stop_capture = threading.Event()
        self._frame: Optional[np.ndarray] = None
        self._frame_time = 0.0
        self._sequence = 0
        self._camera_ok = False
        self._capture: Optional[cv2.VideoCapture] = None
        self._last_sequence = -1
        self._last_command = (0.0, 0.0)
        self._last_command_valid = False
        self._last_motion_time = time.monotonic()
        self._previous_error = 0.0
        self._previous_control_time = 0.0
        self._have_error = False
        self._last_log = 0.0
        self._exit_requested = False
        self._closing = False
        self._window_ready = False

        self._mode = self.MODE_FOLLOW
        self._passed_bridge = False
        self._passed_sidewalk = False
        self._passed_tunnel = False
        self._passed_tunnel_at: Optional[float] = None
        self._passed_finish = False
        self._finish_since: Optional[float] = None
        self._bridge_started: Optional[float] = None
        self._bridge_near_since: Optional[float] = None
        self._bridge_climb_ended = False
        self._cleared_bridge = False
        self._cleared_at: Optional[float] = None
        self._left_line_locked = False
        self._turn_lost_announced = False
        self._crosswalk_frames = 0
        self._hazard_frames = 0
        self._hazard_near_frames = 0
        self._hazard_gone_frames = 0
        self._finish_frames = 0
        self._stable_since: Optional[float] = None
        self._crouch_started: Optional[float] = None
        self._tape_gone_since: Optional[float] = None
        self._lost_since: Optional[float] = None
        self._current_height = self.stand_height
        self._target_height = self.stand_height
        self._current_roll = 0.0
        self._started_at = time.monotonic()
        self._height_time = time.monotonic()
        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None
        self._video_frames = 0
        self._video_size = (int(self.image_width), int(self.image_height))
        self._hdmi_size = self._detect_hdmi_size()
        self._debug_fullscreen_applied = False

        self.line_detector = DualYellowLineDetector(
            self.search_top_ratio,
            self.search_bottom_ratio,
            self.search_left_ratio,
            self.search_right_ratio,
            self.yellow_h_min,
            self.yellow_h_max,
            self.yellow_s_min,
            self.yellow_v_min,
            self.black_value_max,
            self.black_saturation_max,
            self.min_boundary_lane_contrast,
            self.morphology_kernel,
            self.min_line_width_ratio,
            self.max_line_width_ratio,
            self.expected_line_width_ratio,
            self.width_tolerance_ratio,
            self.min_lane_width_ratio,
            self.max_lane_width_ratio,
            self.expected_lane_width_ratio,
            self.lane_width_tolerance_ratio,
            self.max_center_jump_ratio,
            self.min_valid_row_fraction,
            self.min_confidence,
            self.target_offset_pixels,
        )
        self.scene_detector = TrackSceneDetector(
            self.yellow_h_min,
            self.yellow_h_max,
            self.yellow_s_min,
            self.yellow_v_min,
            self.black_value_max,
            self.black_saturation_max,
            self.crosswalk_top_ratio,
            self.crosswalk_bottom_ratio,
            self.hazard_top_ratio,
            self.hazard_bottom_ratio,
            self.side_margin_ratio,
            self.min_crosswalk_stripes,
            self.min_hazard_transitions,
        )

        if self.show_debug:
            try:
                cv2.namedWindow(self.debug_window_name, cv2.WINDOW_NORMAL)
                if DEBUG_FULLSCREEN:
                    self._apply_debug_fullscreen()
                self._window_ready = True
            except cv2.error as error:
                self.get_logger().error(f"调试窗口创建失败: {error}")

        self._publish_zero("startup")
        self._publish_posture(self._current_height)
        self._capture_thread = threading.Thread(
            target=self._capture_loop, name="shiwai_camera", daemon=True
        )
        self._capture_thread.start()
        self._timer = self.create_timer(1.0 / self.control_rate, self._control_timer)
        mode = "运动已启用" if self.enable_motion else "只看画面不发车"
        self._announce(
            f"程序启动，{mode}。后面会打印：进桥、开始加速、加速结束、人行道停车、过隧道、终点"
        )
        self._announce(
            f"当前速度 {self.forward_speed:.2f} m/s，"
            f"冲桥 {self.bridge_climb_speed:.2f} m/s / {self.bridge_climb_seconds:.0f} 秒，"
            f"人行道停 {self.sidewalk_stop_seconds:.0f} 秒"
        )
        if RECORD_DEBUG_VIDEO:
            self._announce(
                f"识别画面会录成视频，保存在 {RECORDINGS_DIR}。"
                "跑完一圈后请停掉程序，文件才会写完"
            )

    def _declare_parameters(self) -> None:
        defaults = {
            "enable_motion": False,
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
            "image_timeout_seconds": 0.35,
            "control_rate": 15.0,
            "show_debug": True,
            "debug_window_name": "ShiWai Track (Q/Esc to stop)",
            "search_top_ratio": SEARCH_TOP_RATIO,
            "search_bottom_ratio": SEARCH_BOTTOM_RATIO,
            "search_left_ratio": 0.0,
            "search_right_ratio": 1.0,
            "yellow_h_min": 12,
            "yellow_h_max": 42,
            "yellow_s_min": 50,
            "yellow_v_min": 70,
            "min_boundary_lane_contrast": 18.0,
            "morphology_kernel": 3,
            "min_line_width_ratio": 0.012,
            "max_line_width_ratio": 0.50,
            "expected_line_width_ratio": 0.08,
            "width_tolerance_ratio": 0.08,
            "min_lane_width_ratio": 0.32,
            "max_lane_width_ratio": 0.82,
            "expected_lane_width_ratio": 0.72,
            "lane_width_tolerance_ratio": 0.30,
            "max_center_jump_ratio": 0.04,
            "min_valid_row_fraction": 0.12,
            "min_confidence": 0.38,
            "target_offset_pixels": 0.0,
            "black_value_max": 95,
            "black_saturation_max": 110,
            "crosswalk_top_ratio": 0.52,
            "crosswalk_bottom_ratio": 0.86,
            "hazard_top_ratio": 0.08,
            "hazard_bottom_ratio": 0.72,
            "side_margin_ratio": 0.03,
            "min_crosswalk_stripes": 4,
            "min_hazard_transitions": 1.8,
            "crosswalk_confirm_frames": 4.0,
            "hazard_confirm_frames": 4.0,
            "hazard_gone_frames": 12.0,
            "sidewalk_stop_seconds": SIDEWALK_STOP_SECONDS,
            "stand_height": STAND_HEIGHT,
            "crouch_height": CROUCH_HEIGHT,
            "height_rate": HEIGHT_RATE,
            "min_crouch_seconds": MIN_CROUCH_SECONDS,
            "tunnel_pass_seconds": TUNNEL_PASS_SECONDS,
            "max_crouch_seconds": MAX_CROUCH_SECONDS,
            "bridge_speed_scale": BRIDGE_SPEED_SCALE,
            "bridge_climb_speed": BRIDGE_CLIMB_SPEED,
            "bridge_climb_seconds": BRIDGE_CLIMB_SECONDS,
            "tunnel_speed_scale": TUNNEL_SPEED_SCALE,
            "lost_line_grace_seconds": 0.50,
            "tunnel_lost_line_grace_seconds": 1.80,
            "forward_speed": FORWARD_SPEED,
            "minimum_forward_speed": MINIMUM_FORWARD_SPEED,
            "linear_direction_sign": 1.0,
            "steering_kp": 1.00,
            "steering_kd": 0.025,
            "steering_sign": -1.0,
            "max_angular_speed": MAX_ANGULAR_SPEED,
            "max_linear_acceleration": 0.25,
            "max_angular_acceleration": 1.00,
            "max_error_derivative": 3.0,
            "error_filter_alpha": 0.45,
            "curve_slowdown": CURVE_SLOWDOWN,
            "status_log_period": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _load_parameters(self) -> None:
        for parameter in self._parameters.values():
            setattr(self, parameter.name, self.get_parameter(parameter.name).value)
        integer_names = (
            "camera_id",
            "image_width",
            "image_height",
            "rotation_degrees",
            "yellow_h_min",
            "yellow_h_max",
            "yellow_s_min",
            "yellow_v_min",
            "morphology_kernel",
            "black_value_max",
            "black_saturation_max",
            "min_crosswalk_stripes",
        )
        bool_names = ("enable_motion", "show_debug")
        string_names = (
            "camera_backend",
            "camera_device",
            "pixel_format",
            "mirror_mode",
            "debug_window_name",
        )
        for name in integer_names:
            setattr(self, name, int(getattr(self, name)))
        for name in bool_names:
            setattr(self, name, bool(getattr(self, name)))
        for name in string_names:
            setattr(self, name, str(getattr(self, name)))
        self.camera_backend = self.camera_backend.lower()
        self.pixel_format = self.pixel_format.upper()
        self.mirror_mode = self.mirror_mode.lower()
        for name in self._parameters:
            if name not in integer_names + bool_names + string_names:
                setattr(self, name, float(getattr(self, name)))

    def _apply_field_tuning(self) -> None:
        """文件开头的现场参数覆盖 yaml，改脚本顶部即可生效。"""
        self.forward_speed = float(FORWARD_SPEED)
        self.minimum_forward_speed = float(MINIMUM_FORWARD_SPEED)
        self.max_angular_speed = float(MAX_ANGULAR_SPEED)
        self.curve_slowdown = float(CURVE_SLOWDOWN)
        self.bridge_speed_scale = float(BRIDGE_SPEED_SCALE)
        self.bridge_climb_speed = float(BRIDGE_CLIMB_SPEED)
        self.bridge_climb_seconds = float(BRIDGE_CLIMB_SECONDS)
        self.tunnel_speed_scale = float(TUNNEL_SPEED_SCALE)
        self.sidewalk_stop_seconds = float(SIDEWALK_STOP_SECONDS)
        self.stand_height = float(STAND_HEIGHT)
        self.crouch_height = float(CROUCH_HEIGHT)
        self.height_rate = float(HEIGHT_RATE)
        self.min_crouch_seconds = float(MIN_CROUCH_SECONDS)
        self.tunnel_pass_seconds = float(TUNNEL_PASS_SECONDS)
        self.max_crouch_seconds = float(MAX_CROUCH_SECONDS)
        self.target_offset_pixels = 0.0
        self.max_lane_width_ratio = 0.82
        self.search_top_ratio = float(SEARCH_TOP_RATIO)
        self.search_bottom_ratio = float(SEARCH_BOTTOM_RATIO)
        self.crosswalk_top_ratio = float(CROSSWALK_TOP_RATIO)
        self.crosswalk_bottom_ratio = float(CROSSWALK_BOTTOM_RATIO)
        self.crosswalk_confirm_frames = float(CROSSWALK_CONFIRM_FRAMES)
        self.min_crosswalk_stripes = float(MIN_CROSSWALK_STRIPES)

    def _on_bridge_section(self) -> bool:
        return (
            self._passed_bridge
            and not self._cleared_bridge
            and not self._passed_sidewalk
        )

    def _update_bridge_target(self) -> None:
        """上桥往右偏；下桥后只跟左侧海绵过大弯。"""
        if self._on_bridge_section():
            offset_cm = BRIDGE_OFFSET_CM
        elif self._cleared_bridge and not self._passed_sidewalk:
            offset_cm = TURN_OFFSET_CM
        else:
            offset_cm = 0.0
        offset = (offset_cm / TRACK_WIDTH_CM) * (
            self.image_width * self.expected_lane_width_ratio
        )
        self.line_detector.target_offset_pixels = float(offset)
        if self._mode == self.MODE_CROUCH:
            self.line_detector.single_left_half_ratio = TUNNEL_SINGLE_LEFT_HALF_RATIO
            self.line_detector.single_right_half_ratio = TUNNEL_SINGLE_RIGHT_HALF_RATIO
            self.line_detector.near_lock_ratio = self.line_detector.base_near_lock_ratio
            self.line_detector.min_valid_row_fraction = (
                self.line_detector.base_min_valid_row_fraction
            )
            self.line_detector.min_single_coverage = (
                self.line_detector.base_min_single_coverage
            )
            self.line_detector.force_left_line = False
        elif self._cleared_bridge and not self._passed_sidewalk:
            self.line_detector.single_left_half_ratio = TURN_SINGLE_LEFT_HALF_RATIO
            self.line_detector.single_right_half_ratio = TURN_SINGLE_RIGHT_HALF_RATIO
            self.line_detector.near_lock_ratio = TURN_NEAR_LOCK_RATIO
            self.line_detector.min_valid_row_fraction = 0.06
            self.line_detector.min_single_coverage = 0.10
            self.line_detector.force_left_line = True
        else:
            self.line_detector.single_left_half_ratio = SINGLE_LEFT_HALF_RATIO
            self.line_detector.single_right_half_ratio = SINGLE_RIGHT_HALF_RATIO
            self.line_detector.near_lock_ratio = self.line_detector.base_near_lock_ratio
            self.line_detector.min_valid_row_fraction = (
                self.line_detector.base_min_valid_row_fraction
            )
            self.line_detector.min_single_coverage = (
                self.line_detector.base_min_single_coverage
            )
            self.line_detector.force_left_line = False

    def _validate_parameters(self) -> None:
        if self.camera_backend not in ("auto", "v4l2"):
            raise ValueError("camera_backend 只能是 auto 或 v4l2")
        if self.camera_id < 0 or self.image_width <= 0 or self.image_height <= 0:
            raise ValueError("摄像头编号和图像尺寸无效")
        if self.capture_frame_rate <= 0.0:
            raise ValueError("capture_frame_rate 必须大于 0")
        if self.rotation_degrees not in self.VALID_ROTATIONS:
            raise ValueError("rotation_degrees 必须是 0/90/180/270")
        if self.mirror_mode not in self.VALID_MIRRORS:
            raise ValueError("mirror_mode 无效")
        if len(self.pixel_format) != 4:
            raise ValueError("pixel_format 必须是四字符格式")
        if not 10.0 <= self.control_rate <= 20.0:
            raise ValueError("control_rate 必须在 10~20 Hz")
        if not 0 <= self.yellow_h_min < self.yellow_h_max <= 180:
            raise ValueError("黄色色调范围无效")
        if not 0 <= self.yellow_s_min <= 255 or not 0 <= self.yellow_v_min <= 255:
            raise ValueError("黄色饱和度或亮度阈值无效")
        if not 0.0 < self.min_boundary_lane_contrast <= 255.0:
            raise ValueError("min_boundary_lane_contrast 必须在 (0,255]")
        ratios = (
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
            "error_filter_alpha",
            "curve_slowdown",
            "side_margin_ratio",
            "crosswalk_top_ratio",
            "crosswalk_bottom_ratio",
            "hazard_top_ratio",
            "hazard_bottom_ratio",
        )
        for name in ratios:
            if not 0.0 < getattr(self, name) <= 1.0:
                raise ValueError(f"{name} 必须在 (0,1]")
        if not 0.0 <= self.search_top_ratio < self.search_bottom_ratio <= 1.0:
            raise ValueError("搜索区域上下比例无效")
        if not 0.0 <= self.search_left_ratio < self.search_right_ratio <= 1.0:
            raise ValueError("搜索区域左右比例无效")
        if not self.crosswalk_top_ratio < self.crosswalk_bottom_ratio:
            raise ValueError("人行道识别区域无效")
        if not self.hazard_top_ratio < self.hazard_bottom_ratio:
            raise ValueError("警戒胶带识别区域无效")
        if not self.min_line_width_ratio < self.max_line_width_ratio:
            raise ValueError("黄线宽度范围无效")
        if not (
            self.min_line_width_ratio
            <= self.expected_line_width_ratio
            <= self.max_line_width_ratio
        ):
            raise ValueError("expected_line_width_ratio 必须在线宽范围内")
        if not self.min_lane_width_ratio < self.max_lane_width_ratio:
            raise ValueError("双线间距范围无效")
        if not (
            self.min_lane_width_ratio
            <= self.expected_lane_width_ratio
            <= self.max_lane_width_ratio
        ):
            raise ValueError("expected_lane_width_ratio 必须在双线间距范围内")
        if self.linear_direction_sign not in (-1.0, 1.0):
            raise ValueError("linear_direction_sign 只能是 -1 或 1")
        if self.steering_sign not in (-1.0, 1.0):
            raise ValueError("steering_sign 只能是 -1 或 1")
        if not 0.0 <= self.minimum_forward_speed <= self.forward_speed:
            raise ValueError("前进速度范围无效")
        if not 0.14 <= self.crouch_height < self.stand_height <= 0.36:
            raise ValueError("站立高度必须高于下蹲高度，且都在 0.14~0.36 m")
        if self.height_rate <= 0.0:
            raise ValueError("height_rate 必须大于 0")
        if self.sidewalk_stop_seconds <= 0.0:
            raise ValueError("sidewalk_stop_seconds 必须大于 0")
        if not 0.0 < self.min_crouch_seconds <= self.max_crouch_seconds:
            raise ValueError("下蹲时间范围无效")
        if self.bridge_climb_speed <= 0.0:
            raise ValueError("bridge_climb_speed 必须大于 0")
        if self.bridge_climb_seconds <= 0.0:
            raise ValueError("bridge_climb_seconds 必须大于 0")
        if self.tunnel_pass_seconds <= 0.0:
            raise ValueError("tunnel_pass_seconds 必须大于 0")
        if (
            self.max_angular_speed <= 0.0
            or self.max_linear_acceleration <= 0.0
            or self.max_angular_acceleration <= 0.0
            or self.max_error_derivative <= 0.0
            or self.status_log_period <= 0.0
        ):
            raise ValueError("速度、加速度、误差变化率和日志周期必须大于 0")
        if self.steering_kp < 0.0 or self.steering_kd < 0.0:
            raise ValueError("转向增益不能为负")
        if self.camera_retry_seconds <= 0.0 or self.image_timeout_seconds <= 0.0:
            raise ValueError("摄像头重试和图像超时必须大于 0")

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
        while not self._stop_capture.is_set():
            with self._capture_lock:
                capture = self._capture
            if capture is None or not capture.isOpened():
                capture = self._open_camera()
                with self._capture_lock:
                    self._capture = capture
                if capture is None:
                    with self._frame_lock:
                        self._camera_ok = False
                        self._frame = None
                    self._publish_zero("camera_open_failed")
                    self._stop_capture.wait(self.camera_retry_seconds)
                    continue
            ok, frame = capture.read()
            if not ok or frame is None:
                with self._frame_lock:
                    self._camera_ok = False
                    self._frame = None
                self._publish_zero("camera_read_failed")
                capture.release()
                with self._capture_lock:
                    self._capture = None
                self._stop_capture.wait(self.camera_retry_seconds)
                continue
            frame = self._transform(frame)
            with self._frame_lock:
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

    def _publish(self, forward_speed: float, angular_speed: float) -> None:
        message = Twist()
        message.linear.x = self.linear_direction_sign * float(forward_speed)
        message.angular.z = float(angular_speed)
        self.cmd_vel_pub.publish(message)

    def _publish_posture(self, height: float) -> None:
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = ["joint_height", "joint_roll", "joint_pitching"]
        message.position = [
            float(np.clip(height, 0.14, 0.36)),
            float(np.clip(self._current_roll, -15.0, 15.0)),
            0.0,
        ]
        self.posture_pub.publish(message)

    def _bridge_roll_target(self) -> float:
        if not self._on_bridge_section():
            return 0.0
        if time.monotonic() - self._started_at < BRIDGE_ROLL_DELAY_SECONDS:
            return 0.0
        return float(BRIDGE_ROLL_DEG)

    def _update_height(self, now: float) -> None:
        dt = max(0.0, min(now - self._height_time, 2.0 / self.control_rate))
        self._height_time = now
        delta = self.height_rate * dt
        self._current_height = float(
            np.clip(
                self._target_height,
                self._current_height - delta,
                self._current_height + delta,
            )
        )
        self._current_roll = self._approach(
            self._current_roll,
            self._bridge_roll_target(),
            ROLL_RATE_DEG * dt,
        )
        self._publish_posture(self._current_height)

    def _publish_zero(self, _reason: str) -> None:
        try:
            self._publish(0.0, 0.0)
        except Exception:
            pass
        self._last_command = (0.0, 0.0)
        self._last_command_valid = False
        self._last_motion_time = time.monotonic()
        self._have_error = False

    @staticmethod
    def _approach(current: float, target: float, delta: float) -> float:
        return float(np.clip(target, current - delta, current + delta))

    def _rate_limit(self, linear: float, angular: float, now: float):
        dt = max(0.0, min(now - self._last_motion_time, 2.0 / self.control_rate))
        lin_step = self.max_linear_acceleration * dt
        if linear < self._last_command[0]:
            lin_step *= 3.0
        linear = self._approach(self._last_command[0], linear, lin_step)
        angular = self._approach(
            self._last_command[1], angular, self.max_angular_acceleration * dt
        )
        self._last_motion_time = now
        return linear, angular

    def _confirm(self, frames_attr: str, seen: bool, needed: float) -> bool:
        current = int(getattr(self, frames_attr))
        current = current + 1 if seen else 0
        setattr(self, frames_attr, current)
        return current >= int(needed)

    def _holding_stop(self) -> bool:
        return self._mode in (self.MODE_STOP, self.MODE_FINISH)

    def _update_mission(self, scene: SceneObservation, now: float) -> None:
        if self._holding_stop():
            return

        hazard_confirmed = self._confirm(
            "_hazard_frames", scene.hazard, self.hazard_confirm_frames
        )
        hazard_gone = self._confirm(
            "_hazard_gone_frames",
            (not scene.hazard) and self._hazard_frames == 0,
            self.hazard_gone_frames,
        )
        crosswalk_confirmed = self._confirm(
            "_crosswalk_frames",
            scene.crosswalk and not self._passed_sidewalk,
            self.crosswalk_confirm_frames,
        )

        near_confirmed = self._confirm(
            "_hazard_near_frames",
            scene.hazard_near and not self._passed_sidewalk,
            max(2, int(self.hazard_confirm_frames) - 2),
        )
        saw_bridge = (
            (hazard_confirmed or near_confirmed)
            and not self._passed_sidewalk
            and not self._cleared_bridge
        )
        if saw_bridge:
            if not self._passed_bridge:
                self._passed_bridge = True
                self._announce("进入单边桥区域：往右偏，右轮上桥")
            if self._bridge_started is None:
                self._bridge_started = now
                self._bridge_climb_ended = False
                self._announce(
                    f"开始加速接近单边桥：{self.bridge_climb_speed:.2f} m/s，"
                    f"靠近桥面后重新计时 {self.bridge_climb_seconds:.0f} 秒"
                )
            if near_confirmed and self._bridge_near_since is None:
                self._bridge_near_since = now
                self._bridge_climb_ended = False
                self._announce(
                    f"已靠近桥面，重新开始冲桥 {self.bridge_climb_speed:.2f} m/s，"
                    f"{self.bridge_climb_seconds:.0f} 秒；"
                    f"{BRIDGE_PASS_SECONDS:.1f} 秒后停车，向左找左侧海绵"
                )

        held_long_enough = (
            self._bridge_started is not None
            and (now - self._bridge_started) >= BRIDGE_MIN_HOLD_SECONDS
        )
        near_long_enough = (
            self._bridge_near_since is not None
            and (now - self._bridge_near_since) >= BRIDGE_PASS_SECONDS
        )
        fallback_long_enough = (
            self._bridge_near_since is None
            and self._bridge_started is not None
            and (now - self._bridge_started) >= max(
                self.bridge_climb_seconds, BRIDGE_PASS_SECONDS + 3.0
            )
        )
        # 远处先看见胶带时 _bridge_started 会提前开始；真正过桥要以靠近桥面为准。
        # 下桥后侧面胶带常留在画面右侧，hazard_gone 一直不成立，必须靠超时强制离开。
        if (
            self._passed_bridge
            and not self._passed_sidewalk
            and not self._cleared_bridge
            and held_long_enough
            and (hazard_gone or near_long_enough or fallback_long_enough)
        ):
            self._cleared_bridge = True
            self._cleared_at = now
            self._left_line_locked = False
            self._have_error = False
            self._mark_bridge_climb_ended("已过单边桥，停止冲桥加速")
            self._announce(
                f"已过单边桥：先停车 {TURN_STOP_SECONDS:.1f} 秒，"
                f"再向左慢转找左侧海绵，顺着左线过大弯"
            )

        if crosswalk_confirmed and not self._passed_sidewalk:
            self._mode = self.MODE_STOP
            self._stable_since = None
            self._have_error = False
            self._target_height = self.stand_height
            self._mark_bridge_climb_ended("到达人行道，单边桥加速结束")
            self._announce(
                f"识别到人行道，开始减速停车，停稳后再计时 "
                f"{self.sidewalk_stop_seconds:.0f} 秒"
            )
            return

        if (
            hazard_confirmed
            and self._passed_sidewalk
            and not self._passed_tunnel
            and self._mode != self.MODE_CROUCH
        ):
            self._mode = self.MODE_CROUCH
            self._crouch_started = now
            self._tape_gone_since = None
            self._target_height = self.crouch_height
            self._announce(
                f"开始过隧道：下蹲到 {self.crouch_height:.3f} m，减速通过"
            )
            return

        if self._mode == self.MODE_CROUCH and self._crouch_started is not None:
            crouched = now - self._crouch_started
            if hazard_gone:
                if self._tape_gone_since is None:
                    self._tape_gone_since = now
                    self._announce("隧道入口已过，继续蹲着往前走")
            else:
                self._tape_gone_since = None
            passed_interior = (
                self._tape_gone_since is not None
                and now - self._tape_gone_since >= self.tunnel_pass_seconds
                and crouched >= self.min_crouch_seconds
            )
            if crouched >= self.max_crouch_seconds or passed_interior:
                self._mode = self.MODE_FOLLOW
                self._passed_tunnel = True
                self._passed_tunnel_at = now
                self._tape_gone_since = None
                self._target_height = self.stand_height
                self._announce("已出隧道，恢复站立，继续巡线")

        tunnel_ready_for_finish = (
            self._passed_tunnel
            and self._passed_tunnel_at is not None
            and (now - self._passed_tunnel_at) >= FINISH_AFTER_TUNNEL_SECONDS
        )
        finish_confirmed = self._confirm(
            "_finish_frames",
            scene.finish
            and tunnel_ready_for_finish
            and self._mode != self.MODE_CROUCH
            and not self._passed_finish,
            FINISH_CONFIRM_FRAMES,
        )
        if finish_confirmed:
            self._mode = self.MODE_FINISH
            self._passed_finish = True
            self._finish_since = now
            self._have_error = False
            self._target_height = self.stand_height
            self._announce(
                f"到达终点，再往前驶入 {FINISH_DRIVE_IN_SECONDS:.1f} 秒后停车"
            )
            return

    def _announce(self, text: str) -> None:
        line = f">>> [赛道] {text}"
        print(f"\n{line}\n", flush=True)
        self.get_logger().info(line)

    def _mark_bridge_climb_ended(self, text: str) -> None:
        if self._bridge_started is None or self._bridge_climb_ended:
            return
        self._bridge_climb_ended = True
        self._announce(text)

    def _check_bridge_climb_end(self, now: float) -> None:
        if (
            self._bridge_started is None
            or self._bridge_climb_ended
            or self._passed_sidewalk
        ):
            return
        if self._bridge_climb_remaining(now) <= 0.0:
            self._mark_bridge_climb_ended("单边桥加速结束，恢复正常巡线速度")

    def _bridge_climb_remaining(self, now: float) -> float:
        if (
            self._bridge_started is None
            or not self._passed_bridge
            or self._passed_sidewalk
            or self._bridge_climb_ended
        ):
            return 0.0
        # 从靠近桥面重新计时。取消右偏不会清掉这段加速。
        origin = self._bridge_near_since or self._bridge_started
        return max(0.0, self.bridge_climb_seconds - (now - origin))

    def _speed_scale(self, now: float) -> float:
        if self._mode == self.MODE_CROUCH:
            return self.tunnel_speed_scale
        remaining = self._bridge_climb_remaining(now)
        if remaining > 0.0:
            return self.bridge_climb_speed / max(self.forward_speed, 1e-3)
        if self._on_bridge_section():
            return float(BRIDGE_ON_RAMP_SPEED_SCALE)
        if self._passed_bridge and not self._passed_sidewalk:
            return float(TURN_SPEED_SCALE)
        return 1.0

    def _control_timer(self) -> None:
        try:
            self._control_cycle()
        except Exception as error:
            self.get_logger().error(f"控制异常，立即停车: {error}")
            self._publish_zero("control_exception")
            try:
                self._current_roll = 0.0
                self._publish_posture(self.stand_height)
            except Exception:
                pass

    def _control_cycle(self) -> None:
        now = time.monotonic()
        with self._frame_lock:
            ok = self._camera_ok
            frame_time = self._frame_time
            sequence = self._sequence
            frame = self._frame.copy() if self._frame is not None else None
        if not ok or frame is None:
            if self._holding_stop():
                self._publish(0.0, 0.0)
            else:
                self._publish_zero("camera_unavailable")
            self._update_height(now)
            self._log(now, "摄像头不可用，停车")
            return
        if now - frame_time > self.image_timeout_seconds:
            if self._holding_stop():
                self._publish(0.0, 0.0)
            else:
                self._publish_zero("image_timeout")
            self._update_height(now)
            self._log(now, "画面超时，停车")
            return
        if sequence == self._last_sequence:
            if self._holding_stop():
                self._publish(0.0, 0.0)
            elif self.enable_motion and self._last_command_valid:
                self._publish(*self._last_command)
            else:
                self._publish_zero("no_new_frame")
            self._update_height(now)
            return
        self._last_sequence = sequence

        self._update_bridge_target()
        result = self.line_detector.detect(frame)
        scene = self.scene_detector.detect(
            frame, result.debug_frame, prefer_finish=self._passed_tunnel
        )
        self._update_mission(scene, now)
        self._check_bridge_climb_end(now)

        if self._mode == self.MODE_STOP:
            self._hold_sidewalk_stop(result, now)
            return
        if self._mode == self.MODE_FINISH:
            self._hold_finish_stop(result, now)
            return

        if self._should_search_after_bridge(result, now):
            self._handle_lost_line(result, now, searching=True)
            return
        if not result.valid:
            self._handle_lost_line(result, now)
            return
        self._lost_since = None

        command = self._follow_command(result, now)
        self._update_height(now)
        self._show(result, *command, self.enable_motion, scene)

    def _follow_command(
        self, result: DetectionResult, now: float
    ) -> Tuple[float, float]:
        if self._have_error:
            filtered = self.error_filter_alpha * result.error + (
                1.0 - self.error_filter_alpha
            ) * self._previous_error
            dt = max(1e-3, now - self._previous_control_time)
            derivative = float(
                np.clip(
                    (filtered - self._previous_error) / dt,
                    -self.max_error_derivative,
                    self.max_error_derivative,
                )
            )
        else:
            filtered = result.error
            derivative = 0.0
            self._have_error = True
        self._previous_error = filtered
        self._previous_control_time = now
        angular = self.steering_sign * (
            self.steering_kp * filtered + self.steering_kd * derivative
        )
        angular = float(
            np.clip(angular, -self.max_angular_speed, self.max_angular_speed)
        )
        turn_ratio = min(1.0, abs(angular) / self.max_angular_speed)
        scale = self._speed_scale(now)
        climbing = self._bridge_climb_remaining(now) > 0.0
        slowdown = 0.0 if climbing else self.curve_slowdown
        linear = max(
            self.minimum_forward_speed * scale,
            self.forward_speed * scale * (1.0 - slowdown * turn_ratio),
        )
        target = (linear, angular)
        if self.enable_motion:
            command = self._rate_limit(*target, now)
            self._publish(*command)
            self._last_command = command
            self._last_command_valid = True
        else:
            self._publish_zero("motion_disabled")
            command = target
        return command

    def _hold_sidewalk_stop(self, result: DetectionResult, now: float) -> None:
        if self.enable_motion:
            command = self._rate_limit(0.0, 0.0, now)
            self._publish(*command)
            self._last_command = command
            self._last_command_valid = True
        else:
            command = (0.0, 0.0)
            self._publish(0.0, 0.0)
            self._last_command = command

        stopped = abs(command[0]) <= 0.02 and abs(command[1]) <= 0.04
        if stopped or not self.enable_motion:
            if self._stable_since is None:
                self._stable_since = now
                self._announce(
                    f"人行道已停稳，开始计时 {self.sidewalk_stop_seconds:.0f} 秒"
                )
            elif now - self._stable_since >= self.sidewalk_stop_seconds:
                self._mode = self.MODE_FOLLOW
                self._passed_sidewalk = True
                self._stable_since = None
                self._have_error = False
                self._announce("人行道停车结束，继续出发")
        else:
            self._stable_since = None

        self._target_height = self.stand_height
        self._update_height(now)
        elapsed = 0.0 if self._stable_since is None else now - self._stable_since
        self._show(
            result,
            *command,
            self.enable_motion,
            None,
            extra=f"STOP {elapsed:.1f}/{self.sidewalk_stop_seconds:.0f}s",
        )

    def _hold_finish_stop(self, result: DetectionResult, now: float) -> None:
        elapsed = 0.0 if self._finish_since is None else now - self._finish_since
        driving_in = elapsed < FINISH_DRIVE_IN_SECONDS
        if driving_in:
            target = (FINISH_DRIVE_IN_SPEED, 0.0)
            extra = f"FINISH IN {elapsed:.1f}/{FINISH_DRIVE_IN_SECONDS:.1f}s"
        else:
            target = (0.0, 0.0)
            extra = "FINISH STOP"
        if self.enable_motion:
            command = self._rate_limit(*target, now)
            self._publish(*command)
            self._last_command = command
            self._last_command_valid = True
        else:
            command = target
            self._publish(0.0, 0.0)
            self._last_command = command

        self._target_height = self.stand_height
        self._update_height(now)
        self._show(
            result,
            *command,
            self.enable_motion,
            None,
            extra=extra,
        )

    def _in_right_turn(self) -> bool:
        return self._cleared_bridge and not self._passed_sidewalk

    def _has_left_foam(self, result: DetectionResult) -> bool:
        """只有黄线在画面左半、行数够，才算真正的外侧左海绵。"""
        if not result.valid or result.left_line_rows < 18:
            return False
        max_x = TURN_LEFT_MAX_X_RATIO * float(self.image_width)
        return 8.0 < result.left_edge_x < max_x

    def _should_search_after_bridge(
        self, result: DetectionResult, now: float
    ) -> bool:
        """下桥后先停车，再向左慢转，直到看见左侧海绵。"""
        if not self._in_right_turn() or self._cleared_at is None:
            return False
        if self._left_line_locked:
            return False
        elapsed = now - self._cleared_at
        min_spin_until = TURN_STOP_SECONDS + TURN_MIN_SPIN_SECONDS
        # 停车后必须先左转够时间，避免立刻把内圈黄块锁成左线。
        if elapsed >= min_spin_until and self._has_left_foam(result):
            self._left_line_locked = True
            self._announce("已锁到左侧海绵，顺着左线慢速过大弯")
            return False
        # 没锁到画面左半的海绵就继续左转，不要超时后改跟内圈。
        return True

    def _handle_lost_line(
        self,
        result: DetectionResult,
        now: float,
        searching: bool = False,
    ) -> None:
        turning = self._in_right_turn()
        if self._lost_since is None:
            self._lost_since = now
        lost_for = now - self._lost_since
        extra = ""
        # 隧道里白墙/反光经常丢黄线：一直慢速往前拱，不要停在洞口。
        if self._mode == self.MODE_CROUCH:
            tunnel_v = self.forward_speed * self.tunnel_speed_scale
            if self._last_command_valid:
                creep_v = float(
                    np.clip(
                        abs(self._last_command[0]),
                        self.minimum_forward_speed,
                        tunnel_v,
                    )
                )
                creep_w = float(np.clip(self._last_command[1], -0.10, 0.10))
            else:
                creep_v = max(self.minimum_forward_speed, 0.85 * tunnel_v)
                creep_w = 0.0
            extra = f"TUNNEL CREEP {lost_for:.1f}s"
            if self.enable_motion:
                command = self._rate_limit(creep_v, creep_w, now)
                self._publish(*command)
                self._last_command = command
                self._last_command_valid = True
            else:
                command = (creep_v, creep_w)
                self._publish_zero(result.reason)
            self._update_height(now)
            self._show(
                result,
                *command,
                self.enable_motion,
                extra=extra,
            )
            self._log(now, f"丢线 {lost_for:.1f} 秒 lost_tunnel {result.reason}")
            return
        if turning:
            grace = TURN_LOST_GRACE_SECONDS
        elif self._passed_tunnel:
            grace = POST_TUNNEL_LOST_GRACE_SECONDS
        else:
            grace = self.lost_line_grace_seconds
        if self.enable_motion and (searching or lost_for < grace):
            if turning or searching:
                elapsed = (
                    0.0
                    if self._cleared_at is None
                    else now - self._cleared_at
                )
                if searching and elapsed < TURN_STOP_SECONDS:
                    target = (0.0, 0.0)
                    extra = f"TURN STOP {elapsed:.1f}s"
                    if not self._turn_lost_announced:
                        self._turn_lost_announced = True
                        self._announce("过完桥先停车，接着向左慢转找左侧海绵")
                else:
                    target = (TURN_LOST_SPEED, TURN_LOST_ANGULAR)
                    extra = (
                        f"TURN SEARCH L {elapsed:.1f}s"
                        if searching
                        else f"TURN LOST L {lost_for:.1f}s"
                    )
                    if not self._turn_lost_announced:
                        self._turn_lost_announced = True
                        self._announce("过完桥向左慢转，寻找左侧海绵，不往前冲")
                command = self._rate_limit(*target, now)
                self._publish(*command)
                self._last_command = command
                self._last_command_valid = True
                reason = f"lost_turn {result.reason}"
            elif self._last_command_valid:
                self._publish(*self._last_command)
                command = self._last_command
                reason = f"lost_hold {result.reason}"
            else:
                self._publish_zero(result.reason)
                command = (0.0, 0.0)
                reason = result.reason
        else:
            self._publish_zero(result.reason)
            command = (0.0, 0.0)
            reason = result.reason
        self._update_height(now)
        self._show(
            result,
            *command,
            self.enable_motion and command != (0.0, 0.0),
            extra=extra,
        )
        self._log(now, f"丢线 {lost_for:.1f} 秒 {reason}")

    def _show(
        self,
        result: DetectionResult,
        linear: float,
        angular: float,
        command_active: bool,
        scene: Optional[SceneObservation] = None,
        extra: str = "",
    ) -> None:
        if self._mode == self.MODE_STOP:
            state, color, _label = "SIDEWALK STOP", (220, 220, 220), "cmd"
        elif self._mode == self.MODE_FINISH:
            driving_in = (
                self._finish_since is not None
                and (time.monotonic() - self._finish_since) < FINISH_DRIVE_IN_SECONDS
            )
            state, color, _label = (
                ("FINISH IN", (255, 255, 255), "cmd")
                if driving_in
                else ("FINISH STOP", (255, 255, 255), "cmd")
            )
        elif self._mode == self.MODE_CROUCH:
            state, color, _label = "TUNNEL CROUCH", (200, 200, 200), "cmd"
        elif command_active:
            state, color, _label = "RUN", (255, 255, 255), "cmd"
        elif self.enable_motion:
            state, color, _label = "ARMED / STOP", (210, 210, 210), "cmd"
        else:
            state, color, _label = "STOP / DRY-RUN", (180, 180, 180), "planned"
        debug = result.debug_frame
        bar = debug[0:92].copy()
        debug[0:92] = cv2.addWeighted(bar, 0.35, np.zeros_like(bar), 0.65, 0.0)
        cv2.putText(
            debug,
            f"{state}  {result.reason}  conf={result.confidence:.2f}",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            f"err={result.error:+.3f}   v={linear:.2f}   w={angular:+.2f}   "
            f"h={self._current_height:.2f}   roll={self._current_roll:+.1f}",
            (12, 52),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        progress = (
            f"bridge={'Y' if self._passed_bridge else 'N'} "
            f"walk={'Y' if self._passed_sidewalk else 'N'} "
            f"tunnel={'Y' if self._passed_tunnel else 'N'} "
            f"finish={'Y' if self._passed_finish else 'N'}"
        )
        if (
            self._passed_tunnel
            and not self._passed_finish
            and self._passed_tunnel_at is not None
        ):
            wait_left = FINISH_AFTER_TUNNEL_SECONDS - (
                time.monotonic() - self._passed_tunnel_at
            )
            if wait_left > 0.0:
                progress += f" fn-wait {wait_left:.1f}s"
        if self._passed_bridge and not self._passed_sidewalk:
            if self._on_bridge_section():
                progress += f" off={BRIDGE_OFFSET_CM:.1f}cm roll={BRIDGE_ROLL_DEG:.0f}deg"
                if self._bridge_near_since is not None:
                    pass_left = max(
                        0.0,
                        BRIDGE_PASS_SECONDS
                        - (time.monotonic() - self._bridge_near_since),
                    )
                    progress += f" pass {pass_left:.1f}s"
            elif self._cleared_bridge:
                progress += f" off={TURN_OFFSET_CM:+.1f}cm"
            climb_left = self._bridge_climb_remaining(time.monotonic())
            if climb_left > 0.0:
                progress += (
                    f" climb {climb_left:.1f}s v={self.bridge_climb_speed:.2f}"
                )
            elif self._cleared_bridge:
                progress += " bridge done"
            elif self._bridge_started is not None:
                progress += " hold"
            else:
                progress += " approach"
        if extra:
            progress = f"{progress} {extra}"
        cv2.putText(
            debug,
            progress,
            (12, 78),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (210, 230, 255),
            1,
            cv2.LINE_AA,
        )
        self._record_debug(debug)
        if not self._window_ready:
            return
        display = self._scale_debug_for_hdmi(debug) if DEBUG_FULLSCREEN else debug
        cv2.imshow(self.debug_window_name, display)
        if DEBUG_FULLSCREEN and not self._debug_fullscreen_applied:
            self._apply_debug_fullscreen()
            self._debug_fullscreen_applied = True
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27):
            self.request_exit("operator_exit")

    @staticmethod
    def _detect_hdmi_size() -> Tuple[int, int]:
        try:
            output = subprocess.check_output(
                ["xrandr", "--current"],
                text=True,
                timeout=1.0,
            )
            match = re.search(r"current\s+(\d+)\s+x\s+(\d+)", output)
            if match:
                return int(match.group(1)), int(match.group(2))
            match = re.search(r"(\d+)x(\d+)\s+\S*\*", output)
            if match:
                return int(match.group(1)), int(match.group(2))
        except (OSError, subprocess.SubprocessError, ValueError):
            pass
        return 1920, 1080

    def _apply_debug_fullscreen(self) -> None:
        width, height = self._hdmi_size
        cv2.setWindowProperty(
            self.debug_window_name,
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN,
        )
        cv2.resizeWindow(self.debug_window_name, width, height)
        cv2.moveWindow(self.debug_window_name, 0, 0)

    def _scale_debug_for_hdmi(self, frame: np.ndarray) -> np.ndarray:
        screen_w, screen_h = self._hdmi_size
        height, width = frame.shape[:2]
        scale = min(screen_w / max(width, 1), screen_h / max(height, 1))
        new_w = max(1, int(width * scale))
        new_h = max(1, int(height * scale))
        scaled = cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        canvas = np.zeros((screen_h, screen_w, 3), dtype=np.uint8)
        x0 = (screen_w - new_w) // 2
        y0 = (screen_h - new_h) // 2
        canvas[y0 : y0 + new_h, x0 : x0 + new_w] = scaled
        return canvas

    def _open_video_writer(self, frame: np.ndarray) -> None:
        folder = Path(RECORDINGS_DIR)
        folder.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = folder / f"shiwai_{stamp}.avi"
        height, width = frame.shape[:2]
        self._video_size = (int(width), int(height))
        fps = max(5.0, float(self.control_rate))
        writer = cv2.VideoWriter(
            str(path),
            cv2.VideoWriter_fourcc(*"MJPG"),
            fps,
            self._video_size,
        )
        if not writer.isOpened():
            self._announce("识别画面录像打开失败，这次只显示不保存")
            return
        self._video_writer = writer
        self._video_path = path
        self._video_frames = 0
        latest = folder / "shiwai_latest.avi"
        try:
            if latest.exists() or latest.is_symlink():
                latest.unlink()
            latest.symlink_to(path.name)
        except OSError:
            pass
        self._announce(f"开始录制识别画面：{path}")

    def _record_debug(self, frame: np.ndarray) -> None:
        if not RECORD_DEBUG_VIDEO or frame is None or frame.size == 0:
            return
        if self._video_writer is None:
            self._open_video_writer(frame)
        writer = self._video_writer
        if writer is None:
            return
        width, height = self._video_size
        if frame.shape[1] != width or frame.shape[0] != height:
            frame = cv2.resize(frame, self._video_size)
        writer.write(frame)
        self._video_frames += 1

    def _close_video_writer(self) -> None:
        writer = self._video_writer
        self._video_writer = None
        if writer is None:
            return
        writer.release()
        path = self._video_path
        if path is None:
            self._announce("识别画面录像已关闭")
            return
        self._announce(
            f"识别画面已保存：{path}，共 {self._video_frames} 帧"
        )

    def _log(self, now: float, text: str) -> None:
        if now - self._last_log >= self.status_log_period:
            self.get_logger().info(text)
            self._last_log = now

    def request_exit(self, reason: str) -> None:
        self._publish_zero(reason)
        self._target_height = self.stand_height
        self._current_height = self.stand_height
        self._current_roll = 0.0
        self._publish_posture(self.stand_height)
        self._exit_requested = True

    @property
    def exit_requested(self) -> bool:
        return self._exit_requested

    def close(self) -> None:
        self._closing = True
        self._stop_capture.set()
        with self._capture_lock:
            capture = self._capture
        if capture is not None:
            capture.release()
        if hasattr(self, "_capture_thread"):
            self._capture_thread.join(timeout=1.0)
        if rclpy.ok():
            self._current_roll = 0.0
            for _ in range(10):
                self._publish(0.0, 0.0)
                self._publish_posture(self.stand_height)
                time.sleep(0.02)
        if self._window_ready:
            cv2.destroyWindow(self.debug_window_name)
        self._close_video_writer()
        self.get_logger().info("室外赛道节点退出，已重复发布零速度并恢复站立高度")


def main(args=None) -> None:
    node = None
    exit_event = threading.Event()

    def signal_handler(signum, _frame) -> None:
        exit_event.set()
        if node is not None:
            node.request_exit(f"signal_{signum}")

    previous_sigint = signal.signal(signal.SIGINT, signal_handler)
    previous_sigterm = signal.signal(signal.SIGTERM, signal_handler)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = ShiWaiFollower()
        while rclpy.ok() and not node.exit_requested and not exit_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    main()
