#!/usr/bin/env python3
"""方案B 实验版：光照归一化的双边白线巡线。

与 dual_line_follower.py 的**唯一算法差别**在 _make_mask()：

  原版： absolute_bright = v >= dynamic_v
         dynamic_v 是【整幅 ROI】内低饱和像素 V 的 75 分位 + 8，是一个全局值。
         场地光照不均时（中心亮、两侧暗），这个阈值被明亮区域抬高，
         偏暗那一侧的白线【整条】低于阈值 → 只剩一条线
         → 车道中心只能靠 _infer_from_single_boundary() 用预设车道宽去猜。

  本版： illum_bright = v >= blur(v, sigma) * ILLUM_RATIO_MIN
         blur(v) 近似「该处的局部照明」，比值天然免疫光照梯度。

  实测依据（install/recordings/dual_white_line_20261011_143858.avi）：
    绝对亮度：左段白线亮峰 166 / 右段 212 → 差 46，全局阈值 201~211 只留右边
    归一化后：左段对比度 +0.169 / 右段 +0.230 → 差 0.060，两边都过得去
    逐门限统计中，全局门限的左/右通过比为 0.10/0.10/0.00，
    而局部自适应门限（adaptiveThreshold）为 3636/3588 —— 本来就平衡。

红底排除、局部自适应、形态学、三段融合、控制律与原版完全一致，
所以两版可以直接 A/B 对比。

用法：
    # 直接跑源码，不用编译
    python3 src/wheel_robot/scripts/dual_line_follower_illum.py --ros-args \
        -p enable_motion:=false -p show_debug_window:=true
        
    ros2 run wheel_robot dual_line_follower_illum.py enable_motion:=true 

    # 回放录像
 python3 src/wheel_robot/scripts/dual_line_follower_illum.py --ros-args \
  -p video_source:=$HOME/wheel_robot/recordings/course_20261011_144107.avi


    # 编译后也可（已加进 CMakeLists 的 PROGRAMS 列表）
    ros2 run wheel_robot dual_line_follower_illum.py --ros-args ...

输出落在 recordings_illum/ 而不是 recordings/，不覆盖原版录像，方便对比。

⚠️ 实验副本：validation 通过后要么合并回 dual_line_follower.py，要么删掉本文件。
   _make_mask() 是照抄原版再改门限的，原版若改动这里不会自动跟随。
"""

from __future__ import annotations

import signal
import sys
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.signals import SignalHandlerOptions

SCRIPTS_DIR = Path(__file__).resolve().parent
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import dual_line_follower as base  # noqa: E402

# ---------------------------------------------------------------- 本版新增参数
# 局部光照背景估计的高斯核大小。太小 → 把白线本身也算进背景，反而压掉它；
# 太大 → 跟不上光照梯度。ROI 是 320x160，取 25 约等于画面宽度的 8%。
ILLUM_BLUR_SIGMA = 25.0

# 相对局部背景的最小抬升倍数。白线一般比周围路面亮 10%~60%，
# 取 1.12 ≈ 抬升 12%。这个值越小越灵敏、噪声越多。
ILLUM_RATIO_MIN = 1.12

# 「强白」判据，用于 (local_bright | illum_strong) 这一支，比 MIN 更保守。
ILLUM_STRONG_RATIO = 1.20

# 除法下限：背景估计太小时比值会被噪声放大。
ILLUM_MIN_BACKGROUND = 30.0


class IlluminationNormalizedDetector(base.DualWhiteLineDetector):
    """把「全局绝对亮度阈值」换成「相对局部背景的亮度比值」。"""

    def _make_mask(self, roi_bgr: np.ndarray) -> Tuple[np.ndarray, int]:
        hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # ---- 本版的核心：局部光照归一化，取代原来的全局 dynamic_v ----
        v_float = v.astype(np.float32)
        background = cv2.GaussianBlur(v_float, (0, 0), ILLUM_BLUR_SIGMA)
        ratio = v_float / np.maximum(background, ILLUM_MIN_BACKGROUND)

        low_saturation = s <= base.WHITE_S_MAX
        # 相对亮度过关，且没有暗到没意义（沿用原版的绝对下限 DYNAMIC_V_MIN）
        illum_bright = (ratio >= ILLUM_RATIO_MIN) & (v >= base.DYNAMIC_V_MIN)
        illum_strong = ratio >= ILLUM_STRONG_RATIO

        # ---- 以下与 dual_line_follower._make_mask 保持一致 ----
        local_bright = cv2.adaptiveThreshold(
            v,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            self._clip_odd(base.ADAPTIVE_BLOCK_SIZE),
            base.ADAPTIVE_C,
        ) > 0
        local_dark = cv2.adaptiveThreshold(
            v,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY_INV,
            self._clip_odd(base.ADAPTIVE_BLOCK_SIZE),
            base.LOCAL_DARK_C,
        ) > 0

        red_1 = (
            (h >= base.RED_H_LOW_1)
            & (h <= base.RED_H_HIGH_1)
            & (s >= base.RED_S_MIN)
            & (v >= base.RED_V_MIN)
        )
        red_2 = (
            (h >= base.RED_H_LOW_2)
            & (h <= base.RED_H_HIGH_2)
            & (s >= base.RED_S_MIN)
            & (v >= base.RED_V_MIN)
        )

        bright_white = (
            (h >= base.WHITE_H_MIN)
            & (h <= base.WHITE_H_MAX)
            & low_saturation
            & illum_bright
            & (local_bright | illum_strong)
            & ~(red_1 | red_2)
        )
        cool_lane_paint = (
            (h >= base.COOL_LINE_H_MIN)
            & (h <= base.COOL_LINE_H_MAX)
            & (s >= base.COOL_LINE_S_MIN)
            & (s <= base.COOL_LINE_S_MAX)
            & (v >= base.COOL_LINE_V_MIN)
            & (local_dark | local_bright | illum_strong)
            & ~(red_1 | red_2)
        )
        white = bright_white | cool_lane_paint

        mask = white.astype(np.uint8) * 255
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            self._open_kernel,
            iterations=base.OPEN_ITERATIONS,
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_CLOSE,
            self._close_kernel,
            iterations=base.CLOSE_ITERATIONS,
        )

        # 遥测列的语义：CSV 里仍是 dynamic_v_threshold，这里折算成
        # 「按画面中位亮度算，等效的绝对亮度阈值」，方便和原版数值对照。
        dynamic_v = int(
            np.clip(
                float(np.median(background)) * ILLUM_RATIO_MIN,
                base.DYNAMIC_V_MIN,
                base.DYNAMIC_V_MAX,
            )
        )
        return mask, dynamic_v


class IlluminationNormalizedFollower(base.DualWhiteLineFollower):
    """原版节点，只把检测器换成归一化版本，并把输出改到 recordings_illum/。"""

    def __init__(self) -> None:
        # 必须在 super().__init__() 之前改：原版 __init__ 里直接用模块级
        # OUTPUT_DIRECTORY 拼路径。改到独立目录，避免和原版录像混淆。
        base.OUTPUT_DIRECTORY = base.OUTPUT_DIRECTORY.parent / "recordings_illum"
        super().__init__()
        self.detector = IlluminationNormalizedDetector()


def main(args=None) -> None:
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[IlluminationNormalizedFollower] = None

    def request_stop(_signum=None, _frame=None) -> None:
        if node is not None:
            node.stop_requested = True
            node.stop_now()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        node = IlluminationNormalizedFollower()
        node.get_logger().info(
            f"光照归一化版已启用: blur_sigma={ILLUM_BLUR_SIGMA} "
            f"ratio_min={ILLUM_RATIO_MIN} strong={ILLUM_STRONG_RATIO}"
        )
        node.run()
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"双白线巡线程序异常: {exc}")
        else:
            print(f"双白线巡线程序初始化失败: {exc}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
