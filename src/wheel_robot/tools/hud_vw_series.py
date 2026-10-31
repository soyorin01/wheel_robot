#!/usr/bin/env python3
"""从 shiwai 调试录像里提取 HUD 的 v / w 时序，导出 CSV + 曲线图。

原理
----
HUD 第二行是固定的 cv2.putText，字体 FONT_HERSHEY_SIMPLEX、scale 0.48、
thickness 1，格式串固定：

    err={:+.3f}   v={:.2f}   w={:+.2f}   h=... roll=... pitch=...

每段宽度是常数（+0.217 / 0.18 / -0.16 都是固定位数），所以 v 和 w 的字符
起点在每一帧都一样。于是：

1. 用同样的字体参数渲染出所有可能的 "v=0.00" ~ "v=0.60"、
   "w=-0.40" ~ "w=+0.40" 文本模板；
2. 在录像帧的固定位置裁一小块，和每个模板做 matchTemplate 取最高分。

不用装 tesseract / pytesseract，只依赖 cv2 + numpy（画图用 matplotlib，可选）。

用法
----
    # 只提取，打印摘要
    python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi

    # 导出 CSV + 图
    python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi \
        --csv /tmp/vw.csv --plot /tmp/vw.png

    # 只跑一段时间
    python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi \
        --start 8 --end 20 --csv /tmp/vw.csv

    # 看每一帧的识别置信度（调模板用）
    python3 src/wheel_robot/tools/hud_vw_series.py recordings/xxx.avi --verbose

输出 CSV 列：t, frame, v, w, v_score, w_score
"""

import argparse
import csv
import os
import sys
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.48
FONT_THICK = 1
TEXT_COLOR = 235
ORIGIN_X = 12          # cv2.putText 的 x
BASELINE_Y = 52        # cv2.putText 的 y
HUD_H = 92             # 顶部压暗条高度；这一块背景是纯黑，不会有 mask 干扰

# 与 shiwai_lin_line.py 里的 f-string 保持一致
ERR_SAMPLE = "err=+0.217"
V_SAMPLE = "v=0.18"
OV_SAMPLE = "ov=0.00"
W_SAMPLE = "w=-0.16"
# 10.30 晚 HUD 从「3 空格」改成「2 空格」并插入了 ov=，间距必须跟着改，
# 否则 v= / w= 的模板会整体错位，score 掉到 0.2~0.3 全是乱数。
GAP = "  "


def text_width(text: str) -> int:
    return cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICK)[0][0]


V_X = ORIGIN_X + text_width(ERR_SAMPLE + GAP)
OV_X = V_X + text_width(V_SAMPLE + GAP)
W_X = OV_X + text_width(OV_SAMPLE + GAP)
# 竖直方向取一整条，保证不同数字（有/无下伸笔画）都能盖住
CROP_Y0 = BASELINE_Y - 16
CROP_Y1 = BASELINE_Y + 6
PAD = 3


def render(text: str, x: int) -> np.ndarray:
    """在 92 行的黑底上按 HUD 的绝对坐标画字，返回裁剪好的模板。"""
    canvas = np.zeros((HUD_H, 640), dtype=np.uint8)
    cv2.putText(canvas, text, (x, BASELINE_Y), FONT, FONT_SCALE,
                TEXT_COLOR, FONT_THICK, cv2.LINE_AA)
    width = text_width(text)
    tile = canvas[CROP_Y0:CROP_Y1, x:x + width]
    # 去掉四周全黑的边，减少无关像素
    ys, xs = np.nonzero(tile)
    if len(ys) == 0:
        return tile
    return tile[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def build_templates(prefix: str, values: Sequence[float], fmt: str,
                    x: int) -> List[Tuple[float, np.ndarray]]:
    out = []
    for value in values:
        text = prefix + fmt.format(value)
        out.append((value, render(text, x)))
    return out


def read_field(frame: np.ndarray, x: int, templates,
               search_pad: int = PAD) -> Tuple[Optional[float], float]:
    """在帧的固定位置读一个数值字段，返回 (值, 置信度)。"""
    width = text_width_from_templates(templates)
    y0 = max(0, CROP_Y0 - search_pad)
    y1 = min(frame.shape[0], CROP_Y1 + search_pad)
    x0 = max(0, x - search_pad)
    x1 = min(frame.shape[1], x + width + search_pad)
    gray = cv2.cvtColor(frame[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame[y0:y1, x0:x1]
    best_value: Optional[float] = None
    best_score = -1.0
    for value, tile in templates:
        th, tw = tile.shape[:2]
        if gray.shape[0] < th or gray.shape[1] < tw:
            continue
        res = cv2.matchTemplate(gray, tile, cv2.TM_CCOEFF_NORMED)
        score = float(res.max())
        if score > best_score:
            best_score, best_value = score, value
    return best_value, best_score


def text_width_from_templates(templates) -> int:
    # 同组模板的字符串宽度是常数，随便取一个即可
    if not templates:
        return 0
    return templates[0][1].shape[1] + 8


def parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", help="调试录像 .avi")
    parser.add_argument("--csv", help="输出 CSV 路径")
    parser.add_argument("--plot", help="输出曲线图 .png")
    parser.add_argument("--start", type=float, default=None, help="起始秒")
    parser.add_argument("--end", type=float, default=None, help="结束秒")
    parser.add_argument("--v-max", type=float, default=0.60,
                        help="v 的搜索上限（默认 0.60）")
    parser.add_argument("--w-max", type=float, default=0.60,
                        help="w 的搜索上限（默认 0.60，绝对值）")
    parser.add_argument("--w-limit", type=float, default=0.48,
                        help="MAX_ANGULAR_SPEED，用于统计打满比例（默认 0.48）")
    parser.add_argument("--v-ref", default="0.18,0.30,0.45",
                        help="图上画横参考线的 v 值，逗号分隔，空字符串则不画")
    parser.add_argument("--verbose", action="store_true", help="逐帧打印识别置信度")
    return parser.parse_args(argv)


def main(argv: Sequence[str]) -> int:
    args = parse_args(argv)
    if not os.path.isfile(args.video):
        print(f"找不到录像：{args.video}", file=sys.stderr)
        return 1

    v_values = np.round(np.arange(0.0, args.v_max + 1e-6, 0.01), 2)
    w_values = np.round(np.arange(-args.w_max, args.w_max + 1e-6, 0.01), 2)
    v_templates = build_templates("v=", v_values, "{:.2f}", V_X)
    ov_templates = build_templates("ov=", v_values, "{:.2f}", OV_X)
    w_templates = build_templates("w=", w_values, "{:+.2f}", W_X)

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        print(f"打不开录像：{args.video}", file=sys.stderr)
        return 1
    fps = cap.get(cv2.CAP_PROP_FPS) or 15.0

    rows = []
    index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = index / fps
        if (args.start is None or t >= args.start) and (args.end is None or t <= args.end):
            v, v_score = read_field(frame, V_X, v_templates)
            ov, ov_score = read_field(frame, OV_X, ov_templates)
            w, w_score = read_field(frame, W_X, w_templates)
            if args.verbose:
                print(f"f{index:4d} t={t:6.2f}  v={v if v is not None else '?':>4}"
                      f" ({v_score:.2f})  ov={ov if ov is not None else '?':>4}"
                      f" ({ov_score:.2f})  w={w if w is not None else '?':>6}"
                      f" ({w_score:.2f})")
            if v is not None and w is not None:
                rows.append((t, index, v, w,
                             ov if ov is not None else float("nan"),
                             v_score, w_score))
        index += 1
    cap.release()

    if not rows:
        print("一帧都没读出来，检查一下录像是不是调试合成图。", file=sys.stderr)
        return 1

    # ---- CSV ----
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(["t", "frame", "v", "w", "ov", "v_score", "w_score"])
            for row in rows:
                writer.writerow([f"{row[0]:.3f}", row[1], f"{row[2]:.2f}",
                                 f"{row[3]:+.2f}", f"{row[4]:.2f}",
                                 f"{row[5]:.3f}", f"{row[6]:.3f}"])
        print(f"CSV  -> {args.csv}  ({len(rows)} 行)")

    # ---- 摘要 ----
    arr = np.array([[r[0], r[2], r[3]] for r in rows])
    v_arr, w_arr = arr[:, 1], arr[:, 2]
    print()
    print(f"帧数 {len(rows)}   时长 {arr[-1, 0] - arr[0, 0]:.2f}s   "
          f"fps {fps:.1f}   最低识别分 v={min(r[4] for r in rows):.2f} "
          f"w={min(r[5] for r in rows):.2f}")
    print(f"v : min {v_arr.min():.2f}  max {v_arr.max():.2f}  均值 "
          f"{v_arr.mean():.3f}  中位 {np.median(v_arr):.2f}")
    print(f"w : min {w_arr.min():+.2f}  max {w_arr.max():+.2f}  "
          f"|w|均值 {np.abs(w_arr).mean():.3f}  |w|max {np.abs(w_arr).max():.2f}")
    sat = float(np.mean(np.abs(w_arr) >= 0.98 * args.w_limit) * 100.0)
    print(f"|w| 打到 MAX_ANGULAR_SPEED({args.w_limit:.2f}) 的比例：{sat:.1f}%")
    for ref in [x for x in args.v_ref.split(",") if x.strip()]:
        try:
            value = float(ref)
        except ValueError:
            continue
        cover = float(np.mean(v_arr >= value - 1e-6) * 100.0)
        print(f"  v >= {value:.2f} 的帧占比：{cover:5.1f}%")

    # 1 秒一档的均值，方便直接看“哪一段该提速”
    print()
    print("  秒    v均值   v最大   |w|均值  |w|最大")
    step = 1.0
    t0, t1 = float(arr[0, 0]), float(arr[-1, 0])
    edge = t0
    while edge < t1:
        mask = (arr[:, 0] >= edge) & (arr[:, 0] < edge + step)
        if mask.any():
            print(f"{edge:5.1f}   {v_arr[mask].mean():5.2f}   "
                  f"{v_arr[mask].max():5.2f}    {np.abs(w_arr[mask]).mean():5.3f}   "
                  f"{np.abs(w_arr[mask]).max():5.3f}")
        edge += step

    # ---- 图 ----
    if args.plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:      # pragma: no cover
            print(f"画图跳过（matplotlib 不可用：{exc}）", file=sys.stderr)
            return 0
        fig, axes = plt.subplots(2, 1, figsize=(11, 6), sharex=True)
        axes[0].plot(arr[:, 0], v_arr, lw=1.2, color="#1f77b4")
        for ref in [x for x in args.v_ref.split(",") if x.strip()]:
            try:
                value = float(ref)
            except ValueError:
                continue
            axes[0].axhline(value, color="#999999", lw=0.7, ls="--")
            axes[0].text(arr[0, 0], value, f" {value:.2f}", va="bottom",
                         fontsize=8, color="#666666")
        axes[0].set_ylabel("v  (m/s)")
        axes[0].grid(alpha=0.3)
        axes[0].set_title(os.path.basename(args.video))
        axes[1].plot(arr[:, 0], w_arr, lw=1.2, color="#d62728")
        axes[1].axhline(0.0, color="k", lw=0.6)
        for limit in (args.w_limit, -args.w_limit):
            axes[1].axhline(limit, color="#999999", lw=0.7, ls="--")
        axes[1].set_ylabel("w  (rad/s)")
        axes[1].set_xlabel("t  (s)")
        axes[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130)
        print(f"\n图   -> {args.plot}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
