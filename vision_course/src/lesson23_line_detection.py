import argparse
from datetime import datetime
from pathlib import Path
import math

import cv2
import numpy as np


# 本课程读取和保存的图片统一放在 vision_course/images 目录。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)
default_image = image_dir / "lesson23_lines.jpg"

window_name = "lesson23_line_detection"

# 默认参数适用于随课道路示例图，更换图片后可通过命令行覆盖。
line_params = {
    "canny_low": 50,
    "canny_high": 150,
    "hough_threshold": 55,
    "min_line_length": 70,
    "max_line_gap": 20,
    "min_abs_angle": 10.0,
    "duplicate_angle": 4.0,
    "duplicate_rho": 14.0,
}


def line_properties(x1, y1, x2, y2):
    """计算线段长度、方向角和所在直线到原点的有符号距离。"""
    dx = x2 - x1
    dy = y2 - y1
    length = math.hypot(dx, dy)
    angle = math.degrees(math.atan2(dy, dx))

    # 把角度统一到[-90, 90)，避免同一方向出现相差180度的结果。
    if angle >= 90:
        angle -= 180
    elif angle < -90:
        angle += 180

    angle_rad = math.radians(angle)
    rho = -x1 * math.sin(angle_rad) + y1 * math.cos(angle_rad)
    return length, angle, rho


def angle_difference(first, second):
    """计算两条无方向直线的最小夹角差。"""
    difference = abs(first - second)
    return min(difference, 180 - difference)


def remove_duplicate_lines(candidates):
    """优先保留长线段，去除角度和位置都很接近的重复直线。"""
    candidates.sort(key=lambda item: item["length"], reverse=True)
    kept = []
    for candidate in candidates:
        duplicate = any(
            angle_difference(candidate["angle"], old["angle"])
            < line_params["duplicate_angle"]
            and abs(candidate["rho"] - old["rho"])
            < line_params["duplicate_rho"]
            for old in kept
        )
        if not duplicate:
            kept.append(candidate)
    return kept


def detect_lines(frame):
    """执行预处理、概率霍夫直线检测、长度角度筛选和去重。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 1.2)
    edges = cv2.Canny(
        blurred, line_params["canny_low"], line_params["canny_high"]
    )

    detected = cv2.HoughLinesP(
        edges,
        rho=1,
        theta=np.pi / 180,
        threshold=line_params["hough_threshold"],
        minLineLength=line_params["min_line_length"],
        maxLineGap=line_params["max_line_gap"],
    )
    if detected is None:
        return gray, edges, []

    candidates = []
    for line in detected[:, 0]:
        x1, y1, x2, y2 = [int(value) for value in line]
        length, angle, rho = line_properties(x1, y1, x2, y2)

        # 过滤过短线段和接近水平的背景边缘，保留道路方向线。
        if length < line_params["min_line_length"]:
            continue
        if abs(angle) < line_params["min_abs_angle"]:
            continue

        candidates.append({
            "points": (x1, y1, x2, y2),
            "length": length,
            "angle": angle,
            "rho": rho,
        })

    lines = remove_duplicate_lines(candidates)
    # 按线段中点横坐标排序，使L1、L2编号在同一场景中更稳定。
    lines.sort(key=lambda item: (item["points"][0] + item["points"][2]) / 2)
    return gray, edges, lines


def draw_info_panel(image, lines, margin=12):
    """绘制自动缩放的信息面板，避免文字超出画面。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    default_scale = 0.60
    thickness = 2
    padding = 9
    gap = 6
    available = max(80, image.shape[1] - 2 * margin - 2 * padding)
    widths = [cv2.getTextSize(text, font, default_scale, thickness)[0][0]
              for text in lines]
    scale = min(default_scale, default_scale * available / max(widths, default=1))
    scale = max(0.30, scale)
    metrics = [cv2.getTextSize(text, font, scale, thickness) for text in lines]
    text_width = max((item[0][0] for item in metrics), default=1)
    line_height = max((item[0][1] + item[1] for item in metrics), default=16)
    panel_width = min(image.shape[1] - 2 * margin, text_width + 2 * padding)
    panel_height = len(lines) * line_height + (len(lines) - 1) * gap + 2 * padding
    x1, y1 = margin, margin
    x2 = min(image.shape[1] - 1, x1 + panel_width)
    y2 = min(image.shape[0] - 1, y1 + panel_height)

    overlay = image.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.60, image, 0.40, 0, image)
    text_y = y1 + padding + line_height
    for text in lines:
        cv2.putText(image, text, (x1 + padding, text_y), font, scale,
                    (0, 255, 0), thickness, cv2.LINE_AA)
        text_y += line_height + gap
    return x1, y1, x2, y2


def put_short_label(image, text, x, y):
    """给直线绘制带黑底的短标签，避免标签互相遮挡。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.48
    thickness = 1
    width, height = cv2.getTextSize(text, font, scale, thickness)[0]
    x = max(3, min(x, image.shape[1] - width - 5))
    y = max(height + 5, min(y, image.shape[0] - 5))
    cv2.rectangle(image, (x - 2, y - height - 3),
                  (x + width + 2, y + 3), (0, 0, 0), -1)
    cv2.putText(image, text, (x, y), font, scale,
                (0, 255, 255), thickness, cv2.LINE_AA)


def build_result(frame, mode):
    """绘制筛选后的直线、编号、角度和数量。"""
    gray, edges, lines = detect_lines(frame)
    result = frame.copy()
    colors = [(0, 255, 255), (0, 255, 0), (255, 128, 0), (255, 0, 255)]

    for index, line in enumerate(lines, start=1):
        x1, y1, x2, y2 = line["points"]
        color = colors[(index - 1) % len(colors)]
        cv2.line(result, (x1, y1), (x2, y2), color, 4, cv2.LINE_AA)
        middle_x = (x1 + x2) // 2
        middle_y = (y1 + y2) // 2
        put_short_label(
            result, f"L{index} {line['angle']:.1f} deg",
            middle_x + 5, middle_y - 7
        )

    draw_info_panel(
        result,
        [
            f"{mode}  lines: {len(lines)}",
            f"min length: {line_params['min_line_length']}",
        ],
    )
    return gray, edges, result, lines


def resize_panel(image, width=320):
    """把原图、边缘图和结果图缩放到相同宽度。"""
    height, old_width = image.shape[:2]
    return cv2.resize(image, (width, int(height * width / old_width)))


def make_comparison(frame, edges, result):
    """生成原图、Canny边缘和直线结果三联图。"""
    edge_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    panels = [
        (resize_panel(frame), "original"),
        (resize_panel(edge_bgr), "Canny edges"),
        (resize_panel(result), "Hough lines"),
    ]
    output = []
    for panel, title in panels:
        # 标题放在独立黑色栏中，避免覆盖结果图的信息面板。
        panel = cv2.copyMakeBorder(
            panel, 38, 0, 0, 0,
            cv2.BORDER_CONSTANT, value=(20, 20, 20)
        )
        cv2.putText(panel, title, (10, 27), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 255, 0), 2, cv2.LINE_AA)
        output.append(panel)
    return cv2.hconcat(output)


def print_lines(lines):
    """在终端输出每条直线的端点、长度和方向角。"""
    print("line count:", len(lines))
    for index, line in enumerate(lines, start=1):
        print(
            f"L{index}: points={line['points']}, "
            f"length={line['length']:.1f}, angle={line['angle']:.1f} deg"
        )


def save_result(comparison, prefix):
    time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = image_dir / f"{prefix}_{time_text}.jpg"
    ok = cv2.imwrite(str(path), comparison)
    print("saved:" if ok else "save failed:", path)


def run_image_mode(image_path):
    """默认模式：读取图片并执行直线检测。"""
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    _gray, edges, result, lines = build_result(frame, "IMAGE")
    comparison = make_comparison(frame, edges, result)
    print("image:", image_path)
    print_lines(lines)
    print("press s to save, q or Esc to quit")
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1100, 650)
    while True:
        cv2.imshow(window_name, comparison)
        key = cv2.waitKey(0) & 0xFF
        if key == ord("s"):
            save_result(comparison, "lesson23_image_lines")
        elif key == ord("q") or key == 27:
            break
    cv2.destroyAllWindows()


def run_camera_mode(camera_index):
    """只有显式传入--camera参数时才打开摄像头。"""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("camera open failed:", camera_index)
        raise SystemExit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1100, 650)
    print(f"camera mode: device {camera_index}")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break
        _gray, edges, result, lines = build_result(frame, "CAMERA")
        comparison = make_comparison(frame, edges, result)
        cv2.imshow(window_name, comparison)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            save_result(comparison, "lesson23_camera_lines")
            print_lines(lines)
        elif key == ord("q") or key == 27:
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Lesson 23: line detection, image input by default"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--image", type=Path, help="input image path")
    source.add_argument("--camera", type=int, help="camera index, for example 0")
    parser.add_argument("--canny-low", type=int)
    parser.add_argument("--canny-high", type=int)
    parser.add_argument("--threshold", type=int, help="Hough vote threshold")
    parser.add_argument("--min-length", type=int)
    parser.add_argument("--max-gap", type=int)
    parser.add_argument("--min-angle", type=float)
    args = parser.parse_args()

    overrides = {
        "canny_low": args.canny_low,
        "canny_high": args.canny_high,
        "hough_threshold": args.threshold,
        "min_line_length": args.min_length,
        "max_line_gap": args.max_gap,
        "min_abs_angle": args.min_angle,
    }
    for name, value in overrides.items():
        if value is not None:
            line_params[name] = value

    if line_params["canny_low"] >= line_params["canny_high"]:
        parser.error("--canny-low must be smaller than --canny-high")

    if args.camera is not None:
        run_camera_mode(args.camera)
    else:
        run_image_mode(args.image or default_image)
