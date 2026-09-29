import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# 本课程读取和保存的图片统一放在 vision_course/images 目录。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)
default_image = image_dir / "lesson22_circles.jpg"

window_name = "lesson22_circle_detection"

# 霍夫圆参数。更换实验图片后，可根据目标尺寸和边缘清晰度调整。
hough_params = {
    "dp": 1.2,
    "min_dist": 100,
    "param1": 120,
    "param2": 42,
    "min_radius": 15,
    "max_radius": 185,
    "min_edge_support": 0.50,
    "edge_tolerance": 3,
}


def circle_edge_support(edges, x, y, radius, tolerance=3, samples=180):
    """计算检测圆周附近存在真实边缘的比例，返回0～1。"""
    hits = 0
    for angle in np.linspace(0, 2 * np.pi, samples, endpoint=False):
        cos_value = np.cos(angle)
        sin_value = np.sin(angle)
        found_edge = False

        # 在估计半径内外搜索几像素，允许霍夫半径存在轻微误差。
        for offset in range(-tolerance, tolerance + 1):
            current_radius = radius + offset
            px = int(round(x + current_radius * cos_value))
            py = int(round(y + current_radius * sin_value))
            if 0 <= px < edges.shape[1] and 0 <= py < edges.shape[0]:
                if edges[py, px] > 0:
                    found_edge = True
                    break
        if found_edge:
            hits += 1

    return hits / samples


def detect_circles(frame):
    """检测圆形目标，返回灰度图和按横坐标排序的圆形列表。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 高斯滤波可降低纹理和小噪点对圆周投票的干扰。
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    detected = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=hough_params["dp"],
        minDist=hough_params["min_dist"],
        param1=hough_params["param1"],
        param2=hough_params["param2"],
        minRadius=hough_params["min_radius"],
        maxRadius=hough_params["max_radius"],
    )

    if detected is None:
        return gray, []

    # 使用与霍夫圆一致的边缘阈值，复核每个候选圆的圆周完整度。
    edges = cv2.Canny(
        blurred, hough_params["param1"] // 2, hough_params["param1"]
    )
    frame_height, frame_width = frame.shape[:2]
    candidates = []
    for x, y, radius in np.round(detected[0]).astype(int):
        # 圆周越过画面边缘时无法验证完整度，因此不作为完整圆保留。
        if (
            x - radius < 0 or y - radius < 0
            or x + radius >= frame_width or y + radius >= frame_height
        ):
            continue

        support = circle_edge_support(
            edges, x, y, radius, hough_params["edge_tolerance"]
        )
        if support >= hough_params["min_edge_support"]:
            candidates.append((support, x, y, radius))

    # 优先保留圆周证据更完整的候选，再去除圆心非常接近的重复圆。
    candidates.sort(key=lambda item: item[0], reverse=True)
    kept = []
    for support, x, y, radius in candidates:
        duplicate = any(
            np.hypot(x - old_x, y - old_y)
            < 0.55 * min(radius, old_radius)
            for old_x, old_y, old_radius in kept
        )
        if not duplicate:
            kept.append((x, y, radius))

    # 从左到右编号，使同一张图片的C1、C2编号更稳定。
    kept.sort(key=lambda circle: circle[0])
    return gray, kept


def draw_info_panel(image, lines, margin=12):
    """绘制自动缩放的信息面板，避免文字超出画面右边界。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    default_scale = 0.62
    thickness = 2
    padding = 10
    line_gap = 7
    available_width = max(80, image.shape[1] - 2 * margin - 2 * padding)

    widths = [
        cv2.getTextSize(line, font, default_scale, thickness)[0][0]
        for line in lines
    ]
    longest_width = max(widths, default=1)
    font_scale = min(default_scale, default_scale * available_width / longest_width)
    font_scale = max(0.30, font_scale)

    metrics = [cv2.getTextSize(line, font, font_scale, thickness) for line in lines]
    text_width = max((item[0][0] for item in metrics), default=1)
    line_height = max((item[0][1] + item[1] for item in metrics), default=16)
    panel_width = min(image.shape[1] - 2 * margin, text_width + 2 * padding)
    panel_height = len(lines) * line_height + (len(lines) - 1) * line_gap + 2 * padding

    x1, y1 = margin, margin
    x2 = min(image.shape[1] - 1, x1 + panel_width)
    y2 = min(image.shape[0] - 1, y1 + panel_height)
    overlay = image.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.60, image, 0.40, 0, image)

    text_y = y1 + padding + line_height
    for line in lines:
        cv2.putText(
            image, line, (x1 + padding, text_y), font,
            font_scale, (0, 255, 0), thickness, cv2.LINE_AA
        )
        text_y += line_height + line_gap
    return x1, y1, x2, y2


def draw_controls(image):
    """在窗口底部显示按键说明，并自动适应画面宽度。"""
    text = "s: save  q/Esc: quit"
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.58
    thickness = 2
    width = cv2.getTextSize(text, font, scale, thickness)[0][0]
    if width > image.shape[1] - 24:
        scale *= (image.shape[1] - 24) / width
    cv2.putText(
        image, text, (12, image.shape[0] - 16), font,
        max(scale, 0.30), (255, 255, 255), thickness, cv2.LINE_AA
    )


def build_result(frame, mode):
    """检测并绘制所有圆形目标、圆心、半径和数量。"""
    gray, circles = detect_circles(frame)
    result = frame.copy()

    for index, (x, y, radius) in enumerate(circles, start=1):
        # 绿色圆周表示检测边界，红点表示估计圆心。
        cv2.circle(result, (x, y), radius, (0, 255, 0), 3)
        cv2.circle(result, (x, y), 5, (0, 0, 255), -1)

        # 小分辨率图片只显示简短标签，完整圆心坐标继续在终端输出。
        label = f"C{index} r:{radius}"
        text_size = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
        )[0]
        text_x = max(4, min(
            x - text_size[0] // 2,
            result.shape[1] - text_size[0] - 4,
        ))
        text_y = min(result.shape[0] - 6, y + radius - 8)

        # 标签放在圆内部下方，并增加黑底，避免相邻圆的文字互相覆盖。
        cv2.rectangle(
            result,
            (text_x - 3, text_y - text_size[1] - 3),
            (text_x + text_size[0] + 3, text_y + 4),
            (0, 0, 0),
            -1,
        )
        cv2.putText(
            result, label, (text_x, text_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2, cv2.LINE_AA
        )

    draw_info_panel(
        result,
        [
            f"{mode}  circles: {len(circles)}",
            (
                f"radius: {hough_params['min_radius']}-{hough_params['max_radius']} "
                f"p2: {hough_params['param2']}"
            ),
        ],
    )
    draw_controls(result)
    return gray, result, circles


def print_circles(circles):
    """在终端输出圆心、半径和目标数量，便于记录实验数据。"""
    print("circle count:", len(circles))
    for index, (x, y, radius) in enumerate(circles, start=1):
        print(f"C{index}: center=({x},{y}), radius={radius}")


def save_result(result, prefix):
    time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = image_dir / f"{prefix}_{time_text}.jpg"
    ok = cv2.imwrite(str(save_path), result)
    print("saved:" if ok else "save failed:", save_path)


def print_parameters():
    """输出当前检测参数，便于记录每次调参实验。"""
    print(
        "parameters: "
        f"min_dist={hough_params['min_dist']}, "
        f"param1={hough_params['param1']}, "
        f"param2={hough_params['param2']}, "
        f"radius={hough_params['min_radius']}-{hough_params['max_radius']}, "
        f"min_support={hough_params['min_edge_support']:.2f}"
    )


def run_image_mode(image_path):
    """默认模式：读取一张图片并执行圆形目标检测。"""
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        print("use --image PATH to specify another image")
        raise SystemExit(1)

    _gray, result, circles = build_result(frame, "IMAGE")
    print("image:", image_path)
    print_parameters()
    print_circles(circles)
    print("press s to save, q or Esc to quit")

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    while True:
        cv2.imshow(window_name, result)
        key = cv2.waitKey(0) & 0xFF
        if key == ord("s"):
            save_result(result, "lesson22_image_circles")
        elif key == ord("q") or key == 27:
            break
    cv2.destroyAllWindows()


def run_camera_mode(camera_index):
    """只有显式传入--camera参数时，程序才会打开摄像头。"""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("camera open failed:", camera_index)
        raise SystemExit(1)

    # 使用常见分辨率，降低开发板实时霍夫圆检测的计算量。
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    print(f"camera mode: device {camera_index}")
    print_parameters()
    print("press s to save, q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        _gray, result, circles = build_result(frame, "CAMERA")
        cv2.imshow(window_name, result)
        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            save_result(result, "lesson22_camera_circles")
            print_circles(circles)
        elif key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Lesson 22: circle detection, image input by default"
    )
    source_group = parser.add_mutually_exclusive_group()
    source_group.add_argument(
        "--image", type=Path,
        help=f"input image path; default: {default_image.relative_to(project_dir)}"
    )
    source_group.add_argument(
        "--camera", type=int,
        help="open a camera only when this option is provided, for example --camera 0"
    )
    parser.add_argument("--min-dist", type=float, help="minimum distance between circle centers")
    parser.add_argument("--param1", type=float, help="high threshold used by internal Canny")
    parser.add_argument("--param2", type=float, help="circle center vote threshold")
    parser.add_argument("--min-radius", type=int, help="minimum circle radius")
    parser.add_argument("--max-radius", type=int, help="maximum circle radius")
    parser.add_argument(
        "--min-support", type=float,
        help="minimum supported circumference ratio from 0 to 1"
    )
    args = parser.parse_args()

    # 命令行参数只覆盖用户明确指定的项目，其他项目继续使用课程默认值。
    overrides = {
        "min_dist": args.min_dist,
        "param1": args.param1,
        "param2": args.param2,
        "min_radius": args.min_radius,
        "max_radius": args.max_radius,
        "min_edge_support": args.min_support,
    }
    for name, value in overrides.items():
        if value is not None:
            hough_params[name] = value

    if hough_params["min_radius"] >= hough_params["max_radius"]:
        parser.error("--min-radius must be smaller than --max-radius")
    if not 0 <= hough_params["min_edge_support"] <= 1:
        parser.error("--min-support must be between 0 and 1")

    if args.camera is not None:
        run_camera_mode(args.camera)
    else:
        run_image_mode(args.image or default_image)
