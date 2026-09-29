from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# 课程生成的图片统一保存到 vision_course/images 目录。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson20_largest_target"

# 复用上一课的颜色阈值。程序会先找到各颜色目标，再从中选择面积最大的一个。
color_configs = [
    {
        "name": "red",
        "ranges": [
            # 收紧红色色调和饱和度，避免棕色纸板进入红色掩膜。
            ((0, 150, 80), (8, 255, 255)),
            ((172, 150, 80), (179, 255, 255)),
        ],
        "dominant_channel": 2,  # BGR中的R通道
        "dominance_ratio": 1.30,
        "min_channel_gap": 30,
    },
    {
        "name": "green",
        "ranges": [((40, 80, 60), (85, 255, 255))],
        "dominant_channel": 1,  # BGR中的G通道
        "dominance_ratio": 1.15,
        "min_channel_gap": 20,
    },
    {
        "name": "blue",
        "ranges": [((95, 100, 60), (130, 255, 255))],
        "dominant_channel": 0,  # BGR中的B通道
        "dominance_ratio": 1.20,
        "min_channel_gap": 25,
    },
]

# 小于该面积的轮廓通常是噪点，不参加最大目标比较。
min_area = 800


def make_color_mask(hsv, ranges):
    """把同一种颜色的一段或多段HSV范围合并成一张掩膜。"""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in ranges:
        part = cv2.inRange(hsv, np.array(low), np.array(high))
        mask = cv2.bitwise_or(mask, part)

    # 开运算去除小噪点，闭运算填补目标内部的小孔洞。
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def touches_border(x, y, w, h, frame_width, frame_height, margin=6):
    """判断轮廓是否贴近画面边缘，避免把大面积背景当成目标。"""
    return (
        x <= margin
        or y <= margin
        or x + w >= frame_width - margin
        or y + h >= frame_height - margin
    )


def has_expected_color(frame, color_mask, contour, config):
    """复核候选区域的主颜色通道，排除棕色纸板等近似颜色。"""
    contour_mask = np.zeros(color_mask.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, -1)

    # 只统计既在轮廓内、又通过HSV阈值的像素。
    valid_pixels = cv2.bitwise_and(contour_mask, color_mask)
    if cv2.countNonZero(valid_pixels) < 100:
        return False

    mean_bgr = cv2.mean(frame, mask=valid_pixels)[:3]
    dominant_index = config["dominant_channel"]
    dominant_value = mean_bgr[dominant_index]
    other_values = [
        value for index, value in enumerate(mean_bgr)
        if index != dominant_index
    ]
    strongest_other = max(other_values)

    # 目标主通道既要按比例领先，也要有足够的绝对差值。
    return (
        dominant_value >= strongest_other * config["dominance_ratio"]
        and dominant_value - strongest_other >= config["min_channel_gap"]
    )


def collect_candidates(frame):
    """收集所有通过面积、尺寸、填充率和边缘检查的候选目标。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    frame_height, frame_width = frame.shape[:2]
    combined_mask = np.zeros((frame_height, frame_width), dtype=np.uint8)
    candidates = []

    for config in color_configs:
        mask = make_color_mask(hsv, config["ranges"])
        combined_mask = cv2.bitwise_or(combined_mask, mask)
        contours, _hierarchy = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            x, y, w, h = cv2.boundingRect(contour)
            box_area = w * h

            # 先过滤无效轮廓，再参加最大面积排序。
            if area < min_area or box_area == 0:
                continue
            if w < 20 or h < 25:
                continue
            if area / box_area < 0.25:
                continue
            if touches_border(x, y, w, h, frame_width, frame_height):
                continue
            if not has_expected_color(frame, mask, contour, config):
                continue

            candidates.append(
                {
                    "name": config["name"],
                    "area": area,
                    "contour": contour,
                    "box": (x, y, w, h),
                    "center": (x + w // 2, y + h // 2),
                }
            )

    # reverse=True 表示从大到小排列，第0个元素就是最大目标。
    candidates.sort(key=lambda item: item["area"], reverse=True)
    return combined_mask, candidates


def describe_position(dx, dead_zone=30):
    """根据水平偏差给出目标位于画面左侧、中央还是右侧。"""
    if dx < -dead_zone:
        return "LEFT"
    if dx > dead_zone:
        return "RIGHT"
    return "CENTER"


def draw_info_panel(image, lines, margin=12):
    """在左上角绘制自动缩放的信息面板，保证文字不越过画面边界。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    default_scale = 0.65
    thickness = 2
    padding = 10
    line_gap = 8
    available_width = max(80, image.shape[1] - 2 * margin - 2 * padding)

    # 如果窗口宽度较小，就按最长一行文字自动缩小字号。
    widths = [
        cv2.getTextSize(line, font, default_scale, thickness)[0][0]
        for line in lines
    ]
    longest_width = max(widths, default=1)
    font_scale = min(default_scale, default_scale * available_width / longest_width)
    font_scale = max(0.42, font_scale)

    metrics = [cv2.getTextSize(line, font, font_scale, thickness) for line in lines]
    text_width = max((size[0][0] for size in metrics), default=1)
    line_height = max((size[0][1] + size[1] for size in metrics), default=16)
    panel_width = min(image.shape[1] - 2 * margin, text_width + 2 * padding)
    panel_height = len(lines) * line_height + (len(lines) - 1) * line_gap + 2 * padding

    x1 = margin
    y1 = margin
    x2 = min(image.shape[1] - 1, x1 + panel_width)
    y2 = min(image.shape[0] - 1, y1 + panel_height)

    # 半透明黑底可防止浅色背景影响黄色文字可读性。
    overlay = image.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.60, image, 0.40, 0, image)

    text_y = y1 + padding + line_height
    for line in lines:
        cv2.putText(
            image, line, (x1 + padding, text_y), font,
            font_scale, (0, 255, 255), thickness, cv2.LINE_AA
        )
        text_y += line_height + line_gap

    return x1, y1, x2, y2


def build_result(frame):
    combined_mask, candidates = collect_candidates(frame)
    result = frame.copy()
    frame_height, frame_width = frame.shape[:2]
    image_center = (frame_width // 2, frame_height // 2)

    # 用灰色细框显示其他候选目标，便于观察最大目标的选择过程。
    for candidate in candidates[1:]:
        x, y, w, h = candidate["box"]
        cv2.rectangle(result, (x, y), (x + w, y + h), (180, 180, 180), 1)

    main_target = candidates[0] if candidates else None
    if main_target is not None:
        x, y, w, h = main_target["box"]
        cx, cy = main_target["center"]
        dx = cx - image_center[0]
        dy = cy - image_center[1]
        position = describe_position(dx)

        # 最大目标使用黄色粗框和中心点突出显示。
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 255), 4)
        cv2.circle(result, (cx, cy), 7, (0, 0, 255), -1)
        cv2.line(result, image_center, (cx, cy), (255, 255, 0), 2)
        # 信息固定分三行放在左上角，不再从目标框位置向右延伸。
        draw_info_panel(
            result,
            [
                f"MAIN: {main_target['name']}  area: {int(main_target['area'])}",
                f"center: ({cx},{cy})  offset: ({dx},{dy})",
                f"position: {position}",
            ],
        )
    else:
        draw_info_panel(result, ["NO VALID TARGET"])

    # 画面中心十字可用于直观看出目标偏向哪一侧。
    cv2.drawMarker(result, image_center, (255, 0, 255), cv2.MARKER_CROSS, 24, 2)
    cv2.putText(
        result,
        f"candidates: {len(candidates)}  s: save  q/Esc: quit",
        (20, frame_height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )
    return combined_mask, result, main_target


def print_main_target(main_target, frame_shape):
    """保存结果时，在终端输出最大目标的位置数据。"""
    if main_target is None:
        print("no valid target")
        return

    frame_height, frame_width = frame_shape[:2]
    cx, cy = main_target["center"]
    dx = cx - frame_width // 2
    dy = cy - frame_height // 2
    print(
        f"main={main_target['name']}, area={int(main_target['area'])}, "
        f"center=({cx},{cy}), offset=({dx},{dy}), "
        f"position={describe_position(dx)}"
    )


def save_results(mask, result):
    time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
    mask_path = image_dir / f"lesson20_largest_mask_{time_text}.jpg"
    result_path = image_dir / f"lesson20_largest_result_{time_text}.jpg"
    cv2.imwrite(str(mask_path), mask)
    cv2.imwrite(str(result_path), result)
    print("saved:", mask_path)
    print("saved:", result_path)


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    print("press s to save, press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        mask, result, main_target = build_result(frame)
        cv2.imshow(window_name, result)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            save_results(mask, result)
            print_main_target(main_target, frame.shape)
        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    # 直接运行本文件时启动摄像头；被其他程序导入时不会自动占用摄像头。
    run_camera_mode()
