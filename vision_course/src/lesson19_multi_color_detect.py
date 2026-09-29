from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson19_multi_color_detect"

# 多颜色识别的关键是把每种颜色的阈值、显示颜色、最小面积放在一起管理。
color_configs = [
    {
        "name": "red",
        "draw_color": (0, 0, 255),
        "ranges": [
            ((0, 100, 70), (12, 255, 255)),
            ((170, 100, 70), (179, 255, 255)),
        ],
        "min_area": 800,
        "max_area_ratio": 0.08,
        "min_width": 18,
        "min_height": 35,
        "max_aspect": 3.0,
        "min_fill_ratio": 0.25,
    },
    {
        "name": "green",
        "draw_color": (0, 255, 0),
        "ranges": [
            ((40, 60, 60), (85, 255, 255)),
        ],
        "min_area": 800,
        "max_area_ratio": 0.12,
        "min_width": 25,
        "min_height": 45,
        "max_aspect": 3.0,
        "min_fill_ratio": 0.35,
    },
    {
        "name": "blue",
        "draw_color": (255, 0, 0),
        "ranges": [
            ((95, 80, 60), (130, 255, 255)),
        ],
        "min_area": 800,
        "max_area_ratio": 0.08,
        "min_width": 18,
        "min_height": 35,
        "max_aspect": 3.0,
        "min_fill_ratio": 0.25,
    },
    {
        "name": "black",
        # 黑色目标本身很暗，所以用白色画框更容易观察。
        "draw_color": (255, 255, 255),
        "text_color": (255, 255, 255),
        "ranges": [
            ((0, 0, 0), (179, 255, 55)),
        ],
        "min_area": 2500,
        "max_area_ratio": 0.18,
        "min_width": 45,
        "min_height": 45,
        "max_aspect": 2.2,
        "min_fill_ratio": 0.45,
    },
]


def resize_to_width(image, width):
    height = image.shape[0]
    old_width = image.shape[1]
    new_height = int(height * width / old_width)
    return cv2.resize(image, (width, new_height))


def put_label(image, text, color=(0, 255, 0)):
    cv2.putText(image, text, (16, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    return image


def mask_to_bgr(mask):
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def clean_mask(mask):
    """先去掉零散噪点，再补目标内部的小断裂。"""
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def make_color_mask(hsv, ranges):
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in ranges:
        low_array = np.array(low)
        high_array = np.array(high)
        part = cv2.inRange(hsv, low_array, high_array)
        mask = cv2.bitwise_or(mask, part)
    return clean_mask(mask)


def touches_border(x, y, w, h, frame_width, frame_height, margin=8):
    return (
        x <= margin or y <= margin
        or x + w >= frame_width - margin
        or y + h >= frame_height - margin
    )


def is_valid_target(contour, config, frame_shape):
    frame_height, frame_width = frame_shape[:2]
    area = cv2.contourArea(contour)
    x, y, w, h = cv2.boundingRect(contour)
    box_area = w * h
    if box_area == 0:
        return False, area

    max_area = frame_width * frame_height * config["max_area_ratio"]
    aspect = max(w / h, h / w)
    fill_ratio = area / box_area

    if area < config["min_area"] or area > max_area:
        return False, area
    if w < config["min_width"] or h < config["min_height"]:
        return False, area
    if aspect > config["max_aspect"]:
        return False, area
    if fill_ratio < config["min_fill_ratio"]:
        return False, area
    if touches_border(x, y, w, h, frame_width, frame_height):
        return False, area

    return True, area


def find_valid_contours(mask, config, frame_shape):
    contours, _hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    valid = []
    for contour in contours:
        ok, area = is_valid_target(contour, config, frame_shape)
        if ok:
            valid.append((area, contour))

    # 从左到右排序，画面中多个同色目标的编号更稳定。
    valid.sort(key=lambda item: cv2.boundingRect(item[1])[0])
    return valid


def draw_contours(result, config, valid_contours):
    infos = []
    for index, (area, contour) in enumerate(valid_contours, start=1):
        x, y, w, h = cv2.boundingRect(contour)
        cx = x + w // 2
        cy = y + h // 2
        label = f"{config['name']}{index}"

        cv2.rectangle(result, (x, y), (x + w, y + h), config["draw_color"], 3)
        cv2.circle(result, (cx, cy), 7, (0, 255, 255), -1)
        cv2.putText(result, f"{label} area:{int(area)}",
                    (x, max(28, y - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    config["draw_color"], 2)
        cv2.putText(result, f"center:({cx},{cy})",
                    (x, max(55, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (0, 255, 255), 2)

        infos.append({
            "name": label,
            "area": area,
            "box": (x, y, w, h),
            "center": (cx, cy),
        })
    return infos


def build_result(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    result = frame.copy()
    masks = {}
    all_infos = []

    for config in color_configs:
        mask = make_color_mask(hsv, config["ranges"])
        valid_contours = find_valid_contours(mask, config, frame.shape)
        infos = draw_contours(result, config, valid_contours)

        masks[config["name"]] = mask
        all_infos.extend(infos)
        text_color = config.get("text_color", config["draw_color"])
        cv2.putText(result, f"{config['name']}: {len(infos)}",
                    (20, 40 + 34 * len(masks)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                    text_color, 2)

    cv2.putText(result, "s: save  q/Esc: quit", (20, result.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    compare = make_display(frame, result, masks)
    return masks, result, compare, all_infos


def make_display(frame, result, masks):
    panel_width = 320
    original_show = put_label(resize_to_width(frame.copy(), panel_width), "original")
    result_show = put_label(resize_to_width(result.copy(), panel_width), "result")

    red_show = put_label(
        resize_to_width(mask_to_bgr(masks["red"]), panel_width), "red mask"
    )
    green_show = put_label(
        resize_to_width(mask_to_bgr(masks["green"]), panel_width), "green mask"
    )
    blue_show = put_label(
        resize_to_width(mask_to_bgr(masks["blue"]), panel_width), "blue mask"
    )
    black_show = put_label(
        resize_to_width(mask_to_bgr(masks["black"]), panel_width), "black mask"
    )

    top = cv2.hconcat([original_show, result_show])
    bottom_left = cv2.hconcat([red_show, green_show])
    bottom_right = cv2.hconcat([blue_show, black_show])
    compare = cv2.vconcat([top, bottom_left, bottom_right])
    return compare


def save_results(masks, result, compare, prefix):
    outputs = {
        f"{prefix}_red_mask.jpg": masks["red"],
        f"{prefix}_green_mask.jpg": masks["green"],
        f"{prefix}_blue_mask.jpg": masks["blue"],
        f"{prefix}_black_mask.jpg": masks["black"],
        f"{prefix}_result.jpg": result,
        f"{prefix}_compare.jpg": compare,
    }
    for filename, image in outputs.items():
        save_path = image_dir / filename
        ok = cv2.imwrite(str(save_path), image)
        print("saved:", save_path if ok else f"save failed: {save_path}")


def print_infos(infos):
    if not infos:
        print("no valid color target")
        return

    for info in infos:
        print(
            f"{info['name']}: area={int(info['area'])}, "
            f"box={info['box']}, center={info['center']}"
        )


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 900, 720)

    print("multi color object detection")
    print("put red, green, blue or black objects in front of camera")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        masks, result, compare, infos = build_result(frame)
        cv2.imshow(window_name, compare)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson19_multi_color_{time_text}"
            save_results(masks, result, compare, prefix)
            print_infos(infos)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


run_camera_mode()
