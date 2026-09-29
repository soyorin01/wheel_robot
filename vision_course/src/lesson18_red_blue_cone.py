from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson18_red_blue_cone"

# 固定参数写在这里，现场识别不稳定时只需要改这一小段。
params = {
    "red1_h_min": 0,
    "red1_h_max": 10,
    "red2_h_min": 156,
    "red2_h_max": 180,
    "red_s_min": 60,
    "red_v_min": 50,
    "blue_h_min": 95,
    "blue_h_max": 130,
    "blue_s_min": 43,
    "blue_v_min": 46,
    "min_area": 800,
}


def resize_for_display(image, max_width=1000):
    """只缩小窗口显示画面，不影响保存的原始图像。"""
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    new_height = int(height * max_width / width)
    return cv2.resize(image, (max_width, new_height))


def put_label(image, text):
    cv2.putText(image, text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return image


def mask_to_bgr(mask):
    """把单通道掩膜转成三通道，方便拼接显示。"""
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


def clean_mask(mask):
    """先去掉小噪点，再连接目标内部断裂区域。"""
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    return mask


def get_color_masks(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 红色在 HSV 中跨过 0 度，所以需要两段 H 范围合并。
    red_low_1 = np.array([
        params["red1_h_min"], params["red_s_min"], params["red_v_min"]
    ])
    red_high_1 = np.array([params["red1_h_max"], 255, 255])
    red_low_2 = np.array([
        params["red2_h_min"], params["red_s_min"], params["red_v_min"]
    ])
    red_high_2 = np.array([params["red2_h_max"], 255, 255])
    red_mask_1 = cv2.inRange(hsv, red_low_1, red_high_1)
    red_mask_2 = cv2.inRange(hsv, red_low_2, red_high_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    # 蓝色锥桶通常 H 在 100 到 130 左右，现场光照不同可微调。
    blue_low = np.array([
        params["blue_h_min"], params["blue_s_min"], params["blue_v_min"]
    ])
    blue_high = np.array([params["blue_h_max"], 255, 255])
    blue_mask = cv2.inRange(hsv, blue_low, blue_high)

    return clean_mask(red_mask), clean_mask(blue_mask)


def select_largest_contour(mask, min_area):
    contours, _hierarchy = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    valid = [contour for contour in contours if cv2.contourArea(contour) >= min_area]
    if not valid:
        return contours, None
    return contours, max(valid, key=cv2.contourArea)


def draw_target(result, contour, name, color):
    if contour is None:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    area = cv2.contourArea(contour)
    cx = x + w // 2
    cy = y + h // 2

    cv2.rectangle(result, (x, y), (x + w, y + h), color, 3)
    cv2.circle(result, (cx, cy), 8, (0, 255, 255), -1)
    cv2.putText(result, f"{name} area:{int(area)}", (x, max(30, y - 35)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
    cv2.putText(result, f"center:({cx},{cy})", (x, max(60, y - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    return {
        "name": name,
        "area": area,
        "box": (x, y, w, h),
        "center": (cx, cy),
    }


def build_result(frame):
    red_mask, blue_mask = get_color_masks(frame)
    min_area = params["min_area"]
    _red_contours, red_target = select_largest_contour(red_mask, min_area)
    _blue_contours, blue_target = select_largest_contour(blue_mask, min_area)

    result = frame.copy()
    red_info = draw_target(result, red_target, "red", (0, 0, 255))
    blue_info = draw_target(result, blue_target, "blue", (255, 0, 0))

    cv2.putText(result, f"min_area:{min_area}  s: save  q/Esc: quit",
                (20, result.shape[0] - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    red_show = put_label(mask_to_bgr(red_mask), "red mask")
    blue_show = put_label(mask_to_bgr(blue_mask), "blue mask")
    result_show = put_label(result.copy(), "result")
    top = cv2.hconcat([put_label(frame.copy(), "original"), result_show])
    bottom = cv2.hconcat([red_show, blue_show])
    bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))
    compare = cv2.vconcat([top, bottom])

    return red_mask, blue_mask, result, compare, red_info, blue_info


def save_results(red_mask, blue_mask, result, compare, prefix, params):
    outputs = {
        f"{prefix}_red_mask.jpg": red_mask,
        f"{prefix}_blue_mask.jpg": blue_mask,
        f"{prefix}_result.jpg": result,
        f"{prefix}_compare.jpg": compare,
    }
    for filename, image in outputs.items():
        save_path = image_dir / filename
        ok = cv2.imwrite(str(save_path), image)
        print("saved:", save_path if ok else f"save failed: {save_path}")

    param_path = image_dir / f"{prefix}_params.txt"
    param_text = "\n".join(f"{key}: {value}" for key, value in params.items())
    param_path.write_text(param_text + "\n", encoding="utf-8")
    print("saved:", param_path)


def print_result(red_info, blue_info):
    for info in [red_info, blue_info]:
        if info is None:
            continue
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
    cv2.resizeWindow(window_name, 1000, 650)

    print("red and blue cone detection")
    print("put red/blue cones in front of camera")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        red_mask, blue_mask, result, compare, red_info, blue_info = build_result(frame)
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson18_cone_{time_text}"
            save_results(red_mask, blue_mask, result, compare, prefix, params)
            print_result(red_info, blue_info)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


run_camera_mode()
