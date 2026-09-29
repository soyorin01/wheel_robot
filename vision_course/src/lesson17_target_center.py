import sys
from datetime import datetime
from pathlib import Path

import cv2
# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)
window_name = "lesson17_target_center"
def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["lesson17_demo.png", "car.png", "dige.png", "49.jpg"]:
        image_path = image_dir / name
        if image_path.exists():
            return image_path

    image_files = sorted(
        list(image_dir.glob("*.jpg"))
        + list(image_dir.glob("*.jpeg"))
        + list(image_dir.glob("*.png"))
    )
    if image_files:
        return image_files[0]

    print("no image found in:", image_dir)
    raise SystemExit(1)


def resize_for_display(image, max_width=1000):
    """只缩小窗口显示画面，不影响保存的原始图像。"""
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    new_height = int(height * max_width / width)
    return cv2.resize(image, (max_width, new_height))


def gray_to_bgr(image):
    """把单通道图像转成三通道，方便拼接显示。"""
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def put_label(image, text):
    cv2.putText(image, text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return image


def get_values():
    v_min = cv2.getTrackbarPos("v_min", window_name)
    min_area = cv2.getTrackbarPos("min_area", window_name)
    return v_min, min_area


def build_binary(frame, v_min):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 白色板子通常是“饱和度低、亮度高”的区域，用 HSV 更容易筛出来。
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_white = (0, 0, v_min)
    upper_white = (179, 90, 255)
    binary = cv2.inRange(hsv, lower_white, upper_white)

    # 开运算去掉小白点，闭运算把白色板子内部的小黑洞补起来。
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=2)
    return gray, binary


def draw_frame_center(image):
    height, width = image.shape[:2]
    frame_cx = width // 2
    frame_cy = height // 2

    # 蓝色十字表示画面中心，后续偏移量都以它为基准。
    cv2.line(image, (frame_cx - 35, frame_cy), (frame_cx + 35, frame_cy),
             (255, 0, 0), 3)
    cv2.line(image, (frame_cx, frame_cy - 35), (frame_cx, frame_cy + 35),
             (255, 0, 0), 3)
    cv2.circle(image, (frame_cx, frame_cy), 8, (255, 0, 0), -1)
    cv2.putText(image, f"frame center: ({frame_cx},{frame_cy})",
                (20, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (255, 0, 0), 3)

    return frame_cx, frame_cy


def draw_centers(frame, binary, min_area):
    contours, _hierarchy = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    box_image = frame.copy()
    center_image = frame.copy()
    frame_cx, frame_cy = draw_frame_center(center_image)
    height, width = frame.shape[:2]

    valid_contours = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        # 墙面、窗户等白色背景经常贴住画面边缘，这里过滤掉贴边轮廓。
        margin = 8
        touches_border = (
            x <= margin
            or y <= margin
            or x + w >= width - margin
            or y + h >= height - margin
        )
        if touches_border:
            continue

        # 白色板子一般是一个面积较大的矩形区域，太细长的轮廓不要。
        aspect = w / h if h else 0
        if aspect < 0.4 or aspect > 4.0:
            continue

        valid_contours.append((area, contour))

    center_list = []
    if valid_contours:
        # 第17节只演示一个主目标：选择面积最大的有效轮廓。
        area, contour = max(valid_contours, key=lambda item: item[0])
        x, y, w, h = cv2.boundingRect(contour)

        # 目标中心点就是检测框左右、上下的中间位置。
        cx = x + w // 2
        cy = y + h // 2
        dx = cx - frame_cx
        dy = cy - frame_cy

        cv2.rectangle(box_image, (x, y), (x + w, y + h), (0, 0, 255), 4)
        cv2.rectangle(center_image, (x, y), (x + w, y + h), (0, 0, 255), 4)
        cv2.circle(center_image, (cx, cy), 12, (0, 255, 255), -1)
        cv2.line(center_image, (frame_cx, frame_cy), (cx, cy), (0, 255, 255), 4)

        cv2.putText(center_image, f"target center: ({cx},{cy})", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
        cv2.putText(center_image, f"offset dx:{dx} dy:{dy}", (20, 165),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
        cv2.putText(box_image, f"x:{x} y:{y} w:{w} h:{h}", (20, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)

        center_list.append({
            "area": area,
            "box": (x, y, w, h),
            "center": (cx, cy),
            "offset": (dx, dy),
        })

    selected_count = len(center_list)
    cv2.putText(box_image, f"valid contours: {len(valid_contours)}  selected: {selected_count}",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)
    cv2.putText(center_image, f"valid contours: {len(valid_contours)}  selected: {selected_count}",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

    return contours, box_image, center_image, center_list


def build_results(frame, v_min, min_area):
    gray, binary = build_binary(frame, v_min)
    contours, box_image, center_image, center_list = draw_centers(
        frame, binary, min_area
    )
    return gray, binary, contours, box_image, center_image, center_list


def build_compare(frame, gray, binary, box_image, center_image):
    original_show = put_label(frame.copy(), "original")
    gray_show = put_label(gray_to_bgr(gray), "gray")
    binary_show = put_label(gray_to_bgr(binary), "binary")
    box_show = put_label(box_image.copy(), "bounding boxes")
    center_show = put_label(center_image.copy(), "center and offset")

    top = cv2.hconcat([original_show, gray_show, binary_show])
    bottom = cv2.hconcat([box_show, center_show])
    bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))
    return cv2.vconcat([top, bottom])


def save_results(gray, binary, box_image, center_image, compare, prefix):
    outputs = {
        f"{prefix}_gray.jpg": gray,
        f"{prefix}_binary.jpg": binary,
        f"{prefix}_boxes.jpg": box_image,
        f"{prefix}_center.jpg": center_image,
        f"{prefix}_compare.jpg": compare,
    }

    for filename, image in outputs.items():
        save_path = image_dir / filename
        ok = cv2.imwrite(str(save_path), image)
        print("saved:", save_path if ok else f"save failed: {save_path}")


def print_center_info(contours, center_list, min_area):
    print("total contours:", len(contours))
    print("min area:", min_area)
    print("kept targets:", len(center_list))
    for index, item in enumerate(center_list[:5], start=1):
        print(
            f"target {index}: area={int(item['area'])}, "
            f"box={item['box']}, center={item['center']}, "
            f"offset={item['offset']}"
        )


def nothing(_value):
    pass


def create_window():
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1000, 650)
    cv2.createTrackbar("v_min", window_name, 160, 255, nothing)
    cv2.createTrackbar("min_area", window_name, 3000, 200000, nothing)


def run_image_mode():
    image_path = get_image_path()
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    create_window()
    print("source image:", image_path)
    print("slide v_min/min_area to change white board center result")
    print("press s to save current result")
    print("press q or Esc to quit")

    saved_default = False
    while True:
        v_min, min_area = get_values()
        gray, binary, contours, box_image, center_image, center_list = (
            build_results(frame, v_min, min_area)
        )
        compare = build_compare(frame, gray, binary, box_image, center_image)
        cv2.imshow(window_name, resize_for_display(compare))

        if not saved_default:
            save_results(gray, binary, box_image, center_image, compare, "lesson17")
            print_center_info(contours, center_list, min_area)
            saved_default = True

        key = cv2.waitKey(80) & 0xFF
        if key == ord("s"):
            save_results(gray, binary, box_image, center_image, compare, "lesson17")
            print_center_info(contours, center_list, min_area)

        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    create_window()
    print("camera target center mode")
    print("slide v_min/min_area to change white board center result")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        v_min, min_area = get_values()
        gray, binary, contours, box_image, center_image, center_list = (
            build_results(frame, v_min, min_area)
        )
        compare = build_compare(frame, gray, binary, box_image, center_image)
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson17_camera_{time_text}"
            save_results(gray, binary, box_image, center_image, compare, prefix)
            print_center_info(contours, center_list, min_area)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
