import sys
from datetime import datetime
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson16_draw_detection_box"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["image.png", "49.jpg"]:
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
    threshold_value = cv2.getTrackbarPos("threshold", window_name)
    min_area = cv2.getTrackbarPos("min_area", window_name)
    return threshold_value, min_area


def build_binary(frame, threshold_value):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _ret, binary = cv2.threshold(
        blur, threshold_value, 255, cv2.THRESH_BINARY_INV
    )
    return gray, binary


def draw_boxes(frame, binary, min_area):
    contours, _hierarchy = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    contour_image = frame.copy()
    box_image = frame.copy()
    rotated_box_image = frame.copy()

    kept_count = 0
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        max_area = max(max_area, area)
        if area < min_area:
            continue

        kept_count += 1
        cv2.drawContours(contour_image, [contour], -1, (0, 255, 0), 2)

        # boundingRect 会找一个水平外接矩形，把整个轮廓装进去。
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(box_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(box_image, f"x:{x} y:{y} w:{w} h:{h}",
                    (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 255), 2)

        # minAreaRect 会找一个可以旋转的最小包围矩形。
        rect = cv2.minAreaRect(contour)
        points = cv2.boxPoints(rect).astype(int)
        cv2.drawContours(rotated_box_image, [points], 0, (255, 0, 0), 2)

    cv2.putText(contour_image, f"kept contours: {kept_count}", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(box_image, f"boxes: {kept_count}  max area: {int(max_area)}",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(rotated_box_image, f"rotated boxes: {kept_count}", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    return contours, contour_image, box_image, rotated_box_image, kept_count, max_area


def build_results(frame, threshold_value, min_area):
    gray, binary = build_binary(frame, threshold_value)
    contours, contour_image, box_image, rotated_box_image, kept_count, max_area = (
        draw_boxes(frame, binary, min_area)
    )
    return gray, binary, contours, contour_image, box_image, rotated_box_image, kept_count, max_area


def build_compare(frame, gray, binary, contour_image, box_image, rotated_box_image):
    original_show = put_label(frame.copy(), "original")
    gray_show = put_label(gray_to_bgr(gray), "gray")
    binary_show = put_label(gray_to_bgr(binary), "binary")
    contour_show = put_label(contour_image.copy(), "contours")
    box_show = put_label(box_image.copy(), "bounding boxes")
    rotated_show = put_label(rotated_box_image.copy(), "rotated boxes")

    top = cv2.hconcat([original_show, gray_show, binary_show])
    bottom = cv2.hconcat([contour_show, box_show, rotated_show])
    return cv2.vconcat([top, bottom])


def save_results(gray, binary, contour_image, box_image, rotated_box_image, compare, prefix):
    outputs = {
        f"{prefix}_gray.jpg": gray,
        f"{prefix}_binary.jpg": binary,
        f"{prefix}_contours.jpg": contour_image,
        f"{prefix}_boxes.jpg": box_image,
        f"{prefix}_rotated_boxes.jpg": rotated_box_image,
        f"{prefix}_compare.jpg": compare,
    }

    for filename, image in outputs.items():
        save_path = image_dir / filename
        ok = cv2.imwrite(str(save_path), image)
        print("saved:", save_path if ok else f"save failed: {save_path}")


def nothing(_value):
    pass


def create_window():
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1000, 650)
    cv2.createTrackbar("threshold", window_name, 127, 255, nothing)
    cv2.createTrackbar("min_area", window_name, 300, 9000, nothing)


def print_box_info(contours, kept_count, max_area, min_area):
    print("total contours:", len(contours))
    print("kept contours:", kept_count)
    print("min area:", min_area)
    print("max area:", int(max_area))


def run_image_mode():
    image_path = get_image_path()
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    create_window()
    print("source image:", image_path)
    print("slide threshold/min_area to change detection boxes")
    print("press s to save current result")
    print("press q or Esc to quit")

    saved_default = False
    while True:
        threshold_value, min_area = get_values()
        gray, binary, contours, contour_image, box_image, rotated_box_image, kept_count, max_area = (
            build_results(frame, threshold_value, min_area)
        )
        compare = build_compare(
            frame, gray, binary, contour_image, box_image, rotated_box_image
        )
        cv2.imshow(window_name, resize_for_display(compare))

        if not saved_default:
            save_results(
                gray, binary, contour_image, box_image,
                rotated_box_image, compare, "lesson16"
            )
            print_box_info(contours, kept_count, max_area, min_area)
            saved_default = True

        key = cv2.waitKey(80) & 0xFF
        if key == ord("s"):
            save_results(
                gray, binary, contour_image, box_image,
                rotated_box_image, compare, "lesson16"
            )
            print_box_info(contours, kept_count, max_area, min_area)

        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    create_window()
    print("camera detection box mode")
    print("slide threshold/min_area to change detection boxes")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        threshold_value, min_area = get_values()
        gray, binary, contours, contour_image, box_image, rotated_box_image, kept_count, max_area = (
            build_results(frame, threshold_value, min_area)
        )
        compare = build_compare(
            frame, gray, binary, contour_image, box_image, rotated_box_image
        )
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson16_camera_{time_text}"
            save_results(
                gray, binary, contour_image, box_image,
                rotated_box_image, compare, prefix
            )
            print_box_info(contours, kept_count, max_area, min_area)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
