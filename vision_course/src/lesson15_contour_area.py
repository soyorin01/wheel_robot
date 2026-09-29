import sys
from datetime import datetime
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson15_contour_area"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["car.png", "image.png", "49.jpg"]:
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

    # 本节课使用反向二值化，适合把较暗目标变成白色前景。
    _ret, binary = cv2.threshold(
        blur, threshold_value, 255, cv2.THRESH_BINARY_INV
    )
    return gray, blur, binary


def find_and_draw_contours(frame, binary, min_area):
    # findContours 会在白色区域边界上找点，contours 就是这些点的集合。
    contours, hierarchy = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    all_contours = frame.copy()
    filtered_contours = frame.copy()

    cv2.drawContours(all_contours, contours, -1, (0, 255, 0), 2)

    kept_count = 0
    max_area = 0
    for contour in contours:
        area = cv2.contourArea(contour)
        max_area = max(max_area, area)

        if area < min_area:
            continue

        kept_count += 1
        cv2.drawContours(filtered_contours, [contour], -1, (0, 0, 255), 2)

        # 用轮廓第一个点附近显示面积，先不讲检测框和中心点。
        x, y = contour[0][0]
        cv2.putText(filtered_contours, f"{int(area)}", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    cv2.putText(all_contours, f"contours: {len(contours)}", (20, 80),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(filtered_contours, f"kept: {kept_count}  max: {int(max_area)}",
                (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    return contours, hierarchy, all_contours, filtered_contours, kept_count, max_area


def build_results(frame, threshold_value, min_area):
    gray, blur, binary = build_binary(frame, threshold_value)
    contours, hierarchy, all_contours, filtered_contours, kept_count, max_area = (
        find_and_draw_contours(frame, binary, min_area)
    )
    return gray, blur, binary, contours, hierarchy, all_contours, filtered_contours, kept_count, max_area


def build_compare(frame, gray, binary, all_contours, filtered_contours):
    original_show = put_label(frame.copy(), "original")
    gray_show = put_label(gray_to_bgr(gray), "gray")
    binary_show = put_label(gray_to_bgr(binary), "binary")
    all_show = put_label(all_contours.copy(), "all contours")
    filtered_show = put_label(filtered_contours.copy(), "area filtered")

    top = cv2.hconcat([original_show, gray_show, binary_show])
    bottom = cv2.hconcat([all_show, filtered_show])
    bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))
    return cv2.vconcat([top, bottom])


def save_results(gray, binary, all_contours, filtered_contours, compare, prefix):
    outputs = {
        f"{prefix}_gray.jpg": gray,
        f"{prefix}_binary.jpg": binary,
        f"{prefix}_all_contours.jpg": all_contours,
        f"{prefix}_filtered_contours.jpg": filtered_contours,
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
    cv2.createTrackbar("min_area", window_name, 300, 5000000, nothing)


def print_contour_info(contours, kept_count, max_area):
    areas = sorted([cv2.contourArea(c) for c in contours], reverse=True)
    print("total contours:", len(contours))
    print("kept contours:", kept_count)
    print("max area:", int(max_area))
    print("top 5 areas:", [int(area) for area in areas[:5]])


def run_image_mode():
    image_path = get_image_path()
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    create_window()
    print("source image:", image_path)
    print("slide threshold/min_area to change contour result")
    print("press s to save current result")
    print("press q or Esc to quit")

    saved_default = False
    while True:
        threshold_value, min_area = get_values()
        gray, _blur, binary, contours, _hierarchy, all_contours, filtered_contours, kept_count, max_area = (
            build_results(frame, threshold_value, min_area)
        )
        compare = build_compare(frame, gray, binary, all_contours, filtered_contours)
        cv2.imshow(window_name, resize_for_display(compare))

        if not saved_default:
            save_results(gray, binary, all_contours, filtered_contours, compare, "lesson15")
            print_contour_info(contours, kept_count, max_area)
            saved_default = True

        key = cv2.waitKey(80) & 0xFF
        if key == ord("s"):
            save_results(gray, binary, all_contours, filtered_contours, compare, "lesson15")
            print_contour_info(contours, kept_count, max_area)

        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    create_window()
    print("camera contour area mode")
    print("slide threshold/min_area to change contour result")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        threshold_value, min_area = get_values()
        gray, _blur, binary, contours, _hierarchy, all_contours, filtered_contours, kept_count, max_area = (
            build_results(frame, threshold_value, min_area)
        )
        compare = build_compare(frame, gray, binary, all_contours, filtered_contours)
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson15_camera_{time_text}"
            save_results(gray, binary, all_contours, filtered_contours, compare, prefix)
            print_contour_info(contours, kept_count, max_area)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
