import sys
from datetime import datetime
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson12_threshold"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["image.png", "49.jpg", "lesson10_gray.jpg"]:
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
    """把单通道灰度图转成三通道，方便和原图拼接显示。"""
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def put_label(image, text):
    cv2.putText(image, text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return image


def build_threshold_images(frame, threshold_value=127):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # 固定阈值：大于阈值变白，小于等于阈值变黑。
    _ret, binary = cv2.threshold(
        gray, threshold_value, 255, cv2.THRESH_BINARY
    )

    # 反向阈值：大于阈值变黑，小于等于阈值变白。
    _ret, binary_inv = cv2.threshold(
        gray, threshold_value, 255, cv2.THRESH_BINARY_INV
    )

    # 自适应阈值：每个小区域根据局部亮度自动计算阈值。
    adaptive = cv2.adaptiveThreshold(
        gray,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        11,
        2,
    )

    return gray, binary, binary_inv, adaptive


def build_compare(frame, gray, binary, binary_inv, adaptive):
    frame_show = put_label(frame.copy(), "original")
    gray_show = put_label(gray_to_bgr(gray), "gray")
    binary_show = put_label(gray_to_bgr(binary), "binary")
    binary_inv_show = put_label(gray_to_bgr(binary_inv), "binary_inv")
    adaptive_show = put_label(gray_to_bgr(adaptive), "adaptive")

    top = cv2.hconcat([frame_show, gray_show])
    bottom = cv2.hconcat([binary_show, binary_inv_show])
    compare_left = cv2.vconcat([top, bottom])

    adaptive_resized = cv2.resize(
        adaptive_show,
        (frame.shape[1], frame.shape[0] * 2),
    )
    compare = cv2.hconcat([compare_left, adaptive_resized])
    return compare


def save_results(gray, binary, binary_inv, adaptive, compare, prefix):
    outputs = {
        f"{prefix}_gray.jpg": gray,
        f"{prefix}_binary.jpg": binary,
        f"{prefix}_binary_inv.jpg": binary_inv,
        f"{prefix}_adaptive.jpg": adaptive,
        f"{prefix}_compare.jpg": compare,
    }

    for filename, image in outputs.items():
        save_path = image_dir / filename
        ok = cv2.imwrite(str(save_path), image)
        print("saved:", save_path if ok else f"save failed: {save_path}")


def run_image_mode():
    image_path = get_image_path()
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    gray, binary, binary_inv, adaptive = build_threshold_images(frame)
    compare = build_compare(frame, gray, binary, binary_inv, adaptive)
    save_results(gray, binary, binary_inv, adaptive, compare, "lesson12")

    print("source image:", image_path)
    print("fixed threshold value: 127")
    print("press q or Esc to close window")

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1000, 650)
    cv2.imshow(window_name, resize_for_display(compare))

    while True:
        key = cv2.waitKey(100) & 0xFF
        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def nothing(_value):
    pass


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1000, 650)
    cv2.createTrackbar("threshold", window_name, 127, 255, nothing)

    print("camera threshold mode")
    print("slide threshold to compare binary result")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        threshold_value = cv2.getTrackbarPos("threshold", window_name)
        gray, binary, binary_inv, adaptive = build_threshold_images(
            frame, threshold_value
        )
        compare = build_compare(frame, gray, binary, binary_inv, adaptive)
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson12_camera_{time_text}"
            save_results(gray, binary, binary_inv, adaptive, compare, prefix)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
