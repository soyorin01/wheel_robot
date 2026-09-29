import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson13_filter_morphology"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["dige.png", "49.jpg", "lesson12_compare.jpg"]:
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
    """把单通道图像转成三通道，方便和原图拼接显示。"""
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def put_label(image, text):
    cv2.putText(image, text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    return image


def build_results(frame, threshold_value=127):
    # 均值滤波：用邻域平均值替换当前像素，能让画面变平滑。
    mean_blur = cv2.blur(frame, (5, 5))

    # 高斯滤波：中心像素权重大，边缘像素权重小，降噪更自然。
    gaussian_blur = cv2.GaussianBlur(frame, (5, 5), 0)

    # 本节课让二值化直接基于原图灰度图，方便观察原始白色区域。
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _ret, binary = cv2.threshold(
        gray, threshold_value, 255, cv2.THRESH_BINARY
    )

    # 结构元素决定腐蚀和膨胀处理时观察的邻域大小。
    kernel = np.ones((5, 5), np.uint8)

    # 腐蚀会让白色区域变小，适合去掉小白点噪声。
    eroded = cv2.erode(binary, kernel, iterations=1)

    # 膨胀会让白色区域变大，适合连接断裂区域。
    dilated = cv2.dilate(binary, kernel, iterations=1)

    return mean_blur, gaussian_blur, binary, eroded, dilated


def build_compare(frame, mean_blur, gaussian_blur, binary, eroded, dilated):
    original_show = put_label(frame.copy(), "original")
    mean_show = put_label(mean_blur.copy(), "mean blur")
    gaussian_show = put_label(gaussian_blur.copy(), "gaussian blur")
    binary_show = put_label(gray_to_bgr(binary), "binary")
    eroded_show = put_label(gray_to_bgr(eroded), "erode")
    dilated_show = put_label(gray_to_bgr(dilated), "dilate")

    top = cv2.hconcat([original_show, mean_show, gaussian_show])
    bottom = cv2.hconcat([binary_show, eroded_show, dilated_show])
    return cv2.vconcat([top, bottom])


def save_results(mean_blur, gaussian_blur, binary, eroded, dilated, compare, prefix):
    outputs = {
        f"{prefix}_mean_blur.jpg": mean_blur,
        f"{prefix}_gaussian_blur.jpg": gaussian_blur,
        f"{prefix}_binary.jpg": binary,
        f"{prefix}_eroded.jpg": eroded,
        f"{prefix}_dilated.jpg": dilated,
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

    mean_blur, gaussian_blur, binary, eroded, dilated = build_results(frame)
    compare = build_compare(
        frame, mean_blur, gaussian_blur, binary, eroded, dilated
    )
    save_results(
        mean_blur, gaussian_blur, binary, eroded, dilated, compare, "lesson13"
    )

    print("source image:", image_path)
    print("kernel size: 5x5")
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

    print("camera filter and morphology mode")
    print("slide threshold to change binary result")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        threshold_value = cv2.getTrackbarPos("threshold", window_name)
        mean_blur, gaussian_blur, binary, eroded, dilated = build_results(
            frame, threshold_value
        )
        compare = build_compare(
            frame, mean_blur, gaussian_blur, binary, eroded, dilated
        )
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson13_camera_{time_text}"
            save_results(
                mean_blur, gaussian_blur, binary, eroded, dilated, compare, prefix
            )

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
