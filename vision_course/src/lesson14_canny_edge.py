import sys
from datetime import datetime
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson14_canny_edge"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["car.png", "49.jpg"]:
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


def get_thresholds():
    low = cv2.getTrackbarPos("low", window_name)
    high = cv2.getTrackbarPos("high", window_name)
    if high <= low:
        high = low + 1
    return low, high


def build_results(frame, low, high):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Canny 前先高斯滤波，减少噪声导致的杂乱边缘。
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # low 和 high 是双阈值，控制弱边缘和强边缘的保留范围。
    edges = cv2.Canny(blur, low, high)

    # 把边缘用红色叠加到原图上，方便观察边缘位置。
    overlay = frame.copy()
    overlay[edges > 0] = (0, 0, 255)

    return gray, blur, edges, overlay


def build_compare(frame, gray, blur, edges, overlay):
    original_show = put_label(frame.copy(), "original")
    gray_show = put_label(gray_to_bgr(gray), "gray")
    blur_show = put_label(gray_to_bgr(blur), "gaussian blur")
    edges_show = put_label(gray_to_bgr(edges), "canny edges")
    overlay_show = put_label(overlay.copy(), "edge overlay")

    top = cv2.hconcat([original_show, gray_show, blur_show])
    bottom = cv2.hconcat([edges_show, overlay_show])
    bottom = cv2.resize(bottom, (top.shape[1], top.shape[0]))
    return cv2.vconcat([top, bottom])


def save_results(gray, blur, edges, overlay, compare, prefix):
    outputs = {
        f"{prefix}_gray.jpg": gray,
        f"{prefix}_blur.jpg": blur,
        f"{prefix}_edges.jpg": edges,
        f"{prefix}_overlay.jpg": overlay,
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
    cv2.createTrackbar("low", window_name, 50, 255, nothing)
    cv2.createTrackbar("high", window_name, 150, 255, nothing)


def run_image_mode():
    image_path = get_image_path()
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    create_window()
    print("source image:", image_path)
    print("slide low/high to change Canny edge result")
    print("press s to save current result")
    print("press q or Esc to quit")

    saved_default = False
    while True:
        low, high = get_thresholds()
        gray, blur, edges, overlay = build_results(frame, low, high)
        compare = build_compare(frame, gray, blur, edges, overlay)
        cv2.imshow(window_name, resize_for_display(compare))

        if not saved_default:
            save_results(gray, blur, edges, overlay, compare, "lesson14")
            saved_default = True

        key = cv2.waitKey(80) & 0xFF
        if key == ord("s"):
            save_results(gray, blur, edges, overlay, compare, "lesson14")

        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    create_window()
    print("camera Canny edge mode")
    print("slide low/high to change edge result")
    print("press s to save current result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        low, high = get_thresholds()
        gray, blur, edges, overlay = build_results(frame, low, high)
        compare = build_compare(frame, gray, blur, edges, overlay)
        cv2.imshow(window_name, resize_for_display(compare))

        key = cv2.waitKey(30) & 0xFF
        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            prefix = f"lesson14_camera_{time_text}"
            save_results(gray, blur, edges, overlay, compare, prefix)

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
