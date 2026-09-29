import sys
from datetime import datetime
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson11_hsv_tuner"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["image.png", "49.jpg", "lesson10_compare.jpg"]:
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


def nothing(_value):
    """Trackbar 回调函数不需要实际处理，读取滑块值时再使用。"""
    pass


def create_trackbars():
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1000, 520)

    # OpenCV 中 H 范围是 0-179，S 和 V 范围是 0-255。
    cv2.createTrackbar("H_min", window_name, 0, 179, nothing)
    cv2.createTrackbar("H_max", window_name, 179, 179, nothing)
    cv2.createTrackbar("S_min", window_name, 50, 255, nothing)
    cv2.createTrackbar("S_max", window_name, 255, 255, nothing)
    cv2.createTrackbar("V_min", window_name, 50, 255, nothing)
    cv2.createTrackbar("V_max", window_name, 255, 255, nothing)


def get_hsv_range():
    h_min = cv2.getTrackbarPos("H_min", window_name)
    h_max = cv2.getTrackbarPos("H_max", window_name)
    s_min = cv2.getTrackbarPos("S_min", window_name)
    s_max = cv2.getTrackbarPos("S_max", window_name)
    v_min = cv2.getTrackbarPos("V_min", window_name)
    v_max = cv2.getTrackbarPos("V_max", window_name)

    lower = (h_min, s_min, v_min)
    upper = (h_max, s_max, v_max)
    return lower, upper


def build_result(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower, upper = get_hsv_range()
    mask = cv2.inRange(hsv, lower, upper)
    result = cv2.bitwise_and(frame, frame, mask=mask)

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    show = cv2.hconcat([frame, mask_bgr, result])
    cv2.putText(show, "original", (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.putText(show, "mask", (frame.shape[1] + 20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.putText(show, "result", (frame.shape[1] * 2 + 20, 35),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    cv2.putText(show, "s: save  q/Esc: quit", (20, 75),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return mask, result, show


def save_result(mask, result, prefix):
    mask_path = image_dir / f"{prefix}_mask.jpg"
    result_path = image_dir / f"{prefix}_result.jpg"
    cv2.imwrite(str(mask_path), mask)
    cv2.imwrite(str(result_path), result)
    print("saved:", mask_path)
    print("saved:", result_path)


def run_image_mode():
    image_path = get_image_path()
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    create_trackbars()
    print("image mode:", image_path)
    print("adjust HSV trackbars")
    print("press s to save mask and result")
    print("press q or Esc to quit")

    while True:
        mask, result, show = build_result(frame)
        cv2.imshow(window_name, resize_for_display(show))
        key = cv2.waitKey(80) & 0xFF

        if key == ord("s"):
            save_result(mask, result, "lesson11_hsv")

        if key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def run_camera_mode():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    create_trackbars()
    print("camera mode")
    print("adjust HSV trackbars")
    print("press s to save mask and result")
    print("press q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        mask, result, show = build_result(frame)
        cv2.imshow(window_name, resize_for_display(show))
        key = cv2.waitKey(30) & 0xFF

        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_result(mask, result, f"lesson11_hsv_camera_{time_text}")

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
