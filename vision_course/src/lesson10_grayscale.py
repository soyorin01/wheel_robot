import sys
from datetime import datetime
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    image_args = [arg for arg in sys.argv[1:] if not arg.startswith("--")]
    if image_args:
        return Path(image_args[0]).expanduser()

    for name in ["image.png", "image.jpg", "lesson09_merged.jpg"]:
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


def wait_to_close():
    """循环等待按键，避免远程窗口一闪而过。"""
    print("press q or Esc to close windows")
    while True:
        key = cv2.waitKey(100) & 0xFF
        if key == ord("q") or key == 27:
            break
    cv2.destroyAllWindows()


def resize_for_display(image, max_width=900):
    """只缩小窗口显示画面，不影响保存的原始图像。"""
    height, width = image.shape[:2]
    if width <= max_width:
        return image
    new_height = int(height * max_width / width)
    return cv2.resize(image, (max_width, new_height))


def run_image_mode():
    image_path = get_image_path()
    image = cv2.imread(str(image_path))

    if image is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    # BGR 转灰度，输出从三通道变成单通道。
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # 为了和原图左右拼接显示，需要把灰度图临时转回三通道。
    gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    compare = cv2.hconcat([image, gray_bgr])

    gray_path = image_dir / "lesson10_gray.jpg"
    compare_path = image_dir / "lesson10_compare.jpg"
    cv2.imwrite(str(gray_path), gray)
    cv2.imwrite(str(compare_path), compare)

    print("source image:", image_path)
    print("source shape:", image.shape)
    print("gray shape:", gray.shape)
    print("saved:", gray_path)
    print("saved:", compare_path)

    cv2.namedWindow("original", cv2.WINDOW_NORMAL)
    cv2.namedWindow("gray", cv2.WINDOW_NORMAL)
    cv2.namedWindow("compare", cv2.WINDOW_NORMAL)
    cv2.imshow("original", image)
    cv2.imshow("gray", gray)
    cv2.imshow("compare", compare)
    wait_to_close()


def run_camera_mode():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    print("camera gray mode")
    print("press s to save gray image")
    print("press q or Esc to quit")
    print("display window is resized, saved image keeps original size")

    cv2.namedWindow("lesson10_camera_gray", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("lesson10_camera_gray", 900, 360)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        show = cv2.hconcat([frame, gray_bgr])
        cv2.putText(show, "left: original  right: gray", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(show, "s: save  q/Esc: quit", (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        display = resize_for_display(show, 900)
        cv2.imshow("lesson10_camera_gray", display)
        key = cv2.waitKey(30) & 0xFF

        if key == ord("s"):
            time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_path = image_dir / f"lesson10_camera_gray_{time_text}.jpg"
            ok = cv2.imwrite(str(save_path), gray)
            print("saved:", save_path if ok else "save failed")

        if key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if "--camera" in sys.argv:
    run_camera_mode()
else:
    run_image_mode()
