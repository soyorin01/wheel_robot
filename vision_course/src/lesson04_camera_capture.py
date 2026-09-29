import cv2
from datetime import datetime
from pathlib import Path


# 自动定位 vision_course 目录，保证图片统一保存到 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

# 摄像头编号通常从 0 开始。如果有多个摄像头，可以改成 1、2 等编号。
camera_id = 0
cap = cv2.VideoCapture(camera_id)

if not cap.isOpened():
    print("camera open failed")
    print("please check /dev/video0 and camera permission")
    raise SystemExit(1)

# 设置期望分辨率。摄像头不支持时，OpenCV 会使用设备默认值。
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("lesson04_camera_capture")
print("press s to save image")
print("press q to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("read frame failed")
        break

    # 在画面上显示操作提示，方便录课时演示。
    cv2.putText(frame, "s: save  q: quit", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("lesson04_camera", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = image_dir / f"lesson04_capture_{time_text}.jpg"
        ok = cv2.imwrite(str(save_path), frame)
        print("saved:", save_path if ok else "save failed")

    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
