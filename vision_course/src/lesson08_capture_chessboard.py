import cv2
from datetime import datetime
from pathlib import Path


# 标定图片属于课程图片素材，统一保存到 vision_course/images/calibration。
project_dir = Path(__file__).resolve().parent.parent
calib_dir = project_dir / "images" / "calibration"
calib_dir.mkdir(parents=True, exist_ok=True)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("camera open failed")
    raise SystemExit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("lesson08_capture_chessboard")
print("put chessboard in different positions")
print("press s to save image")
print("press q or Esc to quit")

while True:
    ret, frame = cap.read()
    if not ret:
        print("read frame failed")
        break

    show = frame.copy()
    cv2.putText(show, "s: save  q/Esc: quit", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.imshow("lesson08_capture", show)

    key = cv2.waitKey(30) & 0xFF

    if key == ord("s"):
        time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
        save_path = calib_dir / f"lesson08_calib_{time_text}.jpg"
        ok = cv2.imwrite(str(save_path), frame)
        print("saved:", save_path if ok else "save failed")

    if key == ord("q") or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
