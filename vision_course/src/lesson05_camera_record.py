import cv2
import argparse
import time
from datetime import datetime
from pathlib import Path


# 摄像头格式预设。
# 切换方法 1：修改 default_preset 的值。
# 切换方法 2：运行时传入预设名，例如：
# python3 src/lesson05_camera_record.py mjpg_640x480_120
# 还可以加 --no-record 或 --no-display，用来排查录像和远程弹窗对帧率的影响。
CAMERA_PRESETS = {
    "mjpg_1280x720_60": {"fourcc": "MJPG", "width": 1280, "height": 720, "fps": 60.0},
    "mjpg_1920x1080_30": {"fourcc": "MJPG", "width": 1920, "height": 1080, "fps": 30.0},
    "mjpg_1024x768_30": {"fourcc": "MJPG", "width": 1024, "height": 768, "fps": 30.0},
    "mjpg_640x480_120": {"fourcc": "MJPG", "width": 640, "height": 480, "fps": 120.0},
    "mjpg_800x600_60": {"fourcc": "MJPG", "width": 800, "height": 600, "fps": 60.0},
    "mjpg_1280x1024_30": {"fourcc": "MJPG", "width": 1280, "height": 1024, "fps": 30.0},
    "mjpg_320x240_120": {"fourcc": "MJPG", "width": 320, "height": 240, "fps": 120.0},
    "yuyv_1280x720_9": {"fourcc": "YUYV", "width": 1280, "height": 720, "fps": 9.0},
    "yuyv_1920x1080_6": {"fourcc": "YUYV", "width": 1920, "height": 1080, "fps": 6.0},
    "yuyv_1024x768_6": {"fourcc": "YUYV", "width": 1024, "height": 768, "fps": 6.0},
    "yuyv_640x480_30": {"fourcc": "YUYV", "width": 640, "height": 480, "fps": 30.0},
    "yuyv_800x600_20": {"fourcc": "YUYV", "width": 800, "height": 600, "fps": 20.0},
    "yuyv_1280x1024_6": {"fourcc": "YUYV", "width": 1280, "height": 1024, "fps": 6.0},
    "yuyv_320x240_30": {"fourcc": "YUYV", "width": 320, "height": 240, "fps": 30.0},
}


default_preset = "yuyv_640x480_30"

parser = argparse.ArgumentParser(description="OpenCV camera record demo")
parser.add_argument("preset", nargs="?", default=default_preset,
                    help="camera preset name")
parser.add_argument("--no-record", action="store_true",
                    help="do not save video, only preview/test fps")
parser.add_argument("--no-display", action="store_true",
                    help="do not show preview window, only record/test fps")
parser.add_argument("--seconds", type=float, default=0.0,
                    help="auto stop after N seconds, 0 means press q or Ctrl+C")
args = parser.parse_args()

preset_name = args.preset
if preset_name not in CAMERA_PRESETS:
    print("unknown camera preset:", preset_name)
    print("available presets:")
    for name in CAMERA_PRESETS:
        print(" ", name)
    raise SystemExit(1)

preset = CAMERA_PRESETS[preset_name]

# 自动定位 vision_course 目录，视频统一保存到 vision_course/videos。
project_dir = Path(__file__).resolve().parent.parent
video_dir = project_dir / "videos"
video_dir.mkdir(exist_ok=True)

camera_id = 0
# 明确使用 V4L2 后端打开 Linux 摄像头，避免 OpenCV 默认走 GStreamer 后端时
# 无法正确设置 MJPG、分辨率和帧率。
cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L2)

if not cap.isOpened():
    print("camera open failed")
    print("please check /dev/video0 and camera permission")
    raise SystemExit(1)

backend_name = cap.getBackendName()

# 设置摄像头格式、目标分辨率和目标帧率。
target_fourcc = preset["fourcc"]
target_width = preset["width"]
target_height = preset["height"]
target_fps = preset["fps"]

cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*target_fourcc))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
cap.set(cv2.CAP_PROP_FPS, target_fps)

width = target_width
height = target_height
video_fps = target_fps
actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_fps = cap.get(cv2.CAP_PROP_FPS)
actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
actual_fourcc_text = "".join(chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4))
if actual_fps > 1:
    video_fps = actual_fps

time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
save_path = video_dir / f"lesson05_record_{time_text}.avi"

# MJPG + AVI 在开发板环境中兼容性较好，适合课程演示。
fourcc = cv2.VideoWriter_fourcc(*"MJPG")
writer = None
if not args.no_record:
    writer = cv2.VideoWriter(str(save_path), fourcc, video_fps, (width, height))

if writer is not None and not writer.isOpened():
    print("video writer open failed")
    cap.release()
    raise SystemExit(1)

print("lesson05_camera_record")
print("preset:", preset_name)
print("backend:", backend_name)
print("recording:", "off" if args.no_record else save_path)
print("display:", "off" if args.no_display else "on")
print(f"request camera: {target_fourcc} {target_width}x{target_height} {target_fps:.1f} fps")
print(f"actual camera : {actual_fourcc_text} {actual_width}x{actual_height} {actual_fps:.1f} fps")
print(f"window image size: {width}x{height}")
print(f"video save fps: {video_fps:.1f}")
print("press q to stop" if not args.no_display else "press Ctrl+C to stop")

window_name = "lesson05_record"
if not args.no_display:
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, width, height)

last_time = time.perf_counter()
start_time = last_time
real_fps = 0.0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        # 写入视频文件的画面尺寸要和 VideoWriter 初始化尺寸一致。
        frame = cv2.resize(frame, (width, height))

        now = time.perf_counter()
        elapsed = now - last_time
        last_time = now
        if elapsed > 0:
            current_fps = 1.0 / elapsed
            if real_fps == 0.0:
                real_fps = current_fps
            else:
                real_fps = real_fps * 0.9 + current_fps * 0.1

        if not args.no_display:
            status_text = "PREVIEW  q: stop" if args.no_record else "REC  q: stop"
            cv2.putText(frame, status_text, (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            cv2.putText(frame, f"LOOP FPS: {real_fps:.1f}", (20, 75),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if writer is not None:
            writer.write(frame)

        if not args.no_display:
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

        if args.seconds > 0 and now - start_time >= args.seconds:
            break
except KeyboardInterrupt:
    print("stopped by Ctrl+C")

cap.release()
if writer is not None:
    writer.release()
if not args.no_display:
    cv2.destroyAllWindows()

print(f"last loop fps: {real_fps:.1f}")
if writer is not None:
    print("saved:", save_path)
