import argparse
from collections import deque
from datetime import datetime
from pathlib import Path
import math
import time

import cv2


# 课程产生的截图统一保存到 vision_course/images 目录。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

window_name = "lesson21_target_tracking"
tracker_name = "CSRT"

# 轨迹最多保存120个中心点，避免程序长时间运行后列表无限增长。
trajectory = deque(maxlen=120)

# 运动估计只使用最近约0.5秒的数据，可降低单帧抖动对速度的影响。
motion_history = deque()
motion_window_seconds = 0.5
stationary_speed = 20.0


def create_tracker():
    """兼容不同OpenCV版本，优先创建CSRT跟踪器。"""
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()

    print("CSRT tracker is unavailable")
    print("please install an OpenCV build with tracking support")
    raise SystemExit(1)


def select_box(frame):
    """让用户在摄像头画面或静态照片上拖动鼠标选择目标。"""
    select_window = "select target and press Enter"
    cv2.namedWindow(select_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(select_window, 960, 720)
    box = cv2.selectROI(
        select_window, frame, showCrosshair=True, fromCenter=False
    )
    cv2.destroyWindow(select_window)

    x, y, w, h = [int(value) for value in box]
    if w <= 1 or h <= 1:
        print("target selection cancelled")
        return None

    return x, y, w, h


def select_target(frame):
    """框选目标并用所选区域初始化摄像头跟踪器。"""
    box = select_box(frame)
    if box is None:
        return None, None

    x, y, w, h = box
    tracker = create_tracker()
    tracker.init(frame, (x, y, w, h))
    print(f"target selected: x={x}, y={y}, w={w}, h={h}")
    return tracker, (x, y, w, h)


def clamp_box(box, frame_shape):
    """把跟踪框限制在画面内部，防止绘图坐标越界。"""
    frame_height, frame_width = frame_shape[:2]
    x, y, w, h = [int(value) for value in box]
    x = max(0, min(x, frame_width - 1))
    y = max(0, min(y, frame_height - 1))
    w = max(0, min(w, frame_width - x))
    h = max(0, min(h, frame_height - y))
    return x, y, w, h


def update_motion(center, timestamp):
    """记录中心点，并使用短时间窗口估算方向和像素速度。"""
    trajectory.append(center)
    motion_history.append((timestamp, center))

    # 删除时间窗口之外的旧数据，只比较最近一段运动。
    while (
        len(motion_history) > 2
        and timestamp - motion_history[0][0] > motion_window_seconds
    ):
        motion_history.popleft()

    if len(motion_history) < 2:
        return "STILL", 0.0, (0, 0)

    old_time, old_center = motion_history[0]
    new_time, new_center = motion_history[-1]
    delta_time = max(new_time - old_time, 1e-6)
    dx = new_center[0] - old_center[0]
    dy = new_center[1] - old_center[1]
    speed = math.hypot(dx, dy) / delta_time

    # 低于阈值的轻微抖动视为静止。
    if speed < stationary_speed:
        return "STILL", speed, (dx, dy)

    if abs(dx) >= abs(dy):
        direction = "RIGHT" if dx > 0 else "LEFT"
    else:
        direction = "DOWN" if dy > 0 else "UP"
    return direction, speed, (dx, dy)


def draw_trajectory(image):
    """依次连接历史中心点，形成目标运动轨迹。"""
    points = list(trajectory)
    for index in range(1, len(points)):
        cv2.line(image, points[index - 1], points[index], (0, 255, 255), 2)


def draw_info_panel(image, lines, margin=12):
    """绘制自动缩放的信息面板，保证文字不会超出画面右边界。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    default_scale = 0.62
    thickness = 2
    padding = 10
    line_gap = 7
    available_width = max(80, image.shape[1] - 2 * margin - 2 * padding)

    widths = [
        cv2.getTextSize(line, font, default_scale, thickness)[0][0]
        for line in lines
    ]
    longest_width = max(widths, default=1)
    font_scale = min(default_scale, default_scale * available_width / longest_width)
    font_scale = max(0.30, font_scale)

    metrics = [cv2.getTextSize(line, font, font_scale, thickness) for line in lines]
    text_width = max((item[0][0] for item in metrics), default=1)
    line_height = max((item[0][1] + item[1] for item in metrics), default=16)
    panel_width = min(image.shape[1] - 2 * margin, text_width + 2 * padding)
    panel_height = len(lines) * line_height + (len(lines) - 1) * line_gap + 2 * padding

    x1, y1 = margin, margin
    x2 = min(image.shape[1] - 1, x1 + panel_width)
    y2 = min(image.shape[0] - 1, y1 + panel_height)

    overlay = image.copy()
    cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.60, image, 0.40, 0, image)

    text_y = y1 + padding + line_height
    for line in lines:
        cv2.putText(
            image, line, (x1 + padding, text_y), font,
            font_scale, (0, 255, 0), thickness, cv2.LINE_AA
        )
        text_y += line_height + line_gap
    return x1, y1, x2, y2


def draw_controls(image, text="r: select  c: clear path  s: save  q/Esc: quit"):
    """在底部显示按键说明，并根据画面宽度自动缩放。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 2
    text_width = cv2.getTextSize(text, font, scale, thickness)[0][0]
    if text_width > image.shape[1] - 24:
        scale *= (image.shape[1] - 24) / text_width
    cv2.putText(
        image, text, (12, image.shape[0] - 16), font,
        max(scale, 0.35), (255, 255, 255), thickness, cv2.LINE_AA
    )


def save_result(result, prefix="lesson21_tracking"):
    time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = image_dir / f"{prefix}_{time_text}.jpg"
    ok = cv2.imwrite(str(save_path), result)
    print("saved:" if ok else "save failed:", save_path)


def build_image_result(frame, selected_box=None):
    """为静态照片绘制目标位置；该函数不负责弹窗，便于独立测试。"""
    result = frame.copy()
    if selected_box is not None:
        x, y, w, h = clamp_box(selected_box, frame.shape)
        center = (x + w // 2, y + h // 2)
        cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.circle(result, center, 6, (0, 0, 255), -1)
        info_lines = [
            "mode: IMAGE",
            "state: TARGET SELECTED",
            f"center: {center}",
            f"size: {w} x {h} px",
        ]
    else:
        info_lines = [
            "mode: IMAGE",
            "state: WAITING",
            "press r to select target",
            "motion: N/A for a single image",
        ]

    draw_info_panel(result, info_lines)
    draw_controls(result, "r: select  s: save  q/Esc: quit")
    return result


def run_image_mode(image_path):
    """读取静态照片，完成目标框选、中心定位和标注结果保存。"""
    frame = cv2.imread(str(image_path))
    if frame is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    selected_box = None
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    print("image mode")
    print("press r to select target, s to save, q or Esc to quit")

    while True:
        result = build_image_result(frame, selected_box)
        cv2.imshow(window_name, result)

        key = cv2.waitKey(0) & 0xFF
        if key == ord("r"):
            selected_box = select_box(frame)
        elif key == ord("s"):
            save_result(result, "lesson21_image_target")
        elif key == ord("q") or key == 27:
            break

    cv2.destroyAllWindows()


def run_camera_mode(camera_index=0):
    """打开摄像头，执行连续目标跟踪和运动分析。"""
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("camera open failed")
        raise SystemExit(1)

    tracker = None
    last_box = None
    state = "WAITING"
    direction = "STILL"
    speed = 0.0
    center = None

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 720)
    print("press r to select a target")
    print("press c to clear trajectory, s to save, q or Esc to quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("read frame failed")
            break

        result = frame.copy()

        if tracker is not None:
            ok, updated_box = tracker.update(frame)
            if ok:
                x, y, w, h = clamp_box(updated_box, frame.shape)
                if w > 1 and h > 1:
                    last_box = (x, y, w, h)
                    center = (x + w // 2, y + h // 2)
                    direction, speed, motion_delta = update_motion(
                        center, time.perf_counter()
                    )
                    state = "TRACKING"

                    cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 3)
                    cv2.circle(result, center, 6, (0, 0, 255), -1)
                    if direction != "STILL" and len(motion_history) >= 2:
                        cv2.arrowedLine(
                            result, motion_history[0][1], center,
                            (255, 255, 0), 2, tipLength=0.25
                        )
                else:
                    ok = False

            if not ok:
                # 跟踪失败后停止更新，等待用户按r重新选择目标。
                tracker = None
                state = "LOST - PRESS R TO RESELECT"
                direction = "UNKNOWN"
                speed = 0.0
                center = None
                motion_history.clear()
                print("target lost, press r to select again")

        draw_trajectory(result)

        if state == "TRACKING" and center is not None:
            info_lines = [
                f"tracker: {tracker_name}  state: {state}",
                f"center: {center}",
                f"direction: {direction}",
                f"speed: {speed:.1f} px/s",
            ]
        else:
            info_lines = [
                f"tracker: {tracker_name}",
                f"state: {state}",
                "press r to select target",
            ]

        draw_info_panel(result, info_lines)
        draw_controls(result)
        cv2.imshow(window_name, result)

        key = cv2.waitKey(30) & 0xFF
        if key == ord("r"):
            tracker, selected_box = select_target(frame)
            if tracker is not None:
                last_box = selected_box
                state = "TRACKING"
                direction = "STILL"
                speed = 0.0
                trajectory.clear()
                motion_history.clear()
        elif key == ord("c"):
            trajectory.clear()
            motion_history.clear()
            direction = "STILL"
            speed = 0.0
            print("trajectory cleared")
        elif key == ord("s"):
            save_result(result)
            if center is not None:
                print(
                    f"center={center}, direction={direction}, "
                    f"speed={speed:.1f} px/s, box={last_box}"
                )
        elif key == ord("q") or key == 27:
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Lesson 21: image target selection or camera tracking"
    )
    parser.add_argument(
        "--image", type=Path,
        help="read a photo instead of opening the camera"
    )
    parser.add_argument(
        "--camera", type=int, default=0,
        help="camera device index, default: 0"
    )
    args = parser.parse_args()

    if args.image is not None:
        run_image_mode(args.image)
    else:
        run_camera_mode(args.camera)
