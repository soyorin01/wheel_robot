import argparse
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


# 本节只做视觉任务输出形式演示，不加载神经网络或YOLO模型。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)
default_image = image_dir / "car.png"

window_name = "lesson25_ai_vision_basics"
panel_width = 480


def resize_to_width(image, width):
    """保持宽高比缩放图片，便于组成四宫格。"""
    height, old_width = image.shape[:2]
    new_height = int(height * width / old_width)
    return cv2.resize(image, (width, new_height))


def add_header(image, title, subtitle):
    """在面板顶部加入任务名称和任务问题。"""
    header_height = 58
    panel = cv2.copyMakeBorder(
        image, header_height, 0, 0, 0,
        cv2.BORDER_CONSTANT, value=(28, 28, 28)
    )
    cv2.putText(
        panel, title, (12, 23), cv2.FONT_HERSHEY_SIMPLEX,
        0.62, (0, 255, 255), 2, cv2.LINE_AA
    )
    cv2.putText(
        panel, subtitle, (12, 48), cv2.FONT_HERSHEY_SIMPLEX,
        0.48, (230, 230, 230), 1, cv2.LINE_AA
    )
    return panel


def scale_box(box, width, height):
    """把0～1的相对坐标转换为当前图片像素坐标。"""
    x1, y1, x2, y2 = box
    return (
        int(x1 * width), int(y1 * height),
        int(x2 * width), int(y2 * height),
    )


def classification_panel(source):
    """分类任务：对整张图片给出类别，不输出目标位置。"""
    image = resize_to_width(source, panel_width)
    overlay = image.copy()
    cv2.rectangle(overlay, (15, 15), (310, 94), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.65, image, 0.35, 0, image)
    cv2.putText(
        image, "scene: parking lot", (28, 48),
        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 0), 2, cv2.LINE_AA
    )
    cv2.putText(
        image, "main object: car", (28, 80),
        cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 255, 0), 2, cv2.LINE_AA
    )
    return add_header(image, "1  IMAGE CLASSIFICATION", "Question: What is in the whole image?")


def detection_panel(source):
    """检测任务：用检测框表示多个目标的位置和类别。"""
    image = resize_to_width(source, panel_width)
    height, width = image.shape[:2]
    boxes = [
        (0.00, 0.46, 0.58, 0.99),
        (0.34, 0.47, 0.75, 0.88),
        (0.54, 0.47, 0.87, 0.80),
        (0.69, 0.47, 0.98, 0.73),
    ]

    for index, box in enumerate(boxes, start=1):
        x1, y1, x2, y2 = scale_box(box, width, height)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label_y = max(18, y1 - 6)
        cv2.putText(
            image, f"car {index}", (x1 + 3, label_y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 0), 2, cv2.LINE_AA
        )
    return add_header(image, "2  OBJECT DETECTION", "Question: What objects are where?")


def segmentation_panel(source):
    """分割任务：用半透明颜色表示每个像素所属的类别区域。"""
    image = resize_to_width(source, panel_width)
    height, width = image.shape[:2]
    overlay = image.copy()

    # 以下区域为教学标注，用于模拟语义分割的输出形式。
    sky_mask = np.zeros((height, width), dtype=np.uint8)
    road_mask = np.zeros((height, width), dtype=np.uint8)
    car_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.rectangle(sky_mask, (0, 0), (width, int(0.43 * height)), 255, -1)
    cv2.rectangle(road_mask, (0, int(0.56 * height)), (width, height), 255, -1)

    car_polygons = [
        np.array([
            [0, int(0.56 * height)], [int(0.10 * width), int(0.49 * height)],
            [int(0.39 * width), int(0.50 * height)], [int(0.58 * width), int(0.74 * height)],
            [int(0.51 * width), int(0.92 * height)], [int(0.13 * width), int(0.90 * height)],
            [0, int(0.82 * height)],
        ], dtype=np.int32),
        np.array([
            [int(0.35 * width), int(0.51 * height)], [int(0.50 * width), int(0.48 * height)],
            [int(0.75 * width), int(0.64 * height)], [int(0.73 * width), int(0.84 * height)],
            [int(0.52 * width), int(0.86 * height)],
        ], dtype=np.int32),
        np.array([
            [int(0.56 * width), int(0.50 * height)], [int(0.68 * width), int(0.48 * height)],
            [int(0.88 * width), int(0.62 * height)], [int(0.85 * width), int(0.77 * height)],
            [int(0.68 * width), int(0.78 * height)],
        ], dtype=np.int32),
    ]
    cv2.fillPoly(car_mask, car_polygons, 255)

    overlay[sky_mask > 0] = (255, 150, 40)
    overlay[road_mask > 0] = (180, 60, 180)
    overlay[car_mask > 0] = (40, 220, 40)
    cv2.addWeighted(overlay, 0.38, image, 0.62, 0, image)

    cv2.putText(image, "sky", (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "car", (20, int(0.70 * height)), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(image, "road", (width - 72, height - 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (255, 255, 255), 2, cv2.LINE_AA)
    return add_header(image, "3  SEMANTIC SEGMENTATION", "Question: Which class owns each pixel?")


def keypoint_panel(source):
    """关键点任务：用结构点表示主车辆的轮子、车灯和外形位置。"""
    image = resize_to_width(source, panel_width)
    height, width = image.shape[:2]
    normalized_points = [
        (0.04, 0.59), (0.19, 0.51), (0.39, 0.52), (0.56, 0.72),
        (0.49, 0.88), (0.35, 0.90), (0.12, 0.86),
    ]
    points = [
        (int(x * width), int(y * height))
        for x, y in normalized_points
    ]

    # 连线用于显示目标结构关系，圆点表示关键位置。
    for first, second in zip(points, points[1:]):
        cv2.line(image, first, second, (0, 255, 255), 2)
    for index, point in enumerate(points, start=1):
        cv2.circle(image, point, 5, (0, 0, 255), -1)
        cv2.putText(
            image, str(index), (point[0] + 5, point[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, (0, 0, 255), 1, cv2.LINE_AA
        )
    return add_header(image, "4  KEYPOINT DETECTION", "Question: Where are the object's structure points?")


def normalize_panel_height(panels):
    """把四个面板补到相同高度，保证拼接时尺寸一致。"""
    target_height = max(panel.shape[0] for panel in panels)
    normalized = []
    for panel in panels:
        bottom = target_height - panel.shape[0]
        normalized.append(cv2.copyMakeBorder(
            panel, 0, bottom, 0, 0,
            cv2.BORDER_CONSTANT, value=(20, 20, 20)
        ))
    return normalized


def build_demo(source):
    """生成四类任务面板和2×2总览画面。"""
    panels = normalize_panel_height([
        classification_panel(source),
        detection_panel(source),
        segmentation_panel(source),
        keypoint_panel(source),
    ])
    overview = cv2.vconcat([
        cv2.hconcat([panels[0], panels[1]]),
        cv2.hconcat([panels[2], panels[3]]),
    ])

    # 总览顶部明确说明这是人工教学标注，不是AI模型的推理结果。
    banner = np.full((42, overview.shape[1], 3), (15, 15, 15), dtype=np.uint8)
    cv2.putText(
        banner, "CONCEPT DEMO - MANUAL ANNOTATIONS, NOT MODEL INFERENCE",
        (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.68,
        (0, 165, 255), 2, cv2.LINE_AA
    )
    overview = cv2.vconcat([banner, overview])
    return panels, overview


def save_display(image, mode_name):
    """保存当前演示画面。"""
    time_text = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = image_dir / f"lesson25_{mode_name}_{time_text}.jpg"
    ok = cv2.imwrite(str(path), image)
    print("saved:" if ok else "save failed:", path)


def run_demo(image_path):
    source = cv2.imread(str(image_path))
    if source is None:
        print("image read failed:", image_path)
        raise SystemExit(1)

    panels, overview = build_demo(source)
    displays = {
        ord("0"): ("overview", overview),
        ord("1"): ("classification", panels[0]),
        ord("2"): ("detection", panels[1]),
        ord("3"): ("segmentation", panels[2]),
        ord("4"): ("keypoints", panels[3]),
    }
    current_name, current_image = displays[ord("0")]

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 1100, 760)
    print("concept demonstration only; no AI model is loaded")
    print("0: overview  1: classification  2: detection")
    print("3: segmentation  4: keypoints  s: save  q/Esc: quit")

    while True:
        cv2.imshow(window_name, current_image)
        key = cv2.waitKey(0) & 0xFF
        if key in displays:
            current_name, current_image = displays[key]
            print("show:", current_name)
        elif key == ord("s"):
            save_display(current_image, current_name)
        elif key == ord("q") or key == 27:
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Lesson 25: visual comparison of four AI vision tasks"
    )
    parser.add_argument(
        "--image", type=Path, default=default_image,
        help="demonstration image path, default: images/car.png"
    )
    args = parser.parse_args()
    run_demo(args.image)
