import sys
from pathlib import Path

import cv2
import numpy as np


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser()

    for name in ["image.png", "image.jpg", "lesson07_resized.jpg"]:
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


def resize_for_display(image, width=480):
    """显示窗口太大时，按固定宽度等比例缩小，保存文件不受影响。"""
    height, old_width = image.shape[:2]
    if old_width <= width:
        return image
    new_height = int(height * width / old_width)
    return cv2.resize(image, (width, new_height))


image_path = get_image_path()
image = cv2.imread(str(image_path))

if image is None:
    print("image read failed:", image_path)
    raise SystemExit(1)

# OpenCV 默认颜色顺序是 BGR，不是 RGB。
b, g, r = cv2.split(image)
zeros = np.zeros_like(b)

# 灰度单通道图：数值越亮，说明该颜色通道越强。
blue_gray = b
green_gray = g
red_gray = r

# 彩色单通道图：只保留某一种颜色，其余通道置零。
blue_only = cv2.merge([b, zeros, zeros])
green_only = cv2.merge([zeros, g, zeros])
red_only = cv2.merge([zeros, zeros, r])

# 通道重组：按 B、G、R 顺序合并，可以恢复原图。
merged = cv2.merge([b, g, r])

# 通道置换：交换红色和蓝色通道，观察颜色变化。
swapped = cv2.merge([r, g, b])

outputs = {
    "lesson09_blue_gray.jpg": blue_gray,
    "lesson09_green_gray.jpg": green_gray,
    "lesson09_red_gray.jpg": red_gray,
    "lesson09_blue_only.jpg": blue_only,
    "lesson09_green_only.jpg": green_only,
    "lesson09_red_only.jpg": red_only,
    "lesson09_merged.jpg": merged,
    "lesson09_swapped.jpg": swapped,
}

for filename, result in outputs.items():
    save_path = image_dir / filename
    cv2.imwrite(str(save_path), result)
    print("saved:", save_path)

print("source image:", image_path)
print("source shape:", image.shape)
print("B channel shape:", b.shape)
print("G channel shape:", g.shape)
print("R channel shape:", r.shape)
print("press q or Esc to close windows")

windows = {
    "original": image,
    "blue_only": blue_only,
    "green_only": green_only,
    "red_only": red_only,
    "merged": merged,
    "swapped": swapped,
}

for title, picture in windows.items():
    cv2.namedWindow(title, cv2.WINDOW_NORMAL)
    cv2.imshow(title, resize_for_display(picture))

while True:
    key = cv2.waitKey(100) & 0xFF
    if key == ord("q") or key == 27:
        break

cv2.destroyAllWindows()
