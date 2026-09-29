import sys
from pathlib import Path

import cv2


# 本课程图片统一放在 vision_course/images 目录。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser()

    preferred = image_dir / "images.jpg"
    if preferred.exists():
        return preferred

    image_files = sorted(
        list(image_dir.glob("*.jpg"))
        + list(image_dir.glob("*.jpeg"))
        + list(image_dir.glob("*.png"))
    )
    if image_files:
        return image_files[0]

    print("no image found in:", image_dir)
    print("please put a jpg or png image into vision_course/images")
    raise SystemExit(1)


image_path = get_image_path()
image = cv2.imread(str(image_path))

if image is None:
    print("image read failed:", image_path)
    raise SystemExit(1)

height, width = image.shape[:2]
channels = 1 if len(image.shape) == 2 else image.shape[2]
center_pixel = image[height // 2, width // 2]

print("image path:", image_path)
print("shape:", image.shape)
print("width:", width)
print("height:", height)
print("channels:", channels)
print("dtype:", image.dtype)
print("center pixel BGR:", center_pixel)

cv2.imshow("lesson06_image", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
