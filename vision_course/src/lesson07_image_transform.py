import sys
from pathlib import Path

import cv2


# 本课程所有图片素材和处理结果都统一放在 vision_course/images。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"


def get_image_path():
    """优先读取命令行传入的图片，否则自动选择 images 目录中的一张图片。"""
    if len(sys.argv) > 1:
        return Path(sys.argv[1]).expanduser()

    for name in ["image.jpg", "lesson03_opencv_test.jpg"]:
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


image_path = get_image_path()
image = cv2.imread(str(image_path))

if image is None:
    print("image read failed:", image_path)
    raise SystemExit(1)

height, width = image.shape[:2]

# 1. 缩放：把图像缩放到固定尺寸。
resized = cv2.resize(image, (320, 240))

# 2. 裁剪：从图像中心裁剪出一块正方形区域。
crop_size = min(width, height) // 2
x1 = width // 2 - crop_size // 2
y1 = height // 2 - crop_size // 2
cropped = image[y1:y1 + crop_size, x1:x1 + crop_size]

# 3. 翻转：flipCode=1 表示水平翻转。
flipped = cv2.flip(image, 1)

# 4. 旋转：以图像中心为旋转中心，逆时针旋转 30 度。
center = (width // 2, height // 2)
matrix = cv2.getRotationMatrix2D(center, 30, 1.0)
rotated = cv2.warpAffine(image, matrix, (width, height))

outputs = {
    "lesson07_resized.jpg": resized,
    "lesson07_cropped.jpg": cropped,
    "lesson07_flipped.jpg": flipped,
    "lesson07_rotated.jpg": rotated,
}

for filename, result in outputs.items():
    save_path = image_dir / filename
    cv2.imwrite(str(save_path), result)
    print("saved:", save_path)

print("source image:", image_path)
print("source shape:", image.shape)
print("resized shape:", resized.shape)
print("cropped shape:", cropped.shape)
print("press q or Esc to close windows")

cv2.namedWindow("original", cv2.WINDOW_NORMAL)
cv2.namedWindow("resized", cv2.WINDOW_NORMAL)
cv2.namedWindow("cropped", cv2.WINDOW_NORMAL)
cv2.namedWindow("flipped", cv2.WINDOW_NORMAL)
cv2.namedWindow("rotated", cv2.WINDOW_NORMAL)
cv2.imshow("original", image)
cv2.imshow("resized", resized)
cv2.imshow("cropped", cropped)
cv2.imshow("flipped", flipped)
cv2.imshow("rotated", rotated)

# 用循环等待按键，窗口会一直保留，适合录课演示。
while True:
    key = cv2.waitKey(100) & 0xFF
    if key == ord("q") or key == 27:
        break

cv2.destroyAllWindows()
