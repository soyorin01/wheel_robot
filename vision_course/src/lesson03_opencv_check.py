import cv2
import numpy as np
from pathlib import Path

# 获取项目目录 vision_course，并在项目目录下创建 images 目录。
project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
image_dir.mkdir(exist_ok=True)

# 创建一张 240x320 的黑色图片。
# OpenCV 中的图片本质上是 NumPy 数组，形状为：高度、宽度、颜色通道数。
# 这里的 3 表示 BGR 三个颜色通道，每个像素值范围是 0~255。
img = np.zeros((240, 320, 3), dtype=np.uint8)

# 在图片上画一个矩形。
# (20, 20) 是左上角坐标，(300, 220) 是右下角坐标。
# (255, 0, 0) 是颜色，OpenCV 默认使用 BGR 顺序，所以这里表示蓝色。
# 最后的 3 表示矩形边框的线条粗细。
cv2.rectangle(img, (20, 20), (300, 220), (255, 0, 0), 3)

# 在图片上写文字。
# (55, 125) 是文字起始位置，0.8 是字体大小，(0, 255, 0) 表示绿色。
cv2.putText(img, "hello", (55, 125),
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

ok = cv2.imwrite(str(image_dir / "lesson03_opencv_test.jpg"), img)

# 打印测试信息，用来确认程序和 OpenCV 是否正常工作。
print("lesson03_ok")
print("opencv:", cv2.__version__)
print("saved:", ok)

# 弹出窗口显示图片。
cv2.imshow("lesson03", img)

# 等待键盘输入。参数 0 表示一直等待，直到按下任意键。
cv2.waitKey(0)

# 关闭所有 OpenCV 创建的窗口。
cv2.destroyAllWindows()
