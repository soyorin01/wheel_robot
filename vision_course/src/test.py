import cv2
import numpy as np

# 创建 500x500 黑色背景图
img = np.zeros((500, 500), dtype=np.uint8)

# 主体白色矩形
cv2.rectangle(img, (100, 100), (350, 350), 255, -1)
# 矩形外小白噪点
cv2.circle(img, (70, 70), 8, 255, -1)
cv2.circle(img, (420, 90), 6, 255, -1)
# 物体内部小黑孔洞
cv2.circle(img, (180, 160), 12, 0, -1)
# 细小突出毛刺
cv2.line(img, (350, 200), (390, 200), 255, 4)
# 临近两条缝隙白线
cv2.line(img, (120, 380), (220, 380), 255, 5)
cv2.line(img, (120, 410), (220, 410), 255, 5)

cv2.imwrite("morph_test.png", img)
cv2.imshow("原始测试图", img)
cv2.waitKey(0)
cv2.destroyAllWindows()