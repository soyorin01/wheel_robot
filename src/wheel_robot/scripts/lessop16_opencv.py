#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lesson 16 - OpenCV Detection V3
--------------------------------
1 : 红/蓝颜色识别
2 : 二维码识别
3 : 人体检测（OpenCV DNN + MobileNet-SSD）
Q : 退出

依赖：
    pip install opencv-python numpy

说明：
    人体检测已取消 HOG，改用 OpenCV DNN。
    第一次切换到人体检测时，如果 models/ 下没有模型，
    程序会自动从 GitHub 下载 MobileNet-SSD 模型（约 22 MB）。
"""

import os
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node


# ============================================================
# 基本参数
# ============================================================

CAMERA_ID = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# ---------- 颜色识别 ----------
# 夜间/室内暖光下颜色会变暗、饱和度下降，默认阈值放宽一点。
RED1_LOWER = np.array([0, 70, 45])
RED1_UPPER = np.array([15, 255, 255])

RED2_LOWER = np.array([165, 70, 45])
RED2_UPPER = np.array([180, 255, 255])

BLUE_LOWER = np.array([88, 50, 40])
BLUE_UPPER = np.array([135, 255, 255])

COLOR_MIN_AREA = 800
COLOR_KERNEL_SIZE = 5

# ---------- DNN人体检测 ----------
# 越高越严格；推荐 0.45 ~ 0.65
PERSON_CONFIDENCE = 0.50

# 重叠框抑制阈值
PERSON_NMS_THRESHOLD = 0.35

# Orange Pi 为降低CPU负载，每隔几帧做一次神经网络推理
# 1 = 每帧检测，最灵敏但CPU占用高
# 2 = 推荐
# 3 = 更省性能
DNN_DETECT_INTERVAL = 2

# VOC 数据集中 person 的类别编号
PERSON_CLASS_ID = 15

BASE_DIR = Path(__file__).resolve().parent
MODEL_DIR = BASE_DIR / "models"

PROTOTXT_PATH = MODEL_DIR / "deploy.prototxt"
CAFFEMODEL_PATH = MODEL_DIR / "mobilenet_iter_73000.caffemodel"

PROTOTXT_URLS = [
    "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/deploy.prototxt",
    "https://github.com/chuanqi305/MobileNet-SSD/raw/refs/heads/master/deploy.prototxt",
]

CAFFEMODEL_URLS = [
    "https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/mobilenet_iter_73000.caffemodel",
    "https://github.com/chuanqi305/MobileNet-SSD/raw/refs/heads/master/mobilenet_iter_73000.caffemodel",
]


# ============================================================
# 通用函数
# ============================================================

def put_text(frame, text, pos, color=(0, 255, 0), scale=0.65, thickness=2):
    cv2.putText(
        frame,
        text,
        pos,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thickness,
        cv2.LINE_AA
    )


def download_file(urls, target_path):
    """
    从多个备用地址尝试下载。
    下载完成前使用 .part 临时文件，避免中断后留下损坏模型。
    """
    target_path = Path(target_path)
    temp_path = target_path.with_suffix(target_path.suffix + ".part")

    for url in urls:
        try:
            print("Downloading:")
            print(url)

            request = urllib.request.Request(
                url,
                headers={"User-Agent": "Mozilla/5.0"}
            )

            with urllib.request.urlopen(request, timeout=60) as response:
                total = response.headers.get("Content-Length")
                total = int(total) if total else 0

                downloaded = 0
                last_percent = -1

                with open(temp_path, "wb") as f:
                    while True:
                        chunk = response.read(1024 * 256)

                        if not chunk:
                            break

                        f.write(chunk)
                        downloaded += len(chunk)

                        if total > 0:
                            percent = int(downloaded * 100 / total)

                            if percent // 10 != last_percent // 10:
                                print("  %d%%" % percent)
                                last_percent = percent

            temp_path.replace(target_path)
            print("Saved:", target_path)
            return True

        except Exception as e:
            print("Download failed:", e)

            if temp_path.exists():
                try:
                    temp_path.unlink()
                except Exception:
                    pass

    return False


def ensure_dnn_model():
    MODEL_DIR.mkdir(parents=True, exist_ok=True)

    if not PROTOTXT_PATH.exists():
        print("\nMissing:", PROTOTXT_PATH)

        if not download_file(PROTOTXT_URLS, PROTOTXT_PATH):
            return False

    # 模型正常大小约22MB。
    # 小于1MB基本可以判断为下载到了错误页面或损坏文件。
    if (not CAFFEMODEL_PATH.exists()
            or CAFFEMODEL_PATH.stat().st_size < 1024 * 1024):

        if CAFFEMODEL_PATH.exists():
            try:
                CAFFEMODEL_PATH.unlink()
            except Exception:
                pass

        print("\nMissing:", CAFFEMODEL_PATH)
        print("First use needs to download MobileNet-SSD model (~22 MB).")

        if not download_file(CAFFEMODEL_URLS, CAFFEMODEL_PATH):
            return False

    return True


# ============================================================
# 1. 红蓝颜色识别
# ============================================================

def draw_color_contours(frame, mask, name, color):
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < COLOR_MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        cv2.rectangle(
            frame,
            (x, y),
            (x + w, y + h),
            color,
            2
        )

        put_text(
            frame,
            "%s  area:%d" % (name, int(area)),
            (x, max(y - 10, 20)),
            color,
            0.55
        )


def color_detect(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 红色在HSV色相环两端，需要两个区间
    red_mask1 = cv2.inRange(hsv, RED1_LOWER, RED1_UPPER)
    red_mask2 = cv2.inRange(hsv, RED2_LOWER, RED2_UPPER)
    red_mask = cv2.bitwise_or(red_mask1, red_mask2)

    blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)

    # 形态学开闭运算，减少小噪点和小孔洞
    kernel = np.ones((COLOR_KERNEL_SIZE, COLOR_KERNEL_SIZE), np.uint8)

    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_OPEN, kernel)
    blue_mask = cv2.morphologyEx(blue_mask, cv2.MORPH_CLOSE, kernel)

    draw_color_contours(frame, red_mask, "RED", (0, 0, 255))
    draw_color_contours(frame, blue_mask, "BLUE", (255, 0, 0))

    put_text(frame, "MODE 1: RED / BLUE DETECT", (10, 30))
    return frame


# ============================================================
# 2. 二维码识别
# ============================================================

qr_detector = cv2.QRCodeDetector()


def draw_qr(frame, data, points):
    points = np.array(points, dtype=np.int32).reshape(-1, 2)

    if len(points) < 4:
        return

    for i in range(4):
        p1 = tuple(points[i])
        p2 = tuple(points[(i + 1) % 4])
        cv2.line(frame, p1, p2, (0, 255, 0), 3)

    if data:
        print("QR:", data)

        # putText 对中文支持不好，窗口里截短显示；
        # 完整二维码内容会输出到终端。
        text = data

        if len(text) > 32:
            text = text[:32] + "..."

        x, y = points[0]

        try:
            text.encode("ascii")
            show_text = "QR: " + text
        except UnicodeEncodeError:
            show_text = "QR decoded - see terminal"

        put_text(
            frame,
            show_text,
            (int(x), max(int(y) - 12, 25)),
            (0, 255, 0),
            0.55
        )


def qr_detect(frame):
    detected = False

    # 新版OpenCV支持多二维码
    if hasattr(qr_detector, "detectAndDecodeMulti"):
        try:
            retval, decoded_info, points, _ = \
                qr_detector.detectAndDecodeMulti(frame)

            if retval and points is not None:
                for data, pts in zip(decoded_info, points):
                    draw_qr(frame, data, pts)

                detected = True

        except Exception:
            detected = False

    # 单二维码兼容模式
    if not detected:
        data, points, _ = qr_detector.detectAndDecode(frame)

        if points is not None:
            draw_qr(frame, data, points)

    put_text(
        frame,
        "MODE 2: QR CODE DETECT",
        (10, 30),
        (0, 255, 255)
    )

    return frame


# ============================================================
# 3. OpenCV DNN 人体检测
# ============================================================

class PersonDetector:
    def __init__(self):
        self.net = None
        self.ready = False
        self.frame_count = 0
        self.last_boxes = []
        self.last_scores = []
        self.error_message = ""

    def load(self):
        if self.ready:
            return True

        if not ensure_dnn_model():
            self.error_message = "DNN model download failed"
            print("\nERROR: MobileNet-SSD model unavailable.")
            print("Check network or manually put files into:")
            print(MODEL_DIR)
            return False

        try:
            print("\nLoading OpenCV DNN model...")

            self.net = cv2.dnn.readNetFromCaffe(
                str(PROTOTXT_PATH),
                str(CAFFEMODEL_PATH)
            )

            # Orange Pi / ARM 默认CPU最稳妥
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

            self.ready = True
            self.error_message = ""
            print("DNN model loaded.")
            return True

        except Exception as e:
            self.error_message = "DNN load failed"
            print("DNN load failed:", e)
            return False

    def infer(self, frame):
        """
        MobileNet-SSD 输入：
        300x300
        mean = 127.5
        scale = 0.007843
        """
        h, w = frame.shape[:2]

        blob = cv2.dnn.blobFromImage(
            frame,
            scalefactor=0.007843,
            size=(300, 300),
            mean=(127.5, 127.5, 127.5),
            swapRB=False,
            crop=False
        )

        self.net.setInput(blob)
        detections = self.net.forward()

        boxes = []
        scores = []

        for i in range(detections.shape[2]):
            class_id = int(detections[0, 0, i, 1])
            confidence = float(detections[0, 0, i, 2])

            # 只保留 person
            if class_id != PERSON_CLASS_ID:
                continue

            if confidence < PERSON_CONFIDENCE:
                continue

            x1 = int(detections[0, 0, i, 3] * w)
            y1 = int(detections[0, 0, i, 4] * h)
            x2 = int(detections[0, 0, i, 5] * w)
            y2 = int(detections[0, 0, i, 6] * h)

            # 防止坐标超出图像
            x1 = max(0, min(w - 1, x1))
            y1 = max(0, min(h - 1, y1))
            x2 = max(0, min(w - 1, x2))
            y2 = max(0, min(h - 1, y2))

            bw = x2 - x1
            bh = y2 - y1

            if bw <= 0 or bh <= 0:
                continue

            # 极小框意义不大
            if bw * bh < (w * h) * 0.005:
                continue

            boxes.append([x1, y1, bw, bh])
            scores.append(confidence)

        # NMS 去除重复人体框
        keep_boxes = []
        keep_scores = []

        if boxes:
            indices = cv2.dnn.NMSBoxes(
                boxes,
                scores,
                PERSON_CONFIDENCE,
                PERSON_NMS_THRESHOLD
            )

            if len(indices) > 0:
                indices = np.array(indices).reshape(-1)

                for idx in indices:
                    keep_boxes.append(boxes[int(idx)])
                    keep_scores.append(scores[int(idx)])

        return keep_boxes, keep_scores

    def detect(self, frame):
        if not self.load():
            put_text(
                frame,
                "DNN MODEL ERROR - SEE TERMINAL",
                (10, 65),
                (0, 0, 255),
                0.55
            )
            return frame

        self.frame_count += 1

        # 降低Orange Pi的CPU负载
        if (self.frame_count % DNN_DETECT_INTERVAL == 0
                or not self.last_boxes):

            try:
                boxes, scores = self.infer(frame)
                self.last_boxes = boxes
                self.last_scores = scores

            except Exception as e:
                print("DNN inference error:", e)

        for box, confidence in zip(
                self.last_boxes,
                self.last_scores):

            x, y, w, h = box

            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            put_text(
                frame,
                "Person %.2f" % confidence,
                (x, max(y - 10, 22)),
                (0, 255, 0),
                0.55
            )

        put_text(
            frame,
            "MODE 3: DNN HUMAN DETECT",
            (10, 30),
            (0, 255, 0)
        )

        put_text(
            frame,
            "People: %d" % len(self.last_boxes),
            (10, 60),
            (0, 255, 0)
        )

        return frame


person_detector = PersonDetector()


# ============================================================
# ROS2节点与主程序
# ============================================================

class Lesson16OpenCV(Node):
    """用ROS2参数配置的颜色、二维码和人体检测课堂节点。"""

    def __init__(self):
        global COLOR_MIN_AREA
        global COLOR_KERNEL_SIZE
        global RED1_LOWER, RED1_UPPER, RED2_LOWER, RED2_UPPER
        global BLUE_LOWER, BLUE_UPPER
        global PERSON_CONFIDENCE
        global PERSON_NMS_THRESHOLD
        global DNN_DETECT_INTERVAL
        super().__init__("lesson16_opencv_detector")
        self.declare_parameter(
            "camera_device",
            "/dev/v4l/by-id/usb-Astra_Pro_HD_Camera_Astra_Pro_HD_Camera-video-index0",
        )
        self.declare_parameter("frame_width", FRAME_WIDTH)
        self.declare_parameter("frame_height", FRAME_HEIGHT)
        self.declare_parameter("initial_mode", 1)
        self.declare_parameter("flip_horizontal", True)
        self.declare_parameter("color_min_area", COLOR_MIN_AREA)
        self.declare_parameter("color_kernel_size", COLOR_KERNEL_SIZE)
        self.declare_parameter("red1_lower", RED1_LOWER.tolist())
        self.declare_parameter("red1_upper", RED1_UPPER.tolist())
        self.declare_parameter("red2_lower", RED2_LOWER.tolist())
        self.declare_parameter("red2_upper", RED2_UPPER.tolist())
        self.declare_parameter("blue_lower", BLUE_LOWER.tolist())
        self.declare_parameter("blue_upper", BLUE_UPPER.tolist())
        self.declare_parameter("person_confidence", PERSON_CONFIDENCE)
        self.declare_parameter("person_nms_threshold", PERSON_NMS_THRESHOLD)
        self.declare_parameter("dnn_detect_interval", DNN_DETECT_INTERVAL)

        COLOR_MIN_AREA = max(1, int(self.get_parameter("color_min_area").value))
        kernel_size = max(1, int(self.get_parameter("color_kernel_size").value))
        COLOR_KERNEL_SIZE = kernel_size if kernel_size % 2 == 1 else kernel_size + 1
        RED1_LOWER = np.array(self.get_parameter("red1_lower").value, dtype=np.uint8)
        RED1_UPPER = np.array(self.get_parameter("red1_upper").value, dtype=np.uint8)
        RED2_LOWER = np.array(self.get_parameter("red2_lower").value, dtype=np.uint8)
        RED2_UPPER = np.array(self.get_parameter("red2_upper").value, dtype=np.uint8)
        BLUE_LOWER = np.array(self.get_parameter("blue_lower").value, dtype=np.uint8)
        BLUE_UPPER = np.array(self.get_parameter("blue_upper").value, dtype=np.uint8)
        PERSON_CONFIDENCE = float(
            np.clip(self.get_parameter("person_confidence").value, 0.05, 0.99)
        )
        PERSON_NMS_THRESHOLD = float(
            np.clip(self.get_parameter("person_nms_threshold").value, 0.05, 0.95)
        )
        DNN_DETECT_INTERVAL = max(
            1, int(self.get_parameter("dnn_detect_interval").value)
        )

        camera_value = str(self.get_parameter("camera_device").value)
        camera_source = int(camera_value) if camera_value.isdigit() else camera_value
        self.frame_width = max(160, int(self.get_parameter("frame_width").value))
        self.frame_height = max(120, int(self.get_parameter("frame_height").value))
        self.mode = int(np.clip(self.get_parameter("initial_mode").value, 1, 3))
        self.flip_horizontal = bool(self.get_parameter("flip_horizontal").value)
        self.window_name = "Lesson 16 - OpenCV Detection V3"
        self.cap = cv2.VideoCapture(camera_source)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        if not self.cap.isOpened():
            raise RuntimeError(f"摄像头打开失败: {camera_value}")

        self.fps = 0.0
        self.fps_count = 0
        self.fps_start = time.time()
        self.get_logger().info(
            f"摄像头={camera_value}，尺寸={self.frame_width}x{self.frame_height}，"
            f"初始模式={self.mode}，颜色最小面积={COLOR_MIN_AREA}，"
            f"人体置信度={PERSON_CONFIDENCE:.2f}，DNN间隔={DNN_DETECT_INTERVAL}"
        )
        print("1:红蓝颜色  2:二维码  3:人体检测  Q:退出", flush=True)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.0)
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("摄像头画面读取失败")
                break
            if self.flip_horizontal:
                frame = cv2.flip(frame, 1)

            if self.mode == 1:
                frame = color_detect(frame)
            elif self.mode == 2:
                frame = qr_detect(frame)
            else:
                frame = person_detector.detect(frame)

            self.fps_count += 1
            now = time.time()
            if now - self.fps_start >= 1.0:
                self.fps = self.fps_count / (now - self.fps_start)
                self.fps_count = 0
                self.fps_start = now

            put_text(
                frame, "FPS: %.1f" % self.fps,
                (frame.shape[1] - 105, 30), (0, 255, 255), 0.50
            )
            put_text(
                frame, "1:Color   2:QR   3:Human   Q:Quit",
                (10, frame.shape[0] - 18), (255, 255, 255), 0.50
            )
            cv2.imshow(self.window_name, frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("1"), ord("2"), ord("3")):
                self.mode = int(chr(key))
                self.get_logger().info(f"切换到模式 {self.mode}")
            elif key in (ord("q"), ord("Q"), 27):
                break

    def close(self):
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Lesson16OpenCV()
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"Lesson 16启动失败: {error}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
