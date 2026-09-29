from pathlib import Path

import cv2
import numpy as np


project_dir = Path(__file__).resolve().parent.parent
image_dir = project_dir / "images"
calib_dir = image_dir / "calibration"
model_dir = project_dir / "models"
model_dir.mkdir(exist_ok=True)

# 棋盘格内角点数量，不是格子数量。常见标定板可使用 9x6 内角点。
pattern_size = (9, 6)
square_size = 1.0


def find_calibration_images():
    image_files = sorted(
        list(calib_dir.glob("*.jpg"))
        + list(calib_dir.glob("*.jpeg"))
        + list(calib_dir.glob("*.png"))
    )
    if not image_files:
        print("no calibration images found:", calib_dir)
        print("please run lesson08_capture_chessboard.py first")
        raise SystemExit(1)
    return image_files


def build_object_points():
    """生成棋盘格在真实世界中的角点坐标，z 坐标为 0。"""
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size
    return objp


image_files = find_calibration_images()
object_points = []
image_points = []
objp = build_object_points()
image_size = None
preview_image = None

for image_path in image_files:
    image = cv2.imread(str(image_path))
    if image is None:
        print("skip unreadable image:", image_path)
        continue

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    image_size = gray.shape[::-1]

    found, corners = cv2.findChessboardCorners(gray, pattern_size)
    print(image_path.name, "corners:", found)

    if not found:
        continue

    # 亚像素优化可以让角点位置更准确。
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    object_points.append(objp)
    image_points.append(corners)

    preview_image = image.copy()
    cv2.drawChessboardCorners(preview_image, pattern_size, corners, found)

if len(object_points) < 3:
    print("valid chessboard images are not enough:", len(object_points))
    print("please capture at least 3 valid chessboard images")
    raise SystemExit(1)

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    object_points,
    image_points,
    image_size,
    None,
    None,
)

param_path = model_dir / "lesson08_camera_params.npz"
np.savez(
    param_path,
    camera_matrix=camera_matrix,
    dist_coeffs=dist_coeffs,
    reprojection_error=ret,
)

print("reprojection error:", ret)
print("camera matrix:")
print(camera_matrix)
print("dist coeffs:")
print(dist_coeffs.ravel())
print("saved params:", param_path)

source = preview_image if preview_image is not None else cv2.imread(str(image_files[0]))
undistorted = cv2.undistort(source, camera_matrix, dist_coeffs)

corner_path = image_dir / "lesson08_chessboard_corners.jpg"
undistort_path = image_dir / "lesson08_undistorted.jpg"
cv2.imwrite(str(corner_path), source)
cv2.imwrite(str(undistort_path), undistorted)

height, width = undistorted.shape[:2]

# 透视变换示例：把画面中的梯形区域转换为俯视矩形。
src_points = np.float32([
    [width * 0.30, height * 0.35],
    [width * 0.70, height * 0.35],
    [width * 0.92, height * 0.92],
    [width * 0.08, height * 0.92],
])
dst_points = np.float32([
    [0, 0],
    [width, 0],
    [width, height],
    [0, height],
])

perspective_matrix = cv2.getPerspectiveTransform(src_points, dst_points)
bird_view = cv2.warpPerspective(undistorted, perspective_matrix, (width, height))

marked = undistorted.copy()
for point in src_points.astype(int):
    cv2.circle(marked, tuple(point), 8, (0, 0, 255), -1)
cv2.polylines(marked, [src_points.astype(int)], True, (0, 255, 0), 2)

marked_path = image_dir / "lesson08_perspective_points.jpg"
bird_path = image_dir / "lesson08_bird_view.jpg"
cv2.imwrite(str(marked_path), marked)
cv2.imwrite(str(bird_path), bird_view)

print("saved:", corner_path)
print("saved:", undistort_path)
print("saved:", marked_path)
print("saved:", bird_path)
print("press q or Esc to close windows")

cv2.namedWindow("corners", cv2.WINDOW_NORMAL)
cv2.namedWindow("undistorted", cv2.WINDOW_NORMAL)
cv2.namedWindow("perspective_points", cv2.WINDOW_NORMAL)
cv2.namedWindow("bird_view", cv2.WINDOW_NORMAL)
cv2.imshow("corners", source)
cv2.imshow("undistorted", undistorted)
cv2.imshow("perspective_points", marked)
cv2.imshow("bird_view", bird_view)

while True:
    key = cv2.waitKey(100) & 0xFF
    if key == ord("q") or key == 27:
        break

cv2.destroyAllWindows()
