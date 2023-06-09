import numpy as np
import cv2

# 准备标定板
rows, cols = 11, 8
square_size = 10  # 棋盘格正方形边长，单位毫米
objp = np.zeros((rows*cols, 3), np.float32)
objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size

# 拍摄标定图像
fpath = "./camera_calibrate/"
f = open(fpath + "filename.txt")

img_paths = [line.strip() for line in f.readlines()]  # 标定图像路径列表
img_points = []    # 角点坐标列表
obj_points = []    # 标定板坐标列表
flags = None # cv2.CALIB_CB_FAST_CHECK
for img_path in img_paths:
    print("processing ", img_path)
    img = cv2.imread(fpath + img_path)
    # print(img.shape)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(gray.shape)
    ret, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    print(ret, " ", corners)
    if ret:
        img_points.append(corners)
        obj_points.append(objp)
f.close()
# 标定相机
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)
print("内参矩阵：")
print(mtx)
print("畸变系数：")
print(dist)
