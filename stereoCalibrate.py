# 类三：相机标定执行函数(双目校正)
import cv2
import numpy as np
import glob
import os

def GetRotationMatrix(theta_x, theta_y, theta_z):
	sx = np.sin(theta_x)
	cx = np.cos(theta_x)
	sy = np.sin(theta_y)
	cy = np.cos(theta_y)
	sz = np.sin(theta_z)
	cz = np.cos(theta_z)
	return np.array([
              [cy*cz, cz*sx*sy-cx*sz, sx*sz+cx*cz*sy],
              [cy*sz, cx*cz+sx*sy*sz, cx*sy*sz-cz*sz],
              [-sy, cy*sx, cx*cy]])

def GetRotationAngles(rot_mat):
	theta_x = np.arctan2(rot_mat[2,1], rot_mat[2,2])
	theta_y = np.arctan2(-rot_mat[2,0], \
          np.sqrt(rot_mat[2,1]*rot_mat[2,1]+rot_mat[2,2]*rot_mat[2,2]))
	theta_z = np.arctan2(rot_mat[1,0], rot_mat[0,0])
	return theta_x, theta_y, theta_z


class Stereo_calibrate():  # 执行校正
    def __init__(self):
        # 终止条件
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        # 准备对象点，棋盘方块交界点排列：6行8列 如 (0,0,0), (1,0,0), (2,0,0) ....,(6,8,0)
        self.row,self.col = 8,11
        self.objpoints = np.zeros((self.row * self.col, 3), np.float32)
        self.square_size = 10 # mm
        self.objpoints[:, :2] = np.mgrid[0:self.row, 0:self.col].T.reshape(-1, 2) * self.square_size

    def split(self, dir):

        img = glob.glob('%s/*'%dir)
        dir_l = dir + '_left/'
        dir_r = dir + '_right/'
        try:
            os.makedirs(dir_l)
        except:
            print("File dir_left exist")
        try:
            os.makedirs(dir_r)
        except:
            print("File dir_right exist")
            
        for i in range(len(img)):
            img_i = cv2.imread(img[i])
            print('loading ', img[i])
            # gray_i = cv2.cvtColor(img_i, cv2.COLOR_BGR2GRAY)
            h, w = img_i.shape[:2]
            gray_l = img_i[:, :int(w/2), :]
            gray_r = img_i[:, int(w/2):, :]
            cv2.imwrite(dir_l  + f'{i}_left.png', gray_l)
            cv2.imwrite(dir_r + f'{i}_right.png', gray_r)
        return dir_l, dir_r


    def exe(self,dir_l,dir_r = None):
        objectpoints = [] # 真实世界中的3d点
        imgpoints_l = []
        imgpoints_r = []
        # 标定所用图像
        images_l = glob.glob('%s/*'%dir_l)
        if dir_r == None:
             nimg = int(len(images_l)/2)
             dir_r = dir_l
        # images_r = glob.glob('%s/*' % dir_r)
        for i in range(nimg):
            img_l = cv2.imread(dir_l + f'{i}_left.png')
            print('loading ', dir_l, f'{i}_left.png')
            gray_l = cv2.cvtColor(img_l, cv2.COLOR_BGR2GRAY)
            
            img_r = cv2.imread(dir_r + f'{i}_right.png')
            print('loading ', dir_l, f'{i}_right.png \n')
            gray_r = cv2.cvtColor(img_r, cv2.COLOR_BGR2GRAY)
            # 寻找到棋盘角点
            ret1, corners_l = cv2.findChessboardCorners(img_l, (self.row, self.col), None)
            ret2, corners_r = cv2.findChessboardCorners(img_r, (self.row, self.col), None)
            # 如果找到，添加对象点，图像点（细化之后）
            if ret1 == True and ret2 == True:
                # 添加每幅图的对应3D-2D坐标
                objectpoints.append(self.objpoints)
                corners_l = cv2.cornerSubPix(gray_l, corners_l, (11, 11), (-1, -1),self.criteria)
                imgpoints_l.append(corners_l)
                corners_r = cv2.cornerSubPix(gray_r, corners_r, (11, 11), (-1, -1), self.criteria)
                imgpoints_r.append(corners_r)
            #     # 绘制并显示拐角
            #     cv2.drawChessboardCorners(img_l, (self.row, self.col), corners_l, ret1)
            #     cv2.drawChessboardCorners(img_r, (self.row, self.col), corners_r, ret2)
            #     view  = np.concatenate((img_l, img_r), axis=1)
            #     cv2.namedWindow('View')
            #     cv2.imshow("View", view)
            #     cv2.waitKey(0)
        # cv2.destroyAllWindows()
        # 利用单目校正函数实现相机内参初始化
        ret, m1, d1, _, _ = cv2.calibrateCamera(objectpoints, imgpoints_l, gray_l.shape[::-1], None, None)
        ret, m2, d2, _, _= cv2.calibrateCamera(objectpoints, imgpoints_r, gray_r.shape[::-1], None, None)
        # config
        flags = 0
        # flags |= cv2.CALIB_FIX_ASPECT_RATIO
        flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        # flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # flags |= cv2.CALIB_ZERO_TANGENT_DIST
        flags |= cv2.CALIB_RATIONAL_MODEL
        # flags |= cv2.CALIB_FIX_K1
        # flags |= cv2.CALIB_FIX_K2
        # flags |= cv2.CALIB_FIX_K3
        # flags |= cv2.CALIB_FIX_K4
        # flags |= cv2.CALIB_FIX_K5
        # flags |= cv2.CALIB_FIX_K6
        stereocalib_criteria = (cv2.TERM_CRITERIA_COUNT +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        # 输入参数：真实3d坐标点，左相机像素点、右相机像素点、左内参、左畸变、右内参、右畸变、图像尺寸、一些配置
        # 输出值：未知、左内参、左畸变、右内参、右畸变（迭代优化后的）、旋转矩阵、平移向量、本质矩阵、基础矩阵
        ret, m1, d1,m2, d2, R, t, E, F = cv2.stereoCalibrate(objectpoints,imgpoints_l,imgpoints_r,
                                                                       m1, d1,m2, d2, gray_l.shape[::-1],
                                                                       criteria=stereocalib_criteria, flags=flags)
        # 构建单应性矩阵
        plane_depth = 40000000.0  # arbitrary plane depth
        # TODO: Need to understand effect of plane_depth. Why does this improve some boards' cals?
        n = np.array([[0.0], [0.0], [-1.0]])
        d_inv = 1.0 / plane_depth
        H = (R - d_inv * np.dot(t, n.transpose()))
        H = np.dot(m2, np.dot(H, np.linalg.inv(m1)))
        H /= H[2, 2]
        # rectify Homography for right camera
        disparity = (m1[0, 0] * t[0] / plane_depth)
        H[0, 2] -= disparity
        H = H.astype(np.float32)
        print(ret,'\n左相机矩阵：%s\n左相机畸变:%s\n右相机矩阵：%s\n右相机畸变:%s\n旋转矩阵:%s\n平移向量:%s'
                  '\n本质矩阵E:%s\n基础矩阵F:%s\n单应性矩阵H:%s'
              %(m1, d1,m2, d2, R, t, E, F,H))
        return m1, d1, m2, d2, R, t, E, F, H


if __name__ =="__main__":
    cal = Stereo_calibrate()
    # dir_l, dir_r = cal.split('./camera_calibrate')
    dir_l = "./camera_calibrate/"
    m1, d1, m2, d2, R, t, E, F, H = cal.exe(dir_l)