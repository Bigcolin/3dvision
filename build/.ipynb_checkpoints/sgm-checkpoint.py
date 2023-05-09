import cv2
import numpy as np

# Load left and right images
img_left = cv2.imread('./chess1l.png')
img_right = cv2.imread('./chess1r.png')


left_gray = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
right_gray = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)


# Define SGM algorithm parameters
window_size = 5
min_disp = 1
num_disp = 16
block_size = window_size * window_size
disp12MaxDiff = -1
uniquenessRatio = 5
speckleWindowSize = 50
speckleRange = 2

# Create SGM object
sgbm = cv2.StereoSGBM_create(
    minDisparity=min_disp,
    numDisparities=num_disp,
    blockSize=block_size,
    P1=3 * block_size * window_size ** 2,
    P2=90 * block_size * window_size ** 2,
    disp12MaxDiff=disp12MaxDiff,
    uniquenessRatio=uniquenessRatio,
    speckleWindowSize=speckleWindowSize,
    speckleRange=speckleRange
)

# Compute disparity map
# disp = sgbm.compute(img_left, img_right)
disp = sgbm.compute(left_gray, right_gray)

# disp = sgbm.compute(img_right, img_left)

# Generate point cloud
focal_length = 1758.23 # focal length in pixels
baseline = 11.153e-2 # baseline in meters
cx, cy = 953.34, 552.29
Q = np.float32([[1, 0, 0, -img_left.shape[1]/2],
                [0, 1, 0, -img_left.shape[0]/2],
                [0, 0, 0, focal_length],
                [0, 0, -1/baseline, 0]])
points = cv2.reprojectImageTo3D(disp, Q)
colors = cv2.cvtColor(left_gray, cv2.COLOR_GRAY2BGR)

# Filter out points with invalid depth
mask = disp > disp.min()
points = points[mask]
colors = colors[mask]

# Save point cloud
np.savetxt('points.txt', points)
np.savetxt('colors.txt', colors)

# Display disparity map
cv2.imshow('Disparity Map', (disp - min_disp) / num_disp)
cv2.waitKey(0)
cv2.destroyAllWindows()

