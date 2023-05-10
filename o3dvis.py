import open3d as o3d
import numpy as np

fn = '/Users/kyan/Projects/3dpcl/pcl0/synthetic_data/test_transform'
ft = '.pcd'
cloud = []
n = 9
for k in range(n):
    cloud.append(o3d.io.read_point_cloud(fn + f'{k}' + ft))

o3d.visualization.draw_geometries(cloud)