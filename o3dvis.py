import open3d as o3d
import numpy as np

fn = '/Users/kyan/Projects/3dpcl/pcl0/synthetic_data/test_transform'
ft = '.pcd'
cloud = []

robort_para = open("./robort_coords.txt")
n = robort_para.readline()
n = int(n) * 9
print(f"visulizing {n} pointclouds...")
# for k in range(n):
    # cloud.append(o3d.io.read_point_cloud(fn + f'{k}' + ft))
# o3d.visualization.draw_geometries(cloud)
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
vis = o3d.visualization.Visualizer()
vis.create_window()

for k in range(n):
    vis.add_geometry(o3d.io.read_point_cloud(fn + f'{k}' + ft))
vis.add_geometry(frame)

vis.run()
