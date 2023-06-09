#include <iostream>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ pt;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pxyz;
typedef pcl::PointCloud<pcl::PointXYZ> xyz;

typedef Eigen::MatrixXf matxf;
typedef Eigen::VectorXf vecxf;
typedef Eigen::Vector3f vec3f;

using namespace std::chrono_literals;

