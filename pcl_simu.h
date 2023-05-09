#include <iostream>
#include <string>
#include <functional>
// #include <thread>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
// #include<pcl/filters/passthrough.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/common/concatenate.h>
#include<pcl/common/transforms.h>

#define YMAX 10.0f
#define YMIN 0.0f
#define XMAX 1.5f
#define XMIN 0.0f
#define ZMAX 3.0f
#define ZMIN 0.0f
#define MAX_ANGLE M_PI / 3
#define RADIUS 0.05f
#define DENSITY 0.01f
#define LOCAL_PCD_SIZE 2000
#define GLOBAL_PCD_SIZE 20000

typedef pcl::PointXYZ pt;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pxyz;
typedef pcl::PointCloud<pcl::PointXYZ> xyz;

typedef Eigen::MatrixXf matxf;
typedef Eigen::VectorXf vecxf;
typedef Eigen::Vector3f vec3f;

namespace pcl_simu{

float curve(float x, float y);
std::function<float(float)> sight(float ang, float height);
std::function<float(vec3f)> calib_sphere(vec3f cir);

class camera{

    float distort_coef;
    float noise_coef;

    float angle_x;
    float angle_y;

    float coord_x;
    float coord_y;
    float coord_z;
    // pxyz cloud; 

    public:
        camera(float distort, float noi, float ang_x, float ang_y);
        void move_to(float& y, float& z);
        void simu_shot(pxyz& cloud);
        float distortion(float d);
};


// void global_pcd(pxyz& cloud, camera& cam);
}
