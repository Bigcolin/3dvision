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
#define ZMAX 2.55f
#define ZMIN 0.0f
#define MAX_ANGLE M_PI / 2
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

// float curve(float x, float y);
std::function<float(float)> sight(float ang, float height);
std::function<float(vec3f)> calib_sphere(vec3f cir);
pcl::PointXYZ sphere(float& r, float& theta, float& phi);

class camera{  // simulation of camera module

    vec3f distort_coefs; // 畸变与误差系数
    // float distort_coef;
    float noise_coef;
    // float angle_z; // 可视角度
    // float angle_y;
    vec3f coord;

    public:
        camera(vec3f distort, float noi);
        void simu_shot(pxyz& cloud, float&d, float& r);
        // float distortion(float d, float k);
};


class robortArm{
    float len_arm1, len_arm0, ang_arm1, ang_arm0;
    vec3f coord; // world_coordinate
    
    public:
        camera robcam;
        robortArm(float l1, float l2, vec3f init_coord, vec3f cam_distort, float cam_noi);
        void rotateTo(float& angle1, float& angle0); // 机器人各自由度旋转后的新角度
        void moveTo(vec3f new_coord); // 机器人位移
        void transform(pxyz& cloud_l, pxyz& cloud_g); // 手眼标定


};


void global_pcd(pxyz& cloud);
}
