#include "pcl_simu.h"

namespace pcl_simu
{

// float curve(float x, float y){
//         float z = -(2.5 + sin(2 * M_PI * y) / 40) * (x - 0.5) * (x - 2.5);
//     return z;
// };

pcl::PointXYZ sphere(float& r, float& theta, float& phi)
{
    // float r = r + d;
    pcl::PointXYZ local_hs(r*cos(theta)*cos(phi),r*cos(theta)*sin(phi), r*sin(theta));
    return local_hs;
};

std::function<float(vec3f)> calib_sphere(vec3f cir){

    auto func = [cir](vec3f p){
        auto r = (p - cir).transpose()*(p - cir);
        return r;
    };
    return func;
};

std::function<float(float)> sight(float ang, float height){
    if (ang > MAX_ANGLE) 
        cout << "error: camera angle" << endl;
    if (height > ZMAX || height < ZMIN)
        cout << "error: camera height" << endl;
    auto func = [ang, height](float x){
        return x * tan(ang) + height;
    };
    return func;
};

camera::camera(vec3f distort, float noi){
    distort_coefs = distort;
    noise_coef = noi;
 };


void camera::simu_shot(pxyz& cloud, float& r)
{
    float density = DENSITY * 5;
    for (float theta = -M_PI_2; theta < M_PI_2; theta+=density)
        for (float phi = -M_PI_2; phi < M_PI_2; phi+=density)
        {   

            pcl::PointXYZ p = sphere(r, theta, phi);
            cloud->push_back(p);

        }
};

// float camera::distortion(float d, float k)
// {
//     vec3f vd(pow(d, 2), pow(d, 4), pow(d, 6));
//     return d*(1.0 + k * vd.transpose()*distort_coefs);  // 1 + k1 r^2 + k2 r^4 + k3 r^ : Distortion
// };

robortArm::robortArm(float l1, float l0, vec3f init_coord, vec3f distort, float noi) : robcam(distort, noi)
{
    len_arm1 = l1;
    len_arm0 = l0;
    coord = init_coord;

};

void robortArm::rotateTo(float& ang1, float& ang2)
{
    ang_arm1 = ang1;
    ang_arm0 = ang2;
};

void robortArm::telescpTo(float len1, float len2)
{
    len_arm0 = len1;
    len_arm1 = len2;
}

void robortArm::moveTo(vec3f new_coord)
{
    coord = new_coord;

};


void robortArm::transform(pxyz& cloud_l, pxyz& cloud_g)
{
    Eigen::Affine3f tf = Eigen::Affine3f::Identity();
    vec3f t(len_arm1*cos(ang_arm1)*cos(ang_arm0), len_arm1*cos(ang_arm1)*sin(ang_arm0), len_arm1*sin(ang_arm1) + len_arm0);
    t = t + coord;
 
    tf.rotate(Eigen::AngleAxisf(ang_arm0, Eigen::Vector3f::UnitZ()));
    tf.rotate(Eigen::AngleAxisf(-ang_arm1, Eigen::Vector3f::UnitY()));
    tf.translation() << t;
    pcl::transformPointCloud(*cloud_l, *cloud_g, tf);

};


}