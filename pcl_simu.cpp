#include "pcl_simu.h"

namespace pcl_simu
{

float curve(float x, float y){
        float z = -(2.0 + sin(2 * M_PI * y) / 40) * (x - 0.5) * (x - 2.5);
    return z;
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

camera::camera(float distort, float noi, float ang_x, float ang_y){
    distort_coef = distort;
    noise_coef = noi;
    angle_x = ang_x;
    angle_y = ang_y;
    coord_x = 0.0f;
};

void camera::move_to(float& y, float& z)
{
    coord_z = z;
    coord_y = y;

};

void camera::simu_shot(pxyz& cloud)
{
    float density = DENSITY * 5;
    auto top_sight_x = sight(angle_x, coord_z);
    // auto top_sight_y = sight(angle_y, height);
    auto bot_sight_x = sight(-angle_x, coord_z);
    // auto bot_sight_y = sight(-angle_y, height);
    for (float y = 0.0f; y < 1.2f; y+=density)
    for (float x = 0.0f; x < 1.2f; x+=density)
    {   

        float v = coord_y + y - 0.6;
        // cout << v << " : " << distortion(v) << endl;
        float u = x + 0.3; // depth, geometry info(v, z). to calcu
        float z = curve(u, v);
        // cout << "(x, y, z): " << u <<", " << v << ", " << z << endl;
        // cout << z << " : " << distortion(z) << endl;
        if (z < top_sight_x(u) && z > bot_sight_x(u))
        {
            pcl::PointXYZ p(u, coord_y + distortion(y - 0.6), coord_z + distortion(z - coord_z));
            cloud->points.push_back(p);
        }
    }
};



float camera::distortion(float d)
{
    return d*(1.0 + d*d*distort_coef);  // 1 + k1 r^2 + k2 r^4 + k3 r^ : Distortion
};

void global_pcd(pxyz& cloud, camera& cam)
{

};

}