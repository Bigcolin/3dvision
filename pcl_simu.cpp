#include "pcl_simu.h"

namespace pcl_simu
{

float curve(float x, float y){
        float z = -(2.5 + sin(2 * M_PI * y) / 40) * (x - 0.5) * (x - 2.5);
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

camera::camera(vec3f distort, float noi, float ang_z, float ang_y){
    distort_coefs = distort;
    noise_coef = noi;
    angle_z = ang_z;
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
    auto top_sight_z = sight(angle_z, coord_z);
    // auto top_sight_y = sight(angle_y, height);
    auto bot_sight_z = sight(-angle_z, coord_z);
    // auto bot_sight_y = sight(-angle_y, height);
    float dist;
    for (float v = 0.0f; v < 1.2f; v+=density)
    {
        if (std::abs(v - 0.6) < 0.3)
            dist = 0.8f; 
        else
            dist = 0.3f;

        for (float x = dist; x < 1.5f; x+=density)
        {   

            float y = coord_y + v - 0.6;

            float z = curve(x, y);
            // cout << "(x, y, z): " << u <<", " << v << ", " << z << endl;
            // cout << z << " : " << distortion(z) << endl;
            if (z < std::min(top_sight_z(x), ZMAX) && z > std::max(bot_sight_z(x), 0.0f))
            {
                pcl::PointXYZ p(x, coord_y + distortion(v - 0.6, (v - 0.6)/x), coord_z + distortion(z - coord_z, 0.0*(z - coord_z)/x));
                cloud->points.push_back(p);
            }
        }
    }
};



float camera::distortion(float d, float k)
{
    vec3f vd(pow(d, 2), pow(d, 4), pow(d, 6));
    return d*(1.0 + k * vd.transpose()*distort_coefs);  // 1 + k1 r^2 + k2 r^4 + k3 r^ : Distortion
};

void global_pcd(pxyz& cloud, camera& cam)
{

};

}