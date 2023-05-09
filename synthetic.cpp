#include "pcl_simu.h"

std::function<float(float)> get_func(float y){
    auto func = [y](float x){
        return -(2.0 + sin(y) / 2) * (x - 0.5) * (x - 2.5);
    };
};


void synthetic_pcd()
{

};

void synthetic_noise()
{

};

