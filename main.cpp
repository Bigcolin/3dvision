#include "pcl_simu.h"
#include <fstream>

int main(int argc, char** argv)
{   
    if (argc < 1)
    {
        cout << "error: at least one scan position is required" << endl;
    }
    vecxf scan_position(argc - 1);
    // scan_position.resize(argc - 1);
    for(int i = 1; i < argc; ++i)
    {
        scan_position(i - 1) = atoi(argv[i]);
    }
    vecxf cam_position(2);
    cam_position(0) = 0.5f;
    cam_position(1) = 1.5f;
    // cam_position(2) = 1.5f;

    float scan_x = 0.0f;
    float z_angle = M_PI/3;
    float y_angle = M_PI/3;
    vec3f cam_distort(0.16f, 0.04f, 0.01f);
    float cam_noise = 0.0f;
    vec3f r0(0.4f, 0.8f, 1.2f);
    auto sph0 = pcl_simu::calib_sphere(r0);    
    
    pxyz global_cloud(new xyz);
    pxyz local_cloud(new xyz);
    pcl_simu::camera cam0(cam_distort, cam_noise, z_angle, y_angle);
    for (float &y : scan_position)
    {   

        for (float &h : cam_position)
        {
            // local_cloud.reset();
            cam0.move_to(y, h);
            cam0.simu_shot(local_cloud);

        }

    };
    cout << "the size of pointcloud: " << local_cloud->size() << endl;
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    viewer->addPointCloud(local_cloud);
    viewer->addCoordinateSystem(2.0, "global");
    viewer->spin();

    return 0;
}
