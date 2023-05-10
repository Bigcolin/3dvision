#include "pcl_simu.h"
#include <fstream>
#include <string>

int main(int argc, char** argv)
{   
    vec3f init_robot_position(0.0f, 0.0f, 0.0f); //default: origin
    // for(int i = 1; i < argc; ++i)
    // {
    //     init_robot_position(i - 1) = atoi(argv[i]);
    // }
    vecxf angle_arm1 = vecxf::LinSpaced(3, 0.2f, 0.8f) * 3 * M_PI * 2;
    // cout << angle_arm1 << endl;
    vecxf angle_arm0= vecxf::LinSpaced(3, 0.0f, 0.4f) * M_PI * 2;
    // cout << angle_arm0 << endl;
    float len_arm1 = 0.5f;
    float len_arm0 = 1.0f;
 
    // float z_angle = M_PI/4;
    // float y_angle = M_PI/4;
    vec3f cam_distort(0.16f, 0.04f, 0.01f);
    float cam_noise = 0.0f;
    pcl_simu::robortArm rob(len_arm1, len_arm0, init_robot_position, cam_distort, cam_noise);
    std::string fn = "/Users/kyan/Projects/3dpcl/pcl0/synthetic_data/test_transform";
    std::string ft = ".pcd";
    float r = 0.05;

    int ind = 0;
    int n_coord;
    std::ifstream inFile("/Users/kyan/Projects/3dpcl/pcl0/robort_coords.txt");
    inFile >> n_coord;
    cout<<"get pointcloud from " << n_coord << " position" << endl;
    // std::string line;

    while (n_coord > 0)
    {
        // getline(inFile, line);
        // std::istringstream iss(line);
        float x, y, z;
        inFile >> x >> y >> z;
        cout << "the coordinate of new position is: " << x << " " << y << " " << z << endl;
        vec3f new_coord(x, y, z);
        rob.moveTo(new_coord);
        for (float &theta : angle_arm1)
        {   

            for (float &phi : angle_arm0)
            {
                rob.rotateTo(theta, phi);
                pxyz local_cloud(new xyz);
                pxyz global_cloud(new xyz);
                rob.robcam.simu_shot(local_cloud, r);
                local_cloud->height = local_cloud->size();
                local_cloud->width = 1; 
                rob.transform(local_cloud, global_cloud);
                // cout << local_cloud->height <<", "<<local_cloud->width<<endl;
                pcl::io::savePCDFileASCII(fn + std::to_string(ind) + ft, *global_cloud);
                local_cloud.reset();
                global_cloud.reset();
                ind++;

            }

        };

        n_coord--;
    };
    inFile.close();

    // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer);
    // viewer->addPointCloud(local_cloud);
    // viewer->addCoordinateSystem(2.0, "global");
    // viewer->spin();

    return 0;
}
