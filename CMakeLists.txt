cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(MY_PCL_PROJECT)
find_package(PCL 1.3 REQUIRED COMPONENTS filters common io visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED opencv_stereo)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(pcd_test main.cpp pcl_simu.cpp)
add_executable(ndt_test ndt.cpp)
target_link_libraries(pcd_test ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES} ${OpenCV_LIBS} opencv_stereo)

