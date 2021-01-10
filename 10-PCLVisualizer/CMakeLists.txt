cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_visualizer_viewports)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Debug")


find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_visualizer_demo src/pcl_visualizer_demo.cpp)
target_link_libraries (pcl_visualizer_demo ${PCL_LIBRARIES})

# pcl::PointNormal
add_executable(vis_normal_cloud src/vis_normal_cloud.cpp)
target_link_libraries(vis_normal_cloud
    ${PCL_LIBRARIES}
    stdc++fs # <experimental/filesystem>
)

# vis normal 2d
add_executable(vis_normal_2d src/vis_normal_2d.cpp)
target_link_libraries(vis_normal_2d
    ${PCL_LIBRARIES}
    stdc++fs # <experimental/filesystem>
)
