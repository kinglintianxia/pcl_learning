cmake_minimum_required(VERSION 2.8)

project(pcd_read)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcd_read src/pcd_read.cpp)
target_link_libraries(pcd_read ${PCL_LIBRARIES})


