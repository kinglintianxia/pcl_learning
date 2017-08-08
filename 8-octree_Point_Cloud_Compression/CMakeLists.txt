cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(point_cloud_compression)

find_package(PCL  REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (point_cloud_compression src/point_cloud_compression.cpp)
target_link_libraries (point_cloud_compression ${PCL_LIBRARIES})
