cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_STANDARD 11)

project(normal_estimation_using_integral_images.cpp)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(normal_estimation_using_integral_images normal_estimation_using_integral_images.cpp)
target_link_libraries (normal_estimation_using_integral_images ${PCL_LIBRARIES})