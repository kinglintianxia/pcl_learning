cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(cylinder_model_segmentation)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(cylinder_model_segmentation src/cylinder_model_segmentation.cpp)
target_link_libraries(cylinder_model_segmentation ${PCL_LIBRARIES})
