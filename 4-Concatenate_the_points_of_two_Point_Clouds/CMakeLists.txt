cmake_minimum_required(VERSION 2.8)

project(concatenate_cloud )

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(concatenate_clouds src/concatenate_clouds.cpp)
target_link_libraries(concatenate_clouds ${PCL_LIBRARIES})

