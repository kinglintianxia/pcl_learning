cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(narf_keypoint_extraction)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(narf_keypoint_extraction src/narf_keypoint_extraction.cpp)
target_link_libraries(narf_keypoint_extraction ${PCL_LIBRARIES})

