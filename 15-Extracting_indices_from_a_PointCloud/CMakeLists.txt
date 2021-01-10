cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extract_indices)
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (extract_indices src/extract_indices.cpp)
target_link_libraries (extract_indices ${PCL_LIBRARIES})

# PCL line seg test
add_executable(pcl_line_seg src/pcl_line_seg.cpp)
target_link_libraries(pcl_line_seg
    ${PCL_LIBRARIES}
    stdc++fs    # <experimental/filesystem>
)