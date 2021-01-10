cmake_minimum_required(VERSION 2.8)

# project
project(pcl_learning)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

## 15
add_subdirectory(15-Extracting_indices_from_a_PointCloud)

## 18
add_subdirectory(18-PCLPlotter)
## 21
add_subdirectory(21-Iterative_closest_point)
## 25
add_subdirectory(25-How_to_incrementally_register_pairs_of_clouds)