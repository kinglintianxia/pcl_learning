cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project (pcd_write)

find_package(PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

add_executable (pcd_write src/pcd_write.cpp)
target_link_libraries(pcd_write ${PCL_LIBRARIES})
