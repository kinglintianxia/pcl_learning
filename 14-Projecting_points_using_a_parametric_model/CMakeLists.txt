cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(project_inliers)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(project_inliers src/project_inliers.cpp)
target_link_libraries(project_inliers ${PCL_LIBRARIES})

# Test parametric_mode.cpp
add_executable(parametric_mode src/parametric_mode.cpp)
target_link_libraries(parametric_mode
    ${PCL_LIBRARIES}
)