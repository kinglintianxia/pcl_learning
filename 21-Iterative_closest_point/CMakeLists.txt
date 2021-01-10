cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(21-iterative_closest_point)

# c++11
set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")
# 
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# demo 1
add_executable(${PROJECT_NAME} iterative_closest_point.cpp)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# demo 2
add_executable(${PROJECT_NAME}_monkey icp_monkey.cpp)
target_link_libraries(${PROJECT_NAME}_monkey ${PCL_LIBRARIES})

# demo 3:
add_library(normal_estimate2d normal_estimate2d/Normal2dEstimation.cpp normal_estimate2d/PCA2D.cpp)
target_link_libraries(normal_estimate2d 
    ${PCL_LIBRARIES}
)

# ICP rail_std & gocator use normal direction.
add_executable(icp_normal icp_normal.cpp)
target_link_libraries(icp_normal 
    ${PCL_LIBRARIES}
    stdc++fs # <experimental/filesystem>
    normal_estimate2d
)

# ICP curvature
add_executable(icp_curvature icp_curvature.cpp)
target_link_libraries(icp_curvature 
    ${PCL_LIBRARIES}
    normal_estimate2d
)

# ICP curvature 01
add_executable(icp_curvature_01 icp_curvature_01.cpp)
target_link_libraries(icp_curvature_01
    ${PCL_LIBRARIES}
    normal_estimate2d
    stdc++fs # <experimental/filesystem>
)

# Transformation estimation 2d
add_executable(transformation_estimation_2d transformation_estimation_2d.cpp)
target_link_libraries(transformation_estimation_2d 
    ${PCL_LIBRARIES}
    normal_estimate2d
    stdc++fs # <experimental/filesystem>
)

# Coherent point drift（CPD) test
find_package(Cpd COMPONENTS jsoncpp)
add_executable(cpd_test cpd_test.cpp)
target_link_libraries(cpd_test
    ${PCL_LIBRARIES}
    stdc++fs    # <experimental/filesystem>
    Cpd::Library-C++ 
    Cpd::Jsoncpp
)

