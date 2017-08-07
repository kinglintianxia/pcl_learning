cmake_minimum_required(VERSION 2.8)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})

add_executable(matrix_transform src/matrix_transform.cpp)
target_link_libraries(matrix_transform ${PCL_LIBRARIES})
