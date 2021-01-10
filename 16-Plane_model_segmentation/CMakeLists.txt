cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(planar_sementation)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (planar_sementation src/planar_sementation.cpp)
target_link_libraries (planar_sementation ${PCL_LIBRARIES})

# line_circle_seg
add_executable(line_circle_seg src/line_circle_seg.cpp)
target_link_libraries(line_circle_seg
    ${PCL_LIBRARIES}
)