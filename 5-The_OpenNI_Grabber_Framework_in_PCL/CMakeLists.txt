cmake_minimum_required (VERSION 2.8 FATAL_ERROR)

project(openni_viewer)

find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

add_executable (openni_viewer src/openni_viewer_simple.cpp)
target_link_libraries(openni_viewer ${PCL_LIBRARIES})

add_executable (openni_grabber_xyzrgb src/openni_grabber_xyzrgb.cpp)
target_link_libraries(openni_grabber_xyzrgb ${PCL_LIBRARIES})
