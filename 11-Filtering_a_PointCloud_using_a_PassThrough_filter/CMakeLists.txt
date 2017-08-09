cmake_minimum_required(VERSION 2.8)

project(passthrough_filter)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 
add_definitions(${PCL_DIFINITIONS})

add_executable(passthrough_filter src/passthrough.cpp)
target_link_libraries(passthrough_filter ${PCL_LIBRARIES})


