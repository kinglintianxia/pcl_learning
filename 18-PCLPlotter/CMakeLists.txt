cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_plotter)

find_package(PCL 1.7 REQUIRED COMPONENTS common visualization)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_plotter src/pcl_plotter_demo.cpp)
target_link_libraries(pcl_plotter ${PCL_COMMONLIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

add_executable(pcl_simple_plotter src/pcl_simple_plotter.cpp)
target_link_libraries(pcl_simple_plotter ${PCL_COMMONLIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

## plot rail std
add_executable(rail_std_plot src/rail_std_plot.cpp)
target_link_libraries(rail_std_plot
    ${PCL_LIBRARIES}
)   