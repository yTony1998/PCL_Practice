cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kd_treeProject)

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization kd_tree)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(kd_tree kd_tree.cpp)
target_link_libraries(kd_tree ${PCL_LIBRARIES})