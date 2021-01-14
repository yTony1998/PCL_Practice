cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(myPCLPROJECT)

find_package(PCL 1.3 REQUIRED COMPONENTS common io visualization)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINTIONS})

add_executable(cloud_viewer cloud_viewer.cpp)
target_link_libraries(cloud_viewer ${PCL_LIBRARIES})
