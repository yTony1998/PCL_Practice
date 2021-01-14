cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(myPCLPROJECT)
find_package(PCL 1.3 REQUIRED COMPONENTS common io )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(LOAD_PCD load_pcd.cpp)
target_link_libraries(LOAD_PCD ${PCL_LIBRARIES})
