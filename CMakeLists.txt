cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree_demo)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ikd_tree_demo ikd_Tree_demo.cpp ikd_Tree.cpp)
target_link_libraries(ikd_tree_demo ${PCL_LIBRARIES})