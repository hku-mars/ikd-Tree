cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree_demo)

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3") 

add_executable(ikd_tree_demo ikd_Tree_demo.cpp ikd_Tree.cpp)
