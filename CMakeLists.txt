cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree_demo)

add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -pthread -O3") 

find_package(PCL 1.8 REQUIRED)

include_directories(
    ${PCL_INCLUDE_DIRS}
    ikd-Tree
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer -fsanitize=address")
set (CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fno-omit-frame-pointer -fsanitize=address")

add_executable(ikd_tree_demo examples/ikd_Tree_demo.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree_demo ${PCL_LIBRARIES})

add_executable(ikd_tree_async_demo examples/ikd_Tree_Async_demo.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree_async_demo ${PCL_LIBRARIES})

add_executable(ikd_tree_Search_demo examples/ikd_Tree_Search_demo.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree_Search_demo ${PCL_LIBRARIES})

add_executable(ikd_simple_test examples/ikd_simple_test.cpp ikd-Tree/ikd_Tree.cpp)
target_link_libraries(ikd_simple_test ${PCL_LIBRARIES})
