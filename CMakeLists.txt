cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ikd_tree_demo)

find_package(PCL 1.8 REQUIRED)


include_directories(
    ${PCL_INCLUDE_DIRS}
    ikd_Tree)
link_directories(${PCL_LIBRARIES_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ikd_tree_demo examples/ikd_Tree_demo.cpp ikd_Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree_demo ${PCL_LIBRARIES})

add_executable(ikd_tree_async_demo examples/ikd_Tree_Async_demo.cpp ikd_Tree/ikd_Tree.cpp)
target_link_libraries(ikd_tree_async_demo ${PCL_LIBRARIES})

add_executable(search_points_by_box examples/ikd_Tree_Search_Points_by_box.cpp ikd_Tree/ikd_Tree.cpp)
target_link_libraries(search_points_by_box ${PCL_LIBRARIES})