cmake_minimum_required(VERSION 2.8.3)
project(like_loam)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

#set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

#set(CMAKE_C_COMPILER "/usr/bin/clang")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")
# target_include_directories(myproject PRIVATE ${PYTHON_INCLUDE_DIRS})
# target_link_libraries(myproject ${PYTHON_LIBRARIES})

# set(Sophus_INCLUDE_DIRS "/usr/local/include/sophus")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
  # PythonLibs 3.6
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(Sophus REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)
  
add_message_files(
  FILES
  Pose6D.msg
  KeyPointPose.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL OpenCV
  INCLUDE_DIRS
)

add_executable(imu_process src/IMU_Processing.cpp)
target_link_libraries(imu_process ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(loam_feat_extract src/unity_feature.cpp)
target_link_libraries(loam_feat_extract ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(loam_laserMapping src/laserMapping.cpp)
target_link_libraries(loam_laserMapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${PYTHON_LIBRARIES})
target_include_directories(loam_laserMapping PRIVATE ${PYTHON_INCLUDE_DIRS})

