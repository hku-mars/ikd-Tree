## LIKE_LOAM
LIKE_LOAM is a lidar-initial mapping package for Livox Avia LiDAR. 
The package currently contains the basic functions of real-time mapping and localization.

<div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div>

Some key issues:
1. Support Livox Avia;
2. IMU Fusion;
3. Remove odometry;
4. New and robust feature extraction

In the development of our package, we reference to LOAM, LOAM_NOTED.
## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.
ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen && openCV**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).
Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).
Follow [openCV Installation](https://opencv.org/releases/).

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/XW-HKU/LIKE_LOAM.git
    catkin_make
    source devel/setup.bash
```

*Remarks:*
- If you want to save the pcd file please add map_file_path in launch file.
## 3. Directly run
### 3.1 For indoor environments
Connect to your PC to Livox LiDAR (mid40) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch like_loam mapping_avia_indoor.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
    
```
### 3.2 For outdoor environments
Connect to your PC to Livox LiDAR (Horizon) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch livox_mapping mapping_avia_outdoor.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
    
```
## 4. Rosbag Example
### 4.1 Livox Avia outdoor rosbag

<div align="center"><img src="doc/results/HKU_MB_002.png" width=90% /></div>

<!-- <div align="center"><img src="doc/results/mid40_outdoor.png" width=90% /></div> -->

Download [mid40_hall_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_hall_example.bag) or [mid40_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_outdoor.bag) 
and then
```
roslaunch livox_mapping mapping_mid.launch
rosbag play YOUR_DOWNLOADED.bag
```
### 4.2 Livox Avia indoor rosbag

<div align="center"><img src="doc/results/HKU_LG_Indoor.png" width=90% /></div>

Download [mid100_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid100_example.bag) and then
```
roslaunch livox_mapping mapping_mid.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).
