## LIKE_LOAM
LIKE_LOAM (Lidar Iterative Kalam Estimator LOAM) is a lidar-initial mapping package for Livox Avia LiDAR. 
The package currently contains the basic functions of real-time mapping and localization.

<div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div>

Some key issues:
1. Real-time tightly-coupled LiDAR-IMU Fusion;
2. Remove odometry;
3. Robust feature extraction.

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
    cd ..
    catkin_make
    source devel/setup.bash
```

*Remarks:*
- If you want to save the pcd file please add map_file_path in launch file.
## 3. Directly run
### 3.1 For outdoor environments
Connect to your PC to Livox Avia LiDAR following [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch like_loam mapping_avia_outdoor.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
    
```
### 3.2 For indoor environments
Connect to your PC to Livox Avia LiDAR by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch livox_mapping mapping_avia_indoor.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
    
```
## 4. Rosbag Example
### 4.1 Livox Avia outdoor rosbag

<div align="center"><img src="doc/results/HKU_MB_002.png" width=90% /></div>

<!-- <div align="center"><img src="doc/results/mid40_outdoor.png" width=90% /></div> -->

Download [avia_hku_mb_example](https://drive.google.com/file/d/1GSb9eLQuwqmgI3VWSB5ApEUhOCFG_Sv5/view?usp=sharing) and then
```
roslaunch livox_mapping mapping_avia_outdoor.launch
rosbag play YOUR_DOWNLOADED.bag
```
### 4.2 Livox Avia indoor rosbag

<div align="center"><img src="doc/results/HKU_LG_Indoor.png" width=90% /></div>

Download [avia_indoor_quick_shake_example](https://drive.google.com/file/d/1SWmrwlUD5FlyA-bTr1rakIYx1GxS4xNl/view?usp=sharing) and then
```
roslaunch livox_mapping mapping_avia_indoor.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).
