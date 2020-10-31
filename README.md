## FAST-LIO
**FAST-LIO** (Fast LiDAR-Inertial Odometry) is a computational effiencient LiDAR-Inertial Odometry package based on tightly coupled iterative Kalman filter. (It has been tested in Livox Lidar and will support some other Lidar platforms in the future)

<div align="center">
    <img src="doc/results/HKU_HW.png" width = 49% >
    <img src="doc/results/HKU_MB_001.png" width = 49% >
</div>

is a robust, low drift, and real time odometry and mapping package for Livox LiDARs, significant low cost and high performance LiDARs that are designed for massive industrials uses. Our package address many key issues: feature extraction and selection in a very limited FOV, robust outliers rejection, moving objects filtering, and motion distortion compensation. In addition, we also integrate other features like parallelable pipeline, point cloud management using cells and maps, loop closure, utilities for maps saving and reload, etc. To know more about the details, please refer to our related paper:)

Our package address many key issues:
1. Real-time tightly-coupled LiDAR-IMU Odometry and mapping;
2. Can be automaticaly initialized at most steady environments;
3. Robust feature extraction and .

## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu >= 18.04.

ROS    >= Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **PCL && Eigen && openCV**
PCL    >= 1.8,   Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

Eigen  >= 3.3.4, Follow [Eigen Installation](http://eigen.tuxfamily.org/index.php?title=Main_Page).

OpenCV >= 3.2,   Follow [openCV Installation](https://opencv.org/releases/).

### 1.3. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).


## 2. Build
Clone the repository and catkin_make:

```
    cd ~/catkin_ws/src
    git clone https://github.com/XW-HKU/fast_lio.git
    cd ..
    catkin_make
    source devel/setup.bash
```

*Remarks:*
- If you want to save the pcd file please add map_file_path in launch file.
## 3. Directly run
### 3.1 For indoor environments and high LiDAR sample rate (20-100hz)
Connect to your PC to Livox Avia LiDAR by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch fast_lio mapping_avia.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
    
```

### 3.2 For outdoor environments
Connect to your PC to Livox Avia LiDAR following [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch fast_lio mapping_avia_outdoor.launch
    roslaunch livox_ros_driver livox_lidar_msg.launch
    
```
## 4. Rosbag Example
### 4.1 Indoor rosbag (Livox Avia LiDAR)

<div align="center"><img src="doc/results/HKU_LG_Indoor.png" width=100% /></div>

Download [avia_indoor_quick_shake_example1](https://drive.google.com/file/d/1SWmrwlUD5FlyA-bTr1rakIYx1GxS4xNl/view?usp=sharing) or [avia_indoor_quick_shake_example2](https://drive.google.com/file/d/1wD485CIbzZlNs4z8e20Dv2Q1q-7Gv_AT/view?usp=sharing) and then
```
roslaunch fast_lio mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

### 4.2 Outdoor rosbag (Livox Avia LiDAR)

<div align="center"><img src="doc/results/HKU_MB_002.png" width=100% /></div>

<!-- <div align="center"><img src="doc/results/mid40_outdoor.png" width=90% /></div> -->

Download [avia_hku_main building_mapping](https://drive.google.com/file/d/1GSb9eLQuwqmgI3VWSB5ApEUhOCFG_Sv5/view?usp=sharing) and then
```
roslaunch fast_lio mapping_avia_outdoor.launch
rosbag play YOUR_DOWNLOADED.bag
```

### 4.3 High-rate rosbag (Livox Avia LiDAR sampled at 100Hz)

Download [high_rate_avia](https://drive.google.com/file/d/1UM6O3PRN3b730ZeuvKKT3yuOLNQuz8Yf/view?usp=sharing)
```
roslaunch fast_lio mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [Livox_Mapping](https://github.com/Livox-SDK/livox_mapping) and [Loam_Livox](https://github.com/hku-mars/loam_livox).
