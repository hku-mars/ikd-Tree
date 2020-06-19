## Livox_mapping
Livox_mapping is a mapping package for Livox LiDARs. 
The package currently contains the basic functions of low-speed mapping.

<div align="center">
    <img src="doc/results/mid40_hall.gif" width = 45% >
    <img src="doc/results/horizon_parking.gif" width = 45% >
</div>

Some key issues:
1. Support multiple livox lidar;
2. Different feature extraction;
3. Remove odometry for small FOV situation;

In the development of our package, we reference to LOAM, LOAM_NOTED.
## 1. Prerequisites
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 16.04 or 18.04.
ROS Kinetic or Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

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
    git clone https://github.com/Livox-SDK/livox_mapping.git
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```

*Remarks:*
- If you want to save the pcd file please add map_file_path in launch file.
## 3. Directly run
### 3.1 Livox Mid-40
Connect to your PC to Livox LiDAR (mid40) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch livox_mapping mapping_mid.launch
    roslaunch livox_ros_driver livox_lidar.launch
    
```
### 3.2 Livox Horizon
Connect to your PC to Livox LiDAR (Horizon) by following  [Livox-ros-driver installation](https://github.com/Livox-SDK/livox_ros_driver), then
```
    ....
    roslaunch livox_mapping mapping_horizon.launch
    roslaunch livox_ros_driver livox_lidar.launch
    
```
## 4. Rosbag Example
### 4.1 Livox Mid-40 rosbag

<div align="center"><img src="doc/results/mid40_hall_01.png" width=90% /></div>

<div align="center"><img src="doc/results/mid40_outdoor.png" width=90% /></div>

Download [mid40_hall_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_hall_example.bag) or [mid40_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid40_outdoor.bag) 
and then
```
roslaunch livox_mapping mapping_mid.launch
rosbag play YOUR_DOWNLOADED.bag
```
### 4.2 Livox Mid-100 rosbag

<div align="center"><img src="doc/results/mid100_01.png" width=90% /></div>

<div align="center"><img src="doc/results/mid100_02.png" width=90% /></div>

Download [mid100_example](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/mid100_example.bag) and then
```
roslaunch livox_mapping mapping_mid.launch
rosbag play YOUR_DOWNLOADED.bag
```
### 4.3 Livox Horizon rosbag

<div align="center"><img src="doc/results/horizon_outdoor_01.png" width=90% /></div>

<div align="center"><img src="doc/results/horizon_parking_01.png" width=90% /></div>

Download [horizon_parking](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/horizon_parking.bag) or [horizon_outdoor](https://terra-1-g.djicdn.com/65c028cd298f4669a7f0e40e50ba1131/Showcase/horizon_outdoor.bag) 
and then
```
roslaunch livox_mapping mapping_horizon.launch
rosbag play YOUR_DOWNLOADED.bag
```
## 5.Acknowledgments
Thanks for LOAM(J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time), [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED).
