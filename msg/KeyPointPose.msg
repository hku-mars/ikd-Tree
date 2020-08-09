
Header header       # timestamp of the first lidar in a frame
float64[3] gravity  # the estimated gravity acceleration
float64[3] bias_acc # accelerator bias
float64[3] bias_gyr # gyroscope bias
float64[3] pos_end  # the estimated position at the end lidar point
float64[9] rot_end  # the estimated attitude at the end lidar point
Pose6D[] pose6D     # 6D pose at each imu measurements