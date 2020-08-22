
Header header          # timestamp of the first lidar in a frame
float32[] gravity      # the estimated gravity acceleration
float32[] bias_gyr     # gyroscope bias
float32[] bias_acc     # accelerator bias
float32[] pos_end      # the estimated position at the end lidar point (world frame)
float32[] vel_end      # the estimated velocity at the end lidar point (world frame)
float32[] rot_end      # the estimated attitude at the end lidar point
float32[] cov          # 18 * 18
Pose6D[] pose6D        # 6D pose at each imu measurements