# the 6D pose of each point in a frame
float64  offset_time        # ROS standard message header
geometry_msgs/Vector3 acc  # the raw accelerator measurement with the bias
geometry_msgs/Vector3 gyr  # the raw angular velocity measurements with the bias
geometry_msgs/Vector3 bias_acc  # accelerator bias
geometry_msgs/Vector3 bias_gyr  # gyroscope bias
geometry_msgs/Vector3 vel  # the relative velocity w.r.t the end imu point
geometry_msgs/Vector3 pos  # the relative position w.r.t the end imu point
float64[9] rot             # the preintegrated rotation at the imu keypoint matrix w.r.t the end imu point