
Header header   # timestamp of the last imu in a frame
float64[3] gravity # the estimated gravity acceleration
float64[3] pos_inframe # the estimated position at the end imu point
float64[9] rot_inframe # the estimated attitude at the end imu point
Pose6D[] pose6D # 6D pose at each imu points