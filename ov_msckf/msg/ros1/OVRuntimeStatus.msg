# This message is published in publish_features() function

# state time (i.e., t_cam)
std_msgs/Header header

# Number of features
int64 num_slam_features
int64 num_msckf_features

# IMU bias (current estimate)
geometry_msgs/Vector3 bias_a
geometry_msgs/Vector3 bias_g


# Time offset base IMU to camera (t_imu = t_cam + t_off)
float64 t_offset_imu_cam