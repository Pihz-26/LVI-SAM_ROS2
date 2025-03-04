# Cloud Info
std_msgs/Header header 

int32[] start_ring_index
int32[] end_ring_index

int32[]  point_col_ind # point column index in range image
float32[] point_range # point range 

int64 imu_available
int64 odom_available

# Attitude for lidar odometry initialization
float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init

# Odometry 
float32 odom_x
float32 odom_y
float32 odom_z
float32 odom_roll
float32 odom_pitch
float32 odom_yaw

# Odometry reset ID
int64 odom_reset_id

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed  # original cloud deskewed
sensor_msgs/PointCloud2 cloud_corner    # extracted corner feature
sensor_msgs/PointCloud2 cloud_surface   # extracted surface feature