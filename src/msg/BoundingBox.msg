std_msgs/Header header
string label_id
int32 tracking_id
float32 confidence

# 3D bbox properties
float32 distance
geometry_msgs/Point centroid
geometry_msgs/Point dimension
geometry_msgs/Quaternion orientation
geometry_msgs/Twist velocity
geometry_msgs/Twist acceleration
sensor_msgs/PointCloud2 object_cluster

# 2D bbox properties:
int32 x
int32 y
int32 w
int32 h
float32 yaw_angle


bool pose_reliable
bool velocity_reliable
bool acceleration_reliable
# FORWARD_STATE = 0, STOPPING_STATE = 1, BRANCH_LEFT_STATE = 2, BRANCH_RIGHT_STATE = 3, YIELDING_STATE = 4, ACCELERATING_STATE = 5, SLOWDOWN_STATE = 6
uint8 behavior_state