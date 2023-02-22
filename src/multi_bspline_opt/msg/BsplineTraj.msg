# std_msgs/Header header

int32 drone_id
int64 traj_id
time start_time
float64 duration

float64 yaw
float64 yaw_rate

geometry_msgs/Point[] position
geometry_msgs/Point[] velocity
geometry_msgs/Point[] acceleration

uint32 current_seq
