<<<<<<< HEAD
std_msgs/Header header

geometry_msgs/PoseStamped[] position
geometry_msgs/PoseStamped[] velocity
geometry_msgs/PoseStamped[] acceleration

float32 yaw
float32 yaw_rate
=======
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

>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
uint32 current_seq
