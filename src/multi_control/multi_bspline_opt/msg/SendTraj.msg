int32 drone_id
int64 traj_id
int32 order
int32 cps_num_
# int32 Dim_
# int32 TrajSampleRate
# float64 beta
time start_time


geometry_msgs/Point start_pos_
geometry_msgs/Point start_vel_
geometry_msgs/Point start_acc_
geometry_msgs/Point end_pos_
# float64 start_pos_x
# float64 start_pos_y
# float64 start_vel_x
# float64 start_vel_y
# float64 start_acc_x
# float64 start_acc_y
# float64 end_pos_x
# float64 end_pos_y
# float64 yaw_rate

geometry_msgs/Point[] control_pts
float64[] knots