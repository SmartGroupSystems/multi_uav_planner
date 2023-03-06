#include <iostream>
#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;
/*-----------------PARAM_SAVED-----------------*/
Eigen::Vector3d odom_pos_, odom_vel_;
Eigen::Quaterniond odom_orient_;
geometry_msgs::PoseStamped  msg_pos;
geometry_msgs::TwistStamped msg_vel;

geometry_msgs::TwistStamped msg_vel_local;
geometry_msgs::PoseStamped  msg_pos_local;

nav_msgs::Odometry msg;
nav_msgs::Odometry msg_local;
nav_msgs::Odometry init_state;
string map_frame_id, base_link_frame_id;

bool first_call = true;
ros::Publisher fsm_pub_global , fsm_pub_local,swarm_pub;
double init_x, init_y, init_z;
void rcvPositionCallback(const nav_msgs::Odometry &odom);
int main(int argc,char** argv)
{
    ros::init(argc,argv,"multi_model_state");
    ros::NodeHandle nh("~");

   //获得参数
    nh.param("multi_model_state/map_frame_id", map_frame_id,string("map_uav"));
    nh.param("multi_model_state/base_link_frame_id", base_link_frame_id,string("base_link"));
    nh.param("multi_model_state/init_x", init_x, 0.0);
    nh.param("multi_model_state/init_y", init_y, 0.0);
    nh.param("multi_model_state/init_z", init_z, 0.0);
    //订阅话题
    // ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>
    //                             ("/gazebo/get_model_state");
    // ros::Publisher fsm_pub    = nh.advertise<nav_msgs::Odometry>
    //                             ("/extract_state_client/global_iris",1)
    ros::Subscriber position_sub   = nh.subscribe( "odom",  10, rcvPositionCallback );
// 坐标大广播
    swarm_pub    = nh.advertise<nav_msgs::Odometry>
                                ("/others_odom",1);                                
//发布map坐标
    fsm_pub_local    = nh.advertise<nav_msgs::Odometry>
                                ("local_odom",1);
//发布全局坐标   
    fsm_pub_global   = nh.advertise<nav_msgs::Odometry>
                                ("global_odom",1);

    ros::Rate rate(50.0); 
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}

void rcvPositionCallback(const nav_msgs::Odometry &odom)
{
            msg.header= odom.header;
            msg.header.frame_id       = "earth";
            msg.child_frame_id        =  map_frame_id;
            msg.pose.pose.position.x  = odom.pose.pose.position.x ;
            msg.pose.pose.position.y  =  odom.pose.pose.position.y;
            msg.pose.pose.position.z  = odom.pose.pose.position.z ;
            msg.pose.pose.orientation = odom.pose.pose.orientation;
            msg.twist.twist           = odom.twist.twist;
            msg.twist.covariance  = odom.twist.covariance;
            // msg_vel.twist             = odom->twist.twist;
            // msg_vel.header            = odom->header;
            // msg_vel.header.frame_id   = "world";
            // msg_pos.pose              = msg.pose.pose;
            // msg_pos.header            = odom->header;
            // msg_pos.header.frame_id   = "world";
            fsm_pub_global.publish(msg);
            swarm_pub.publish(msg);
            msg_local.header                = odom.header;
            msg_local.header.frame_id       = map_frame_id;
            msg_local.child_frame_id        = base_link_frame_id;
            msg_local.pose.pose.position.x  =  odom.pose.pose.position.x  - init_x;
            msg_local.pose.pose.position.y  =  odom.pose.pose.position.y- init_y;
            msg_local.pose.pose.position.z  =   odom.pose.pose.position.z - init_z;
            msg_local.pose.pose.orientation = odom.pose.pose.orientation;
            msg_local.twist.twist           = odom.twist.twist;
            msg_local.twist.covariance  = odom.twist.covariance;
            fsm_pub_local.publish(msg_local);
           

}