# include<ros/ros.h>
#include<multi_bspline_opt/bspline_opt.h>
using namespace my_planner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Test_planning");
    ros::NodeHandle nh("~");
    plan_manager manager(nh);
    ros::Rate rate(50.0); 
    ros::spin();
    return 0;
}
