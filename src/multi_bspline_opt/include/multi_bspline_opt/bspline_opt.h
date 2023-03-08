#ifndef  _MULTI_BSPLINE_OPT_H
#define  _MULTI_BSPLINE_OPT_H

//Eigen
#include<eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include<eigen3/Eigen/Core>

//Nlopt optimization
#include<nlopt/nlopt.hpp>

//STANDARD
#include <algorithm>
#include <iostream>
#include<math.h>
#include<vector>
#include <numeric>
#include<string>
#include <memory>

//ros
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/Imu.h> 
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include "tf/transform_datatypes.h"
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
//ros多线程
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <boost/thread/thread.hpp>

//宏定义
#define pi 3.14159265359

//自定义
#include<multi_bspline_opt/BsplineTraj.h>
#include<multi_bspline_opt/EdtTransform.hpp>
#include<multi_bspline_opt/MultiBsplines.h>
#include<multi_bspline_opt/SendTraj.h>

using namespace std;

namespace my_planner
{
     class UniformBspline
    {
        public://
           int  p_, n_, m_;//p degree， n+1 is the number of control points, m = n+p+1
           Eigen::VectorXd u_ ; // knot vectors
           double beta_;// time scale t(real time) * beta = u
           int D_;// Dimension of control points


    //物理限制
           double limit_vel_,limit_acc_,limit_ratio_,feasibility_tolerance_;
        //    double 
        public://变量
            Eigen::MatrixXd control_points_;
            Eigen::MatrixXd A_ini, A_ter; // A_ini*[cp1, cp2, cp3] = [p0, v0, a0] ,similarly A_ter; DIM: 3*3
            Eigen::MatrixXd s_ini_, s_ter_; // initial state, terminal state   DIM: 3*2
            Eigen::Vector2d s_range;
            Eigen::Vector2d t_range;
            double interval_;//节点时间间隔
            Eigen::VectorXd time_;//时间长度向量

        public://函数
            UniformBspline() {}
            //类的重载函数，放在这里
            UniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                               const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter);
            ~UniformBspline();
            void initUniformBspline(const int &p,  const int &n, const double &beta, const int &D, 
                                                               const Eigen::MatrixXd &s_ini, const Eigen::MatrixXd &s_ter); 
            void setControlPoints(const Eigen::MatrixXd &ctrl_points);
            void setIniTerMatrix();
            Eigen::MatrixXd getTrajectory(const Eigen::VectorXd &t);
            Eigen::Vector2d singleDeboor(const double &u_probe);
            Eigen::VectorXd getKnot() { return this->u_; }
            void getAvailableSrange();
            double getAvailableTrange();
            void getInterval();
            void getT(const int &trajSampleRate);//轨迹采样点的时间序列
            UniformBspline getDerivative();//返回b样条的导数类
            Eigen::VectorXd getBoundConstraintb();//得到边界
            void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
            bool checkFeasibility(bool show);
              void lengthenTime();
    };
//SWARM
//swarm
struct OneTrajDataOfSwarm
  {
    /* info of generated traj */

    int drone_id;
    double duration_;
    ros::Time start_time_;
    Eigen::Vector2d start_pos_;
    UniformBspline position_traj_;
    // vector<Eigen::Vector2d> traj_;
    // vector<Eigen::Vector2d> acc_;
    // vector<Eigen::Vector2d> vel_;
  };
  typedef std::vector<OneTrajDataOfSwarm> SwarmTrajData;
    class bspline_optimizer
    {
        public:
            int cps_num_;//控制点个数
            int p_order_;//轨迹阶次
            int Dim_;//b样条维度
            double bspline_interval_;//节点向量时间间隔
            double beta_;
            UniformBspline u_;

            //从状态机读入的参数
            double lambda1_,lambda2_,lambda3_,lambda4_, lambda5_ ;// smooth cost, ESDF cost, feasibility cost
            double max_vel_;//最大速度
            double max_acc_;//最大加速度

            //从A star得到的路径点
            std::vector<Eigen::Vector2d> path_;
            
             //集群参数
           #define INIT_min_ellip_dist_ 123456789.0123456789
            double min_ellip_dist_;
            //ESDF
            Eigen::MatrixXd esdf_map_;
            double map_resolution_;//地图分辨率
            double origin_x_, origin_y_;//地图起始点x,y，右手坐标系 ，最左下角的栅格子的中心
            double startPoint_x,startPoint_y;//找到地图左上角的点，x负，y正

            //nlopt optimizaiton
            Eigen::MatrixXd control_points_;
            int iter_num_;       // iteration of the solver
            int variable_num;//变量个数
            std::vector<double> best_variable_;  //nlopt最终输出
            double safe_distance_, swarm_clearance_;//安全距离
            
            double min_cost_;       //
            int    algorithm1_ = 15;             // optimization algorithms for quadratic cost
            int    algorithm2_ = 11;             // optimization algorithms for general cost
        
            Eigen::Vector2d drone_pos_world; //无人机位置
            Eigen::Vector4d  other_drone_pose;//其它无人机位置
        //多机
        private:
            SwarmTrajData *swarm_trajs_{NULL};
            int drone_id_;
        public://函数
            bspline_optimizer() {}
            bspline_optimizer(const std::vector<Eigen::Vector2d> &path, const int&Dim,  const int&p,const Eigen::Vector2d drone_pos_world_, const Eigen::Vector4d other_drone_pose_);
            bspline_optimizer(const int&Dim,const int &p,const double &dist);
            ~bspline_optimizer();

      
            void setSwarmTrajs(SwarmTrajData *swarm_trajs_ptr);
            void setDroneId(const int drone_id) { drone_id_ = drone_id; }  
            void setOptParam(const double lambda1,const double lambda2,const double lambda3,const double lambda4,const double lambda5,
                                                    const double safe_dist,const double swarm_clearance_);
            void setMapParam(const double &origin_x,const double &origin_y, const double &map_resolution,
                                                    const double &start_x, const double &start_y);
            void setVelAcc(const double vel, const double acc);
            void setSmoothParam(const double lambda1,const double lambda2,
                                                            const double vel, const double acc);
            void setSplineParam(const UniformBspline &u);
            
            //从mapping读入
            void setEsdfMap(const Eigen::MatrixXd &esdf_map);
            void initialControlPoints(UniformBspline u);
            //nlopt相关
            void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                                                    Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
            void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);
            void calcEsdfCost(const Eigen::MatrixXd &q, double &cost,
                                                        Eigen::MatrixXd &gradient);   
            double calcDistance(const Eigen::MatrixXd &q);
            void  calcSwarmCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
             void  calcDroneCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
            Eigen::Vector2d calcGrad(const Eigen::MatrixXd &q);   
            void getSurroundPts(const Eigen::Vector2d& pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d& diff);
            void getSurroundDistance(Eigen::Vector2d pts[2][2], double dists[2][2]);
            void interpolateBilinear(double values[2][2], const Eigen::Vector2d& diff, Eigen::Vector2d& grad);
            void interpolateBilinearDist(double values[2][2], const Eigen::Vector2d& diff, double& dist);    
            void combineCost( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine);
            void combineCostSmooth( const std::vector<double>& x,Eigen::MatrixXd &grad,double &f_combine);
            static double costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data);
            static double costFunctionSmooth(const std::vector<double>& x, std::vector<double>& grad,
                                                        void* func_data);
            void optimize();//求解NLOPT，将最优变量保存为控制点  
            void optimize_withoutesdf();//对比
            void optimizesmooth();//const std::shared_ptr u
            template <typename T> std::vector<size_t> sort_indexes(std::vector<T> &v)//实现matlab mink函数
            {   
                std::vector<size_t> idx(v.size());
                iota(idx.begin(), idx.end(), 0);
                sort(idx.begin(), idx.end(),
                [&v](size_t i1, size_t i2) {return v[i1] < v[i2]; });
                return idx;// 返回索引
            }
            template<typename T>  inline T lerp(const T &lo, const T &hi, float t)  { return (lo * (0.1 - t) + hi * t)*10; }
    };


    class plan_manager
    {
        public:
            //从launch读取的参数
            double last_time_;
            double back_time_ ;
            int drone_id_;
            int p_order_;// order of bspline
            int N_;// number of control points
            int Dim_;// dimension of traj
            int TrajSampleRate;// 轨迹采样频率
            double beta;
            double dist_p;//控制点之间的距离
            double max_vel_,max_acc_;//最大速度，加速度
            Eigen::MatrixXd initial_state,terminal_state;//初始，结束P V A
            double start_x_, start_y_;// A star找到的起点
            double goal_x_,goal_y_;//A star找到的终点
            int map_size_x,map_size_y,map_size;//esdf x y
            Eigen::MatrixXd esdf_map_;//esdf地图
            Eigen::MatrixXd grid_map_;//grid map
            std::string frame_;
            double map_resolution_;//地图分辨率
            double origin_x_, origin_y_;
            double startPoint_x,startPoint_y;//找到地图左上角的点，x负，y正
            double safe_distance_,swarm_clearance_;//安全距离
            double esdf_collision;
            double tolerance_;
            Eigen::MatrixXd p_,v_,a_;//轨迹buffer

            int current_seq = 0;
            Eigen::Vector2d current_pos;
            Eigen::Vector2d last_endpoint;
            Eigen::Vector2d current_aim = Eigen::Vector2d::Zero();
            Eigen::Vector2d current_vel = Eigen::Vector2d::Zero();
            Eigen::Vector2d current_acc = Eigen::Vector2d::Zero();
            //同步器
            typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, 
                                                                    geometry_msgs::PoseStamped, 
                                                                    sensor_msgs::Imu> syncPolicy;
            message_filters::Subscriber<geometry_msgs::TwistStamped>* subscriber_vel;
            message_filters::Subscriber<geometry_msgs::PoseStamped>* subscriber_pos;
            message_filters::Subscriber<sensor_msgs::Imu>* subscriber_acc;
            message_filters::Synchronizer<syncPolicy>* sync;


            //状态相关
            enum traj_state 
            {
                COLLIDE = 0,
                SAFE = 1
            };  //碰撞状态

            enum PLANNER_STATE
           {
                 FLYING,
                 SEQUENTIAL_START
            };

            PLANNER_STATE now_state_;
            int continously_called_times_{0};  //状态连续几次


            bool get_path = false;
            bool get_map = false;
            bool get_traj = false;
            bool first_rifine = true;
            bool enable_flag = false;
            bool start_up = true;//是否启动状态

            //智能类指针
            std::shared_ptr<UniformBspline> u;
            std::shared_ptr<UniformBspline> u1;
            std::shared_ptr<bspline_optimizer> opt;
            std::shared_ptr<bspline_optimizer> opt1;
            //nlopt 相关
            double lambda1_,lambda2_,lambda3_,lambda4_,lambda5_ ,lambda3_saved;

            //多无人机
             double planning_horizen_, planning_horizen_time_;
             multi_bspline_opt::MultiBsplines multi_bspline_msgs_buf_;
              bool have_recv_pre_agent_;  //是否收到之前无人机轨迹
             SwarmTrajData swarm_trajs_buf_;
             bool have_odom_;
            //ros
            ros::Subscriber goal_suber;//订阅RVIZ导航点，备用
            ros::Subscriber path_suber;//订阅A star路径点
            ros::Subscriber map_suber;//订阅esdf map
            ros::Subscriber odom_suber;//订阅无人机位置
            ros::Publisher Traj_vis;//轨迹可视化发布
            ros::Publisher Traj_vis1;//轨迹可视化发布
            ros::Publisher Traj_puber;//发布轨迹
            ros::Publisher Time_puber;
            ros::Publisher Map_puber;//发布esdf地图可视化
            ros::Publisher col_check;// JS change
            ros::Publisher traj_smooth;
            ros::Publisher state_pub;//状态发布器
            ros::ServiceServer fsm_call;
            ros::Subscriber state_suber;
            ros::Subscriber aim_suber;
            ros::Subscriber fullaim_suber;
            ros::Subscriber waypoint_suber;
            ros::Subscriber fsm_suber;
            ros::Subscriber arrived_suber;

            //多无人机
            ros::Subscriber  swarm_trajs_sub_;  //订阅上一个无人机的轨迹集合
            ros::Subscriber  broadcast_bspline_sub_;
            ros::Subscriber   droneX_odom_sub_;
            ros::Publisher swarm_trajs_pub_;
            ros::Publisher  broadcast_bspline_pub_ ;
            ros::Timer traj_timer_, safety_timer_ ;
            Eigen::Vector2d drone_pos_world; //无人机位置
            Eigen::Vector4d  other_drone_pose;//其它无人机位置
            //ros msg
            nav_msgs::Path traj_vis;//轨迹可视化
            nav_msgs::Path traj_vis_;//轨迹可视化
            nav_msgs::Path traj_vis1;//轨迹可视化
            multi_bspline_opt::BsplineTraj traj;//执行轨迹
            multi_bspline_opt::BsplineTraj traj_;//执行轨迹
            multi_bspline_opt::SendTraj traj_pub;//发给其它无人机
            std::vector<Eigen::Vector2d> astar_path_;

            /* 色表 */
            vector<int> R = { 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240 };
            vector<int> G = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
            vector<int> B = { 0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
            240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        public:
            plan_manager(){}
            plan_manager(ros::NodeHandle &nh);
            ~plan_manager();
            void setParam(ros::NodeHandle &nh);//从ros节点中读取参数
            void TrajPlanning(ros::NodeHandle &nh);//轨迹规划
            bool checkTrajCollision();//检测轨迹是否发生碰撞
            void uav_goal_subCallback(const geometry_msgs::PoseStampedConstPtr &goal_msg);
            void esdf_map_subCallback(const std_msgs::Float64MultiArrayConstPtr &map_msg);
            std::vector<float>  calculate_color(double esdf_value, double max_dist, double min_dist, std::vector<int> R_values, std::vector<int> G_values, std::vector<int> B_values);
            void astar_getCallback(const nav_msgs::PathConstPtr &path);
            void map_slice_output(const Eigen::MatrixXd &esdf_matrix);
            bool fsm_callback(std_srvs::Trigger::Request & req,std_srvs::Trigger::Response & res);
            void current_state_callback(const geometry_msgs::TwistStampedConstPtr & vel_msg,
                                        const geometry_msgs::PoseStampedConstPtr &pos_msg,
                                        const sensor_msgs::ImuConstPtr &imu_msg);
            void OdomCallback(const nav_msgs::Odometry &pos_msg);                            
            void aim_callback(const geometry_msgs::PoseStamped::ConstPtr & aim_msg);
            void fullaim_callback(const mavros_msgs::PositionTarget::ConstPtr & aim_msg);
            bool  astar_subCallback(const std::vector<Eigen::Vector2d> &astar_path_);
             void stateFSMCallback(/*const ros::TimerEvent &e*/); 
             void StateChange( PLANNER_STATE new_state, string pos_call);
            void   checkCollisionCallback(const ros::TimerEvent &e);
            void smooth_subCallback(const nav_msgs::Path::ConstPtr & msg);
            void fsm_subCallback(const std_msgs::Int64::ConstPtr & msg);
            void arrive_callback(const std_msgs::Int64::ConstPtr & msg);
           void BroadcastBsplineCallback(const multi_bspline_opt::SendTraj::ConstPtr &msg) ;
           void PublishSwarm(bool startup_pub);
           void swarmTrajsCallback(const multi_bspline_opt::MultiBsplinesPtr &msg);
           void rcvDroneXOdomCallback(const nav_msgs::Odometry& odom);
           void rcvDroneOdomCallbackBase(const nav_msgs::Odometry& odom, int other_drone_id);
            Eigen::MatrixXd getSmoothTraj(const geometry_msgs::PoseStamped &start,
                                                                            const geometry_msgs::PoseStamped &end);
            Eigen::Vector2i posToIndex(const Eigen::MatrixXd &pos)
            {
                Eigen::Vector2i curr_index;
                double dist_x = pos(0,0) - startPoint_x;
                double dist_y = startPoint_y - pos(0,1);
                curr_index<< floor(dist_y/map_resolution_),floor(dist_x/map_resolution_);
                return curr_index;
            }
    };

}
#endif
