#include <iostream>
#include <vector>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <std_msgs/Float64MultiArray.h>
#include <complex.h>
using namespace std;
using namespace Eigen;
void haha_test();
void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                        Eigen::MatrixXd &gradient);
int main(){
    Eigen::MatrixXd control_points;
    double cost;
    Eigen::MatrixXd grad;


    return 0;
}
void haha_test()
{
    Eigen::Vector3i aim_ptr(7,-13,-2);
    Eigen::Vector3i start_ptr(0,0,0);
    Eigen::Vector3i vector_ori = aim_ptr - start_ptr;
    Eigen::Vector3i vector_abs = vector_ori.array().abs();
    Eigen::Vector3i vector     = Eigen::Vector3i::Zero();
    std::vector<int> vector_seq(3, 0);
    Eigen::Matrix3i R_ = Matrix3i::Zero(3,3);
    std::pair<bool,Eigen::Vector3i> end_idx(false, start_ptr);
// 准备
        
    for (int i = 0 ; i < 3 ; i++) {
        vector_seq[i] = i;
    }
        
    sort(vector_seq.begin(), vector_seq.end(),
        [&](const int& a, const int& b) {
            return (vector_abs[a] > vector_abs[b]);
        }
    );
        
    for (size_t i = 0; i < 3; i++)
    {
        int index_ptr = vector_seq[i];
        R_( index_ptr, i) = 1;
    }
        
    vector = Eigen::Vector3i(vector_ori.transpose()*R_);
        
// 计算
        
    int dx, dy, dz,
        sx, sy, sz,
        func_add_y, func_add_z,
        flag_y, flag_z,
        d;
    dx = vector(0);
    dy = vector(1);
    dz = vector(2);
    sx = (dx < 0)? -1:1;
    sy = (dy < 0)? -1:1;
    sz = (dz < 0)? -1:1;
    dx = sx * dx;
    dy = sy * dy;
    dz = sz * dz;
    func_add_y = 2*dy;
    func_add_z = 2*dz;
    flag_y     = -dx;
    flag_z     = -dx;
    d          = -2*dx;
    Eigen::Vector3i next_idx(-sx,0,0);

    while (next_idx.x() != vector.x() || 
           next_idx.y() != vector.y() || 
           next_idx.z() != vector.z())
    {
        next_idx[0] = next_idx[0] + sx;
        next_idx[1] = next_idx[1] + (flag_y>0)*sy;
        next_idx[2] = next_idx[2] + (flag_z>0)*sz;
        flag_y = flag_y + func_add_y + (flag_y>0)*d;
        flag_z = flag_z + func_add_z + (flag_z>0)*d;
        cout << Eigen::Vector3i(next_idx.transpose()*R_.transpose()) + start_ptr<<endl;
    }
}

void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                        Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    bool falg_use_jerk = false;
    if (falg_use_jerk)
    {
        Eigen::Vector3d jerk, temp_j;

        for (int i = 0; i < q.cols() - 3; i++)
        {
            /* evaluate jerk */
            jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
            cost += jerk.squaredNorm();
            temp_j = 2.0 * jerk;
            /* jerk gradient */
            gradient.col(i + 0) += -temp_j;
            gradient.col(i + 1) += 3.0 * temp_j;
            gradient.col(i + 2) += -3.0 * temp_j;
            gradient.col(i + 3) += temp_j;
        }
    }
    else
    {
        Eigen::Vector3d acc, temp_acc;

        for (int i = 0; i < q.cols() - 2; i++)
        {
            /* evaluate acc */
            acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
            cost += acc.squaredNorm();
            temp_acc = 2.0 * acc;
            /* acc gradient */
            gradient.col(i + 0) += temp_acc;
            gradient.col(i + 1) += -2.0 * temp_acc;
            gradient.col(i + 2) += temp_acc;
        }
    }
}
