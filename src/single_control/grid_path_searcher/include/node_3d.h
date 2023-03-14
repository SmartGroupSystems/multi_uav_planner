#ifndef _NODE_3d_H_
#define _NODE_3d_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
//#include "backward.hpp"

#define inf 1>>20
struct GridNode3d;
typedef GridNode3d* GridNodePtr3d;

struct GridNode3d
{     
    int id;        // 1--> open set, -1 --> closed set
    Eigen::Vector3d coord; //坐标（真实，常为标准化后的
    Eigen::Vector3i dir;   // direction of expanding 扩展方向，为JPS准备?
    Eigen::Vector3i index;//索引（格子图
	
    double gScore, fScore;//打分
    GridNodePtr3d cameFrom;
    bool obs_around = false;
    // GridNodePtr3d twistFrom;
    // int cameFrom_Slash = 0;
    std::multimap<double, GridNodePtr3d>::iterator nodeMapIt;

    GridNode3d(Eigen::Vector3i _index, Eigen::Vector3d _coord){  
      //USAGE  初始化函数
		id = 0;//均为open list
		index = _index;
		coord = _coord;
		dir   = Eigen::Vector3i::Zero();//不设置扩展方向

		gScore = inf;//评分设为无穷
		fScore = inf;
		cameFrom = NULL;//无先导结点
    obs_around = false;
    }

    GridNode3d(){};
    ~GridNode3d(){};
};


#endif
