

#ifndef _ASTAR_SEARCHER_3d_H
#define _ASTAR_SEARCHER_3d_H

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <vector>
//#include "backward.hpp"
#include "node_3d.h"

class AstarPathFinder3d
{	
	private:

	protected:
		uint8_t * data;
		GridNodePtr3d *** GridNodeMap;
		std::vector<GridNodePtr3d> usedPtrList;
		Eigen::Vector3i goalIdx;
		int GLX_SIZE, GLY_SIZE, GLZ_SIZE;
		int GLXYZ_SIZE, GLYZ_SIZE;

		double resolution, inv_resolution, interval_;
		double gl_xl, gl_yl, gl_zl;
		double gl_xu, gl_yu, gl_zu;
		// bool first_node_expanded;

		GridNodePtr3d terminatePtr;
		std::multimap<double, GridNodePtr3d> openSet;
		// std::multimap<double, GridNodePtr3d> closedSet;
		/* USAGE openSet
			multimap 类型，存储多键值对
			此处存储的键值对为：
					<double, GridNodePtr3d>{ Score  ,  Ptr }
			参考：http://c.biancheng.net/view/7190.html
		 */

		double getHeu(GridNodePtr3d node1, GridNodePtr3d node2);
		bool AstarGetSucc(GridNodePtr3d currentPtr, std::vector<GridNodePtr3d> & neighborPtrSets, std::vector<double> & edgeCostSets);		

		bool isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isOccupied(const Eigen::Vector3i & index) const;		
		bool isFree(const int & idx_x, const int & idx_y, const int & idx_z) const;
		bool isFree(const Eigen::Vector3i & index) const;


	public:
		;//
		AstarPathFinder3d(){};
		~AstarPathFinder3d(){};
		Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i & index);
		Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d & pt);
		bool getData(const Eigen::Vector3d & pt);
		bool getData(const Eigen::Vector3i & pt);
		bool arrived(const Eigen::Vector3d & pt1,const Eigen::Vector3d & pt2);
		bool nearly_arrived(const Eigen::Vector3d & pt1,const Eigen::Vector3d & pt2, double threshold);
		bool AstarGraphSearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);
		void resetGrid(GridNodePtr3d ptr);
		void resetUsedGrids();
		int twistTest(int i,int j,int k);
		void cleanStartObs(Eigen::Vector3d _start_pt);

		void initGridMap(double _resolution, Eigen::Vector3d global_xyz_l, Eigen::Vector3d global_xyz_u, 
						 int max_x_id, int max_y_id, int max_z_id, double _interval);
		void setObs(const double coord_x, const double coord_y, const double coord_z);
		void cleanObs();

		Eigen::Vector3d coordRounding(const Eigen::Vector3d & coord);
		std::vector<Eigen::Vector3d> getPath();
		// std::vector<Eigen::Vector3d> getTwist();
		std::vector<Eigen::Vector3d> getTwist2();
		std::vector<Eigen::Vector3d> getTwist3();
		std::vector<Eigen::Vector3d> getTwist4();
		std::vector<Eigen::Vector3d> getTwist_checkcolision(Eigen::Vector3d start_pt);
		std::vector<Eigen::Vector3d> getTwist_checkcolision2();
		std::vector<Eigen::Vector3d> getTwist_checkcolision3(Eigen::Vector3d start_pt);
		std::pair<bool,Eigen::Vector3i> check_collision(GridNodePtr3d start_ptr,GridNodePtr3d aim_ptr);
		std::pair<bool,Eigen::Vector3i> check_collision(Eigen::Vector3d start_ptr,Eigen::Vector3d aim_ptr);
		std::vector<Eigen::Vector3d> getVisitedNodes();


		Eigen::Vector3d start_pt_real;
};

#endif