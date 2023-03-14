#include "Astar_searcher_3d.h"
#include <iostream>
#include <fstream>
#include <math.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/point_cloud_conversion.h>
using namespace std;
using namespace Eigen;
bool tie_break = false;
// GridNodePtr3d firstPtr  = NULL;
void AstarPathFinder3d::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, 
                                    int max_x_id, int max_y_id, int max_z_id, double _interval)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    interval_ = _interval;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr3d ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr3d * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr3d [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode3d(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder3d::resetGrid(GridNodePtr3d ptr)
{
    //USAGE 将点ptr的id置0，即不在open/closedlist中
    ptr->id = 0;
    ptr->cameFrom = NULL;
    // ptr->twistFrom = NULL;
    // first_node_expanded = false;
    // firstPtr = NULL;
    // ptr->cameFrom_Slash = 0;
    ptr->gScore = inf;
    ptr->fScore = inf;
    ptr->obs_around = false;
}

void AstarPathFinder3d::resetUsedGrids()
{   
    for (auto i = usedPtrList.begin(); i < usedPtrList.end(); i++)
    {
        resetGrid(*(i));
    }
    usedPtrList.clear();
    
    // for(int i=0; i < GLX_SIZE ; i++)
    //     for(int j=0; j < GLY_SIZE ; j++)
    //         for(int k=0; k < GLZ_SIZE ; k++)
    //             resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder3d::setObs(const double coord_x,
                               const double coord_y,
                               const double coord_z)
{   
    if( coord_x <  gl_xl || coord_y <  gl_yl || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        {
            cout << coord_x << " " << coord_y << " "<< coord_z << endl;
            ROS_WARN("out range");
            return;//检测超越边界
        }
        
    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution); 

    //中心点
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;


    //膨胀一圈
    for(int i = -1;i <= 1;++i ){
        for(int j = -1;j <= 1;++j ){
            for(int k = -1;k <= 1;++k ){
                if( i == 0 && j == 0 && k == 0)
                    continue; // to avoid this node
                data[(idx_x+i) * GLYZ_SIZE + (idx_y+j) * GLZ_SIZE + idx_z+k] = 1;
            }
        }
    }
    

}
void AstarPathFinder3d::cleanObs()
{   
    // for(int i = 0;i<GLXYZ_SIZE;i++){
    //     data[i] = 0;
    // }
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}
void AstarPathFinder3d::cleanStartObs(Eigen::Vector3d _start_pt){
    Eigen::Vector3i pt;
    pt = coord2gridIndex(_start_pt);
    data[pt(0) * GLYZ_SIZE + pt(1) * GLZ_SIZE + pt(2)] = 1;
}

vector<Vector3d> AstarPathFinder3d::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++){   
            for(int k = 0; k < GLZ_SIZE; k++){   
                if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }
        }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder3d::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder3d::coord2gridIndex(const Vector3d & pt) 
{
    //USAGE 点云坐标数据转换为栅格地图坐标
    //
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);
    return idx;
}

Eigen::Vector3d AstarPathFinder3d::coordRounding(const Eigen::Vector3d & coord)
{
    //USAGE 从真实坐标到栅格地图再到真实坐标
    //从而使得可视化的地图为正正经经的方格
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder3d::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder3d::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder3d::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder3d::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

bool AstarPathFinder3d::getData(const Eigen::Vector3d & pt)
{
    return isOccupied(coord2gridIndex(pt));
}
bool AstarPathFinder3d::getData(const Eigen::Vector3i & pt)
{
    return isOccupied(pt);
}
bool AstarPathFinder3d::arrived(const Eigen::Vector3d & pt1,const Eigen::Vector3d & pt2)
{
    Vector3i pt_1 = coord2gridIndex(pt1);
    Vector3i pt_2 = coord2gridIndex(pt2);
    if(pt_1 == pt_2) return true;
    else return false;
}
bool AstarPathFinder3d::nearly_arrived(const Eigen::Vector3d & pt1,const Eigen::Vector3d & pt2,double threshold)
{
    if(Eigen::Vector3d(pt1 - pt2).norm()<threshold) return true;
    else return false;
}
//TODO 
//AstarGetSucc函数
// 获取该点周围的所有节点和周围点的edgeCostSets(edgeCostSets:该点到目标的的距离)
inline bool AstarPathFinder3d::AstarGetSucc(GridNodePtr3d currentPtr, vector<GridNodePtr3d> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear(); // Note: the pointers in this set copy pointers to GridNodeMap
    edgeCostSets.clear();
    /*
    *
    STEP 4: finish AstarPathFinder3d::AstarGetSucc yourself 
    please write your code below
    *
    */
    // idea index -> coordinate -> edgecost
    if(currentPtr == nullptr)
        std::cout << "Error: Current pointer is null!" << endl;


    Eigen::Vector3i thisNode = currentPtr -> index;
    int this_x = thisNode[0];
    int this_y = thisNode[1];
    int this_z = thisNode[2];
    auto this_coord = currentPtr -> coord;
    int  n_x, n_y, n_z;
    double dist;
    GridNodePtr3d temp_ptr = nullptr;
    Eigen::Vector3d n_coord;
    bool obs_around = false;

    // 遍历周围点,获取周围点的edgeCostSets
    for(int i = -1;i <= 1;++i ){
        for(int j = -1;j <= 1;++j ){
            for(int k = -1;k <= 1;++k ){
                if( i == 0 && j == 0 && k == 0)
                    continue; // to avoid this node

                n_x = this_x + i;
                n_y = this_y + j;
                n_z = this_z + k;

                if( (n_x < 0) || (n_x > (GLX_SIZE - 1)) 
                 || (n_y < 0) || (n_y > (GLY_SIZE - 1))
                 || (n_z < 0) || (n_z > (GLZ_SIZE - 1)) ) continue; // to avoid index problem

                if(isOccupied(n_x, n_y, n_z))
                {
                    obs_around = true;
                    continue; // to avoid obstacles
                }
                    

                // put the pointer into neighborPtrSets
                temp_ptr = GridNodeMap[n_x][n_y][n_z];

                if(temp_ptr->id == -1) 
                    continue; // TODO to check this; why the node can transversing the obstacles
                
                if(i*i+j*j+k*k != 1){
                    if(isOccupied(n_x+i, n_y, n_z)&&
                       isOccupied(n_x, n_y+j, n_z)&&
                       isOccupied(n_x, n_y, n_z+k)) continue;
                    
                }

                n_coord = temp_ptr->coord;

                if(temp_ptr == currentPtr){
                    std::cout << "Error: temp_ptr == currentPtr)" << std::endl;
                }

                if( (std::abs(n_coord[0] - this_coord[0]) < 1e-6) and 
                    (std::abs(n_coord[1] - this_coord[1]) < 1e-6) and 
                    (std::abs(n_coord[2] - this_coord[2]) < 1e-6) ){
                    std::cout << "Error: Not expanding correctly!" << std::endl;
                    std::cout << "n_coord:" << n_coord[0] << " "<<
                                               n_coord[1] <<" "<<
                                               n_coord[2] << std::endl;
                    std::cout << "this_coord:" << this_coord[0] << " "<<
                                                  this_coord[1] << " "<<
                                                  this_coord[2] << std::endl;

                    std::cout << "current node index:" << this_x << " "<< 
                                                          this_y << " "<< 
                                                          this_z << std::endl;
                    std::cout << "neighbor node index:" << n_x << " "<< 
                                                           n_y << " "<< 
                                                           n_z << std::endl;
                }
                dist = std::sqrt( (n_coord[0] - this_coord[0]) * (n_coord[0] - this_coord[0])+
                                  (n_coord[1] - this_coord[1]) * (n_coord[1] - this_coord[1])+
                                  (n_coord[2] - this_coord[2]) * (n_coord[2] - this_coord[2]));
                
                neighborPtrSets.push_back(temp_ptr); // calculate the cost in edgeCostSets: inf means that is not unexpanded
                edgeCostSets.push_back(dist); // put the cost inot edgeCostSets
            }

        }
    }
    return obs_around;
}

// 启发函数 getHeu
inline double AstarPathFinder3d::getHeu(GridNodePtr3d node1, GridNodePtr3d node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    *
    *
    *
    STEP 1: finish the AstarPathFinder3d::getHeu , which is the heuristic function
    please write your code below
    *
    *
    */
    double heu_dist = inf;
    auto node1_coord = node1->coord;
    auto node2_coord = node2->coord;

    // Heuristics 1: Manhattan
    // h = std::abs(node1_coord(0) - node2_coord(0) ) +
    //     std::abs(node1_coord(1) - node2_coord(1) ) +
    //     std::abs(node1_coord(2) - node2_coord(2) );

    // Heuristics 2: Euclidean
    // h = std::sqrt(std::pow((node1_coord(0) - node2_coord(0)), 2 ) +
    //     std::pow((node1_coord(1) - node2_coord(1)), 2 ));

    // Heuristics 3: Diagnol distance
    double dx = std::abs(node1_coord(0) - node2_coord(0) );
    double dy = std::abs(node1_coord(1) - node2_coord(1) );
    double dz = std::abs(node1_coord(2) - node2_coord(2) );
    enum HeuristicType
    {
        NO_HEURISTIC,
        MANHATTAN,
        EUCLIDEAN,
        DIAGONAL
    }type_;
    type_ = MANHATTAN;
    switch (type_)
    {
    case NO_HEURISTIC:
        heu_dist = 0;
        // std::cout << "NO_HEURISTIC............." << std::endl;
        break;

    case MANHATTAN:
        heu_dist = dx + dy + dz;
        // std::cout << "MANHATTAN............." << std::endl;
        break;

    case EUCLIDEAN:
        heu_dist = sqrt(dx * dx + dy * dy + dz * dz);
        // std::cout << "EUCLIDEAN............." << std::endl;
        break;

    case DIAGONAL:
        heu_dist = dx + dy + dz + (1.73205081 - 3) * fmin(dx,fmin(dy,dz))
                                + (1.41421356 - 2) * fmax(dx,fmax(dy,dz));
        // std::cout << "DIAGONAL............." << std::endl;
    default:
        break;
    }
    //这个tie_break目前还没搞懂什么意思
    if(tie_break){
        double p = 1.0 / 25.0;
        heu_dist *= (1.0 + p);
        //std::cout << "Tie Break!" << std::endl;
    }

    return heu_dist*1.5;
}



//TODO 寻找路径函数
// A*路径搜索
bool AstarPathFinder3d::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr3d startPtr = new GridNode3d(start_idx, start_pt);
    GridNodePtr3d endPtr   = new GridNode3d(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    //currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr3d currentPtr  = NULL;
    GridNodePtr3d neighborPtr = NULL;
    

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder3d::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    usedPtrList.push_back(startPtr); // QUES
    // make_pair的用法:无需写出型别,就可以生成一个pair对象;比如std::make_pair(42, '@'),而不必费力写成：std::pair<int, char>(42, '@')
    //todo Note: modified, insert the pointer GridNodeMap[i][j][k] to the start node in grid map
    openSet.insert( make_pair(startPtr -> fScore, startPtr) ); 
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    // three dimension pointer GridNodeMap[i][j][k] is pointed to a struct GridNode(Eigen::Vector3i _index, Eigen::Vector3d _coord);
    // assign g(xs) = 0, g(n) = inf (already done in initialzation of struct)
    // mark start point as visited(expanded) (id 0: no operation, id: 1 in OPEN, id -1: in CLOSE )

    // 这个到底需不需要???测试后发现不需要也可以实现功能
    // JS: 标识该结点已进入openset
    GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]] -> id = 1;
    usedPtrList.push_back(GridNodeMap[start_idx[0]][start_idx[1]][start_idx[2]]);

    vector<GridNodePtr3d> neighborPtrSets;
    vector<double> edgeCostSets;
    Eigen::Vector3i current_idx; // record the current index

    // this is the main loop
    while ( !openSet.empty() ){
        /*
        *
        *
        step 3: Remove the node with lowest cost function from open set to closed set
        please write your code below
        
        IMPORTANT NOTE!!!
        This part you should use the C++ STL: multimap, more details can be find in Homework description
        *
        *
        */
        //JS change
        
        // for (auto iter = openSet.begin(); iter != openSet.end(); ++iter) {
        //     cout << iter->first << " " << iter->second << endl;
        // }
        //JS change end


        // openset:待访问节点容器;closed set:访问过节点容器
        // openSet.begin()返回👈已经排好顺序的第一个键值对的双向迭代器
        // 此处xyz = openset中第一个结点的第二个键值（Ptr）的格子图坐标
        int x = openSet.begin()->second->index(0); 
        int y = openSet.begin()->second->index(1);  
        int z = openSet.begin()->second->index(2);
        openSet.erase(openSet.begin());//删除指定键值对
        currentPtr = GridNodeMap[x][y][z];//当前👈
        // if(!first_node_expanded){
        //     //使得第一个节点👈自己
        //     currentPtr->twistFrom = currentPtr;
        //     first_node_expanded = true;
        //     firstPtr = currentPtr;
        // }
        

        // 如果节点被访问过;则返回
        if(currentPtr->id == -1)
            continue;
        // 标记id为-1,表示节点已被扩展
        currentPtr->id = -1;
        usedPtrList.push_back(currentPtr);

        // currentPtr = openSet.begin() -> second; // first T1, second T2
        // openSet.erase(openSet.begin()); // remove the node with minimal f value
        // current_idx = currentPtr->index;
        // GridNodeMap[current_idx[0]][current_idx[1]][current_idx[2]] -> id = -1;// update the id in grid node map
        
        // if the current node is the goal 
        if( currentPtr->index == goalIdx )
        {
            // 到达目标点，退出循环
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            // terminatePtr->twistFrom = currentPtr->cameFrom;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return 1;
        }
            
        //get the succetion
        bool obs_around = AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder3d::AstarGetSucc yourself     
        currentPtr->obs_around = obs_around;
        // 获取该点周围的所有节点和周围点的edgeCostSets(edgeCostSets:该点到目标的的距离)


        /*
        *
        *
        STEP 5:  For all unexpanded neigbors "m" of node "n", please finish this for loop
        please write your code below
        *        
        */         
        for(int i = 0; i < (int)neighborPtrSets.size(); i++){
            //遍历邻居节点
            /*
            *
            *
            Judge if the neigbors have been expanded
            please write your code below
            
            IMPORTANT NOTE!!!
            neighborPtrSets[i]->id = -1 : expanded, equal to this node is in close set
            neighborPtrSets[i]->id = 1 : unexpanded, equal to this node is in open set
            *        
            */
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr -> id == 0){ //discover a new node, which is not in the closed set and open set
                /*
                *
                *
                STEP 6:  As for a new node, do what you need do ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                // shall update: gScore = inf; fScore = inf; cameFrom = NULL, id, mayby direction
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];//步数损失
                neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr,endPtr);//启发式
                neighborPtr->cameFrom = currentPtr; // todo shallow copy or deep copy
                // if(currentPtr->cameFrom_Slash!=neighborPtr->cameFrom_Slash){
                //     //若发生转折
                //     neighborPtr->twistFrom = currentPtr;
                // }
                // else{
                //     neighborPtr->twistFrom = currentPtr->twistFrom;
                // }

                // push node "m" into OPEN
                openSet.insert(make_pair(neighborPtr -> fScore, neighborPtr));
                neighborPtr -> id = 1;
                usedPtrList.push_back(neighborPtr);
                continue;
            }
            else if(neighborPtr -> id == 1){ //this node is in open set and need to judge if it needs to update, the "0" should be deleted when you are coding
                /*
                *
                *
                STEP 7:  As for a node in open set, update it , maintain the openset ,and then put neighbor in open set and record it
                please write your code below
                *        
                */
                // shall update: gScore; fScore; cameFrom, mayby direction
                if( neighborPtr->gScore > (currentPtr->gScore+edgeCostSets[i]))
                {//从当前节点currentPtr走更近一些
                    neighborPtr -> gScore = currentPtr -> gScore + edgeCostSets[i];
                    neighborPtr -> fScore = neighborPtr -> gScore + getHeu(neighborPtr,endPtr);
                    neighborPtr -> cameFrom = currentPtr;
                //     if(currentPtr->cameFrom_Slash!=neighborPtr->cameFrom_Slash){
                //     //若发生转折
                //     neighborPtr->twistFrom = currentPtr;
                // }
                // else{
                //     neighborPtr->twistFrom = currentPtr->twistFrom;
                // }
                
                }

                continue;
            }
            else{//this node is in closed set
                /*
                *
                please write your code below
                *        
                */
                // todo nothing to do here?
                continue;
            }
        }      
    }
    ros::Time time_2 = ros::Time::now();
    ROS_ERROR("Time consume in Astar path finding is %f.", (time_2 - time_1).toSec() );
    //if search fails
    ROS_ERROR("[Astar] No Grid need to be searched. Pathfinding process ended prematurely.");
    return 0;
}

// vector<Vector3d> AstarPathFinder3d::getTwist(){
//     vector<Vector3d> path;
//     vector<GridNodePtr3d> gridPath;

//     auto ptr = terminatePtr;
//     while(ptr -> twistFrom != firstPtr){
//         gridPath.push_back(ptr);
//         ptr = ptr->twistFrom;
//         //         thisNode = ptr -> index;
//         // twistTest()
//     }
//     gridPath.push_back(firstPtr);
//     for (auto ptr: gridPath)
//         path.push_back(ptr->coord);
        
//     reverse(path.begin(),path.end());

//     return path;
// }

vector<Vector3d> AstarPathFinder3d::getTwist2(){
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath;
    Eigen::Vector3i thisNode;
    Eigen::Vector3i lastNode;
    Eigen::Vector3i ori;
    int this_twist;
    int last_twist;
    bool twist_confirm;
    // gridPath.push_back(terminatePtr);    
    auto ptr = terminatePtr;
    auto ptr_ = terminatePtr;
    //放入终止点
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
    while(ptr -> cameFrom -> cameFrom != NULL){
        // gridPath.push_back(ptr);
        thisNode = ptr -> index;
        ptr_ = ptr->cameFrom;
        lastNode = ptr_ -> index;
        ori = thisNode - lastNode;
        this_twist = twistTest(ori[0],ori[1],ori[2]);
        // if(ptr!=NULL) lastNode = ptr -> index;
        // else break;
        twist_confirm = (last_twist != this_twist);
        if(twist_confirm) gridPath.push_back(ptr);
        last_twist = this_twist;
        ptr = ptr_;
    }
    // 放入起始点
    gridPath.push_back(ptr);
    // gridPath.push_back(ptr);
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());

    return path;
}

vector<Vector3d> AstarPathFinder3d::getTwist3(){
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath;
    Eigen::Vector3i thisNode;
    Eigen::Vector3i lastNode;
    Eigen::Vector3i ori;
    int this_twist;
    int last_twist;
    bool twist_confirm;
    // gridPath.push_back(terminatePtr);    
    auto ptr = terminatePtr;
    auto ptr_ = terminatePtr;
    //放入终止点
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
    int count_length = 1;
    if(ptr->cameFrom!=NULL){
        while(ptr -> cameFrom -> cameFrom != NULL){
            // gridPath.push_back(ptr);
            thisNode = ptr -> index;
            ptr_ = ptr->cameFrom;
            lastNode = ptr_ -> index;
            ori = thisNode - lastNode;
            this_twist = twistTest(ori[0],ori[1],ori[2]);
            // if(ptr!=NULL) lastNode = ptr -> index;
            // else break;
            twist_confirm = (last_twist != this_twist);
            if(twist_confirm || count_length == 3) 
            {
                gridPath.push_back(ptr);
                count_length = 0;
            }
            count_length++;
            last_twist = this_twist;
            ptr = ptr_;
        }
    }
    // 放入起始点
    // cout<<gridPath.size()<<endl;
    gridPath.push_back(ptr);
    // gridPath.push_back(ptr);
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);

    reverse(path.begin(),path.end());
    

    return path;
}

vector<Vector3d> AstarPathFinder3d::getTwist4(){
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath;
    Eigen::Vector3i thisNode;
    Eigen::Vector3i lastNode;
    Eigen::Vector3i ori;
    int this_twist;
    int last_twist;
    bool twist_confirm; 
    auto ptr = terminatePtr;
    auto ptr_ = terminatePtr;
    //放入终止点
    gridPath.push_back(ptr);
    ptr = ptr->cameFrom;
    int count_length = 1;
    if(ptr->cameFrom!=NULL){
        while(ptr -> cameFrom -> cameFrom != NULL){
            // gridPath.push_back(ptr);
            thisNode   = ptr -> index;
            ptr_       = ptr->cameFrom;
            lastNode   = ptr_ -> index;
            ori        = thisNode - lastNode;
            this_twist = twistTest(ori[0],ori[1],ori[2]);
            twist_confirm = (last_twist != this_twist);
            if(twist_confirm) 
            {
                gridPath.push_back(ptr);
                count_length = 0;
            }
            count_length++;
            last_twist = this_twist;
            ptr        = ptr_;
        }
    }
    // 放入起始点
    ptr->coord = start_pt_real;
    gridPath.push_back(ptr);
    reverse(gridPath.begin(),gridPath.end());
    // 重新计算

    // cout<<"[start]"<<endl;
    for(auto iter=gridPath.begin();iter!=gridPath.end()-1;iter++)
    {
        // cout<<"[into_iter]"<<endl;
        auto start_ptr = *iter;
        auto end_ptr   = *(iter + 1);
        Vector3d vector_s2e;
        Vector3d vector_online;
        double lengh_of_line;
        int count_insert;
            // cout<<"start_ptr====\n"<<start_ptr->coord<<"====\n"<<endl;

        vector_s2e = end_ptr->coord - start_ptr->coord;
        lengh_of_line = vector_s2e.norm();
        count_insert = (int)(lengh_of_line/(double)(0.999));
        // cout<<"~~~~"<<endl;
        // cout<<"~~"<<lengh_of_line<<"~~"<<endl;
        // cout<<"~~"<<count_insert<<"~~"<<endl;
        // cout<<"~~~~"<<endl;
        for(int i = 0;i<=count_insert;i++)
        {
            vector_online = start_ptr->coord + 1.0*i*vector_s2e/vector_s2e.norm();
            // cout<<"vector_online====\n"<<vector_online<<"====\n"<<endl;
            path.push_back(vector_online);
        }
    }
    auto end_ptr = *(gridPath.end()-1);
    path.push_back(end_ptr->coord);
    // reverse(path.begin(),path.end());
    return path;
}


vector<Vector3d> AstarPathFinder3d::getTwist_checkcolision(Eigen::Vector3d start_pt){
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath_;
    vector<GridNodePtr3d> gridPath;
    auto ptr = terminatePtr;
    // 轨迹缩减
    while(ptr -> cameFrom != NULL){
        gridPath_.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    reverse(gridPath_.begin(),gridPath_.end());
    
    cout <<"[debug] current target: "<< (*(gridPath_.end()-1))->coord<<endl;
    cout <<"[debug] last ptr: "<< ptr->coord<<endl;
    GridNode3d real_start_pt;
    real_start_pt = *(*(gridPath_.begin()));
    real_start_pt.coord = start_pt;
    gridPath.push_back(&real_start_pt);
    auto start_pt_ = *(gridPath_.begin());
    while(!gridPath_.empty())
    {
        int id = 0, longest_id = 0;
        Eigen::Vector3i longest_colipoint(start_pt_->index);
        for(auto iter = gridPath_.begin(); iter != gridPath_.end(); iter++)
        {
            auto gridnode_ptr = *(iter);
            std::pair<bool,Eigen::Vector3i> check_point = check_collision(gridnode_ptr,start_pt_);
            if(!check_point.first)
                longest_id = id;
            id++;
        }
        cout<<"\033[0m"<<endl;
        // rate_.sleep();
        cout << "[debug] current longest id: "<<longest_id<<endl;
        cout << "[debug] current length: "<<gridPath_.size()<<endl;
        auto iter = gridPath_.begin();
        iter = iter + longest_id;
        gridPath.push_back(*(iter));
        start_pt_ = *(iter);
        gridPath_.erase(gridPath_.begin(),iter+1);
    }
    cout <<"[debug] current set target: "<< (*(gridPath.end()-1))->coord<<endl;

    // 轨迹补全
    bool fk_flag = false;
    auto iter_head = gridPath.begin();
    int j = 0;
    for(auto iter = gridPath.begin();iter!=gridPath.end();iter++)
    {
        auto start_ptr = *(iter_head);
        auto end_ptr   = *(iter);
        if((*iter_head)->coord == (*iter)->coord) continue;
        else
        {
            double lengh_of_line;
            int count_insert;
            Vector3d vector_s2e = (end_ptr->coord - start_ptr->coord);
            lengh_of_line = vector_s2e.norm();
            count_insert = (int)(lengh_of_line/(double)(interval_));
            for(int i = 0;i<=count_insert;i++)
            {
                Vector3d vector_online = start_ptr->coord + interval_*i*vector_s2e/vector_s2e.norm();
                // cout << "line[ "<< j <<"], number [ "<< i << "]:\n"<<vector_online<<endl;
                
                path.push_back(vector_online);
            }
            j++;
            iter_head = iter;
        }
    }
    path.push_back((*(gridPath.end()-1))->coord);
    return path;
}

vector<Vector3d> AstarPathFinder3d::getTwist_checkcolision2(){
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath_;
    vector<GridNodePtr3d> gridPath;
    auto ptr = terminatePtr;
    // 轨迹缩减
    cout <<"--------------getTwist_checkcolision---------------"<<endl;
    gridPath_.push_back(ptr);
    while(ptr -> cameFrom != NULL){
        if(ptr -> obs_around)
            gridPath_.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    reverse(gridPath_.begin(),gridPath_.end());
    gridPath.push_back(*(gridPath_.begin()));
    auto start_pt_ = *(gridPath_.begin());
    while(!gridPath_.empty())
    {
        int id = 0, longest_id = 0;
        Eigen::Vector3i longest_colipoint(start_pt_->index);
        for(auto iter = gridPath_.begin(); iter != gridPath_.end(); iter++)
        {
            auto gridnode_ptr = *(iter);
            std::pair<bool,Eigen::Vector3i> check_point = check_collision(gridnode_ptr,start_pt_);
            if(!check_point.first)
                longest_id = id;
            id++;
        }
        cout<<"\033[0m"<<endl;
        // rate_.sleep();
        auto iter = gridPath_.begin();
        iter = iter + longest_id;
        gridPath.push_back(*(iter));
        cout << "[debug] current tunning coord: "<<(*(iter))->coord<<endl;
        start_pt_ = *(iter);
        gridPath_.erase(gridPath_.begin(),iter+1);
    }

    // 轨迹补全
    bool fk_flag = false;
    auto iter_head = gridPath.begin();
    int j = 0;
    for(auto iter = gridPath.begin();iter!=gridPath.end();iter++)
    {
        auto start_ptr = *(iter_head);
        auto end_ptr   = *(iter);
        if((*iter_head)->coord == (*iter)->coord) continue;
        else
        {
            double lengh_of_line;
            int count_insert;
            Vector3d vector_s2e = (end_ptr->coord - start_ptr->coord);
            lengh_of_line = vector_s2e.norm();
            count_insert = (int)(lengh_of_line/(double)(interval_));
            for(int i = 0;i<=count_insert;i++)
            {
                Vector3d vector_online = start_ptr->coord + interval_*i*vector_s2e/vector_s2e.norm();
                // cout << "line[ "<< j <<"], number [ "<< i << "]:\n"<<vector_online<<endl;
                
                path.push_back(vector_online);
            }
            j++;
            iter_head = iter;
        }
    }
    return path;
}

vector<Vector3d> AstarPathFinder3d::getTwist_checkcolision3(Eigen::Vector3d start_pt){
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath_;
    vector<GridNodePtr3d> gridPath;
    auto ptr = terminatePtr;
    // 轨迹缩减
    while(ptr -> cameFrom != NULL){
        gridPath_.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    reverse(gridPath_.begin(),gridPath_.end());
    
    cout <<"[debug] current target: "<< (*(gridPath_.end()-1))->coord<<endl;
    cout <<"[debug] last ptr: "<< ptr->coord<<endl;
    GridNode3d real_start_pt;
    real_start_pt = *(*(gridPath_.begin()));
    real_start_pt.coord = start_pt;
    gridPath.push_back(&real_start_pt);
    auto start_pt_ = *(gridPath_.begin());
    while(!gridPath_.empty())
    {
        int id = 0, longest_id = 0;
        Eigen::Vector3i longest_colipoint(start_pt_->index);
        for(auto iter = gridPath_.begin(); iter != gridPath_.end(); iter++)
        {
            auto gridnode_ptr = *(iter);
            std::pair<bool,Eigen::Vector3i> check_point = check_collision(gridnode_ptr,start_pt_);
            if(!check_point.first)
                longest_id = id;
            id++;
        }
        cout<<"\033[0m"<<endl;
        // rate_.sleep();
        // cout << "[debug] current length: "<<gridPath_.size()<<endl;
        auto iter = gridPath_.begin();
        iter = iter + longest_id;
        cout << "[debug] current target coord: "<< (*(iter))->coord <<endl;
        gridPath.push_back(*(iter));
        start_pt_ = *(iter);
        gridPath_.erase(gridPath_.begin(),iter+1);
    }
    // cout <<"[debug] current set target: "<< (*(gridPath.end()-1))->coord<<endl;

    // 轨迹补全
    bool fk_flag = false;
    double last_remain_length = interval_;// 上一段轨迹残余长度

    auto iter_head = gridPath.begin();
    int j = 0;
    for(auto iter = gridPath.begin();iter!=gridPath.end();iter++)
    {
        auto start_ptr = *(iter_head);
        auto end_ptr   = *(iter);
        if((*iter_head)->coord == (*iter)->coord) continue;
        else
        {
            double   this_supp_length = interval_ - last_remain_length;
            Vector3d vector_s2e       = (end_ptr->coord - start_ptr->coord);  // 👈向量
            double lengh_of_line = vector_s2e.norm(); 
            if(lengh_of_line < this_supp_length)
            {
                last_remain_length = this_supp_length - lengh_of_line;
                continue;
            }
            Vector3d start_coord = start_ptr->coord +
                                   this_supp_length*vector_s2e/vector_s2e.norm();// 构建为当前位置加上一个初始向量长度
                       vector_s2e         = (end_ptr->coord - start_coord);
                       lengh_of_line      = vector_s2e.norm();                         // 向量长度
                   int count_insert       = (int)(lengh_of_line/(double)(interval_));
                       last_remain_length = lengh_of_line - interval_*count_insert;
            // cout << "\033[45m[debug] last_remain_length: " << last_remain_length << "\033[0m\n";

            for(int i = 0;i<=count_insert;i++)
            {
                Vector3d vector_online = start_coord + interval_*i*vector_s2e/vector_s2e.norm();
                path.push_back(vector_online);
            }
            j++;
            iter_head = iter;
        }
        if(iter == gridPath.end()-1)
        {
            path.push_back(end_ptr->coord);
        }
    }
    
    // if(!check_point.first)
    // {
    //     path.push_back((*(gridPath.end()-1))->coord);
    // }
    // else if(path.size() == 1) 
    // {
    //     path.push_back(*(path.end()-1));
    // }
    // cout << "\033[45m[debug] size of path: " << path.size() << "\033[0m\n";
    // path.push_back((*(gridPath.end()-1))->coord);
    return path;
}

std::pair<bool,Eigen::Vector3i> AstarPathFinder3d::check_collision(GridNodePtr3d aim_ptr,GridNodePtr3d start_ptr)
{
    Eigen::Vector3i vector_ori = aim_ptr->index - start_ptr->index;
    Eigen::Vector3i vector_abs = vector_ori.array().abs();
    Eigen::Vector3i vector     = Eigen::Vector3i::Zero();
    std::vector<int> vector_seq(3, 0);
    Eigen::Matrix3i R_ = Matrix3i::Zero(3,3);
    std::pair<bool,Eigen::Vector3i> end_idx(false, start_ptr->index);
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
        end_idx.second = Eigen::Vector3i(Eigen::Vector3i(next_idx.transpose()*R_.transpose()) + start_ptr->index);
        if(getData(end_idx.second))
        {
            cout << "\033[33m>";
            end_idx.first = true;
            break;
        }
    }
    if(!end_idx.first)
    {
        cout << "\033[37m>";
    }
    return end_idx;
}

std::pair<bool,Eigen::Vector3i> AstarPathFinder3d::check_collision(Eigen::Vector3d aim_ptr,Eigen::Vector3d start_ptr)
{
    Eigen::Vector3i vector_ori = coord2gridIndex(aim_ptr) - 
                                 coord2gridIndex(start_ptr);
    Eigen::Vector3i vector_abs = vector_ori.array().abs();
    Eigen::Vector3i vector     = Eigen::Vector3i::Zero();
    std::vector<int> vector_seq(3, 0);
    Eigen::Matrix3i R_ = Matrix3i::Zero(3,3);
    std::pair<bool,Eigen::Vector3i> end_idx(false, coord2gridIndex(start_ptr));
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
        Eigen::Vector3i real_next_idx = Eigen::Vector3i(Eigen::Vector3i(next_idx.transpose()*R_.transpose()) + coord2gridIndex(start_ptr));
        if(!getData(real_next_idx))
        {
            end_idx.first  = false;
            end_idx.second = real_next_idx;
        }
    }
    return end_idx;
}

// std::pair<bool,Eigen::Vector3i> AstarPathFinder3d::check_collision(GridNodePtr3d aim_ptr,GridNodePtr3d start_ptr)
// {
//     cout << "[CHECK] collision"<<endl;
//     Eigen::Vector3i start_idx = start_ptr->index;
//     Eigen::Vector3i aim_idx   = aim_ptr->index;
//     Eigen::Vector3i check_idx = aim_idx;
//     bool collision_happened = false;
//     std::pair<bool,Eigen::Vector3i> end_idx(collision_happened, start_idx);

//     int delta_x_ = abs(Eigen::Vector3i(aim_idx - start_idx).x()),
//         sx = Eigen::Vector3i(aim_idx - start_idx).x() > 0 ? 1 : -1;
//     int delta_y_ = abs(Eigen::Vector3i(aim_idx - start_idx).y()),
//         sy = Eigen::Vector3i(aim_idx - start_idx).y() > 0 ? 1 : -1;
//     double slope_ = double(delta_y_)/double(delta_x_);
//     int erro = (delta_x_ > delta_y_ ? delta_x_ : -delta_y_) / 2;

//     Eigen::Vector3i next_idx = check_idx;
//     ros::Rate rsss(1.0);
//     while(next_idx.x() != start_idx.x() && next_idx.y() != start_idx.y()) // 到起点截止
//     {
//         rsss.sleep();
//         int e2 = erro;
//         if(e2 > -delta_x_) { erro -= delta_y_; next_idx[0] += sx;}
//         if(e2 <  delta_y_) { erro += delta_x_; next_idx[1] += sy;}
//         cout << "next_idx" <<next_idx << endl;
//         if(!getData(next_idx))// 如果没发生碰撞
//         {
//             check_idx = next_idx;
//         }
//         else
//         {
//             ROS_WARN("collision_happened");
//             collision_happened = true;
//             break;
//         }
//     }
//     end_idx = std::pair<bool,Eigen::Vector3i>(collision_happened, check_idx);
//     return end_idx;

// }

//TODO 反向追溯得到路径
vector<Vector3d> AstarPathFinder3d::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr3d> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    auto ptr = terminatePtr;
    while(ptr -> cameFrom != NULL){
        gridPath.push_back(ptr);
        ptr = ptr->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());
    return path;
}

// if the difference of f is trivial, then choose then prefer the path along the straight line from start to goal
// discared!!!
// 目前这个函数我没有用到.
GridNodePtr3d & TieBreaker(const std::multimap<double, GridNodePtr3d> &  openSet, const GridNodePtr3d & endPtr)
{
    // todo do I have to update the f in openSet??
    std::multimap<double, GridNodePtr3d> local_set;

    auto f_min = openSet.begin()->first;
    auto f_max = f_min + 1e-2;
    auto itlow = openSet.lower_bound (f_min);
    auto itup = openSet.upper_bound(f_max);
    double cross, f_new;

    for (auto it=itlow; it!=itup; ++it)
    {
        std::cout << "f value is:" << (*it).first << " pointer is: " << (*it).second << '\n';
        cross = std::abs(endPtr->coord(0) - (*it).second->coord(0)) +
                std::abs(endPtr->coord(1) - (*it).second->coord(1));
        f_new = (*it).second->fScore + 0.001 * cross;
        local_set.insert( make_pair(f_new, (*it).second) ); // todo what is iterator, is this way correct?
    }


    return local_set.begin()->second;
}

int AstarPathFinder3d::twistTest(int i,int j,int k){
    // return 3*(i+1)+(j+1)+1;
    return 3*3*(k+1) + 3*(j+1) + i+1 + 1;
}