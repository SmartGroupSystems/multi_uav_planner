<<<<<<< HEAD
## 说明

##### ！！建议下载without_timer分支里的代码，它删去了B样条函数里的定时器并且修复了代价函数的一些bug，仿真效果更好一些。main分支里的代码还存在一些bug。

### 编译

1. 将build与devel删除后进行编译(不小心传上去了QAQ)
2. 进入工作空间后直接 `catkin_make` 进行编译。
3. 如果编译过程报错，提示缺少.h文件，可以尝试以下方法：
   
     1)  多次运行 `catkin_make` 

     2)   如果仍然报错，可以尝试先运行`catkin_make -j1` 再运行 `catkin_make`
     
     3)  如果1）、2）都不行，将   devel 以及build再次删除，重复1）、2）

     4) 如果上述几种方法都失败了，则可以从分支里下载的devel文件中，从include的文件夹里的bspline_race以及multi_bspline_opt(这里可以根据报错选择文件夹)中将里面的.h文件复制到你自己编译的devel文件夹的对应位置

### 运行方法

#### 运行单架无人机

1. 运行static_planner.sh文件,设置2D nav goal 

#### 运行多架无人机

1.  可以运行simulate_uav里面的swarm_simulator.launch，也可以运行multi_planner.sh。两者的区别在于一个只开了一个终端（swarm_simulator.launch），一个打开了多个终端（multi_planner.sh，打开多个终端是为了方便调试时输出信息的查看），通过2D Nav goal启动。
   
2. 如果要修改无人机数量，建议直接从 swarm_simulator.launch修改，比较省事

3. 可以在launch文件里修改起点和终点
   
### 运行效果（主要展示多机）

![Alt Text](/pics/multi.gif) 


### 目前主要问题

1. 轨迹衔接不流畅

2. 多机避障不太灵敏
=======
##多机运行
1.试验单架无人机运行multi_planner.sh
2.运行多架无人机，运行simulate_uav里面的swarm_simulator.launch
3.多机代码主要为multi开头的代码
4.multi_mapping建图，multi_grid_path_searcher路径搜索，multi_bspline_opt为轨迹优化，simulate_uav为仿真运行相关
>>>>>>> 660a75c39eb17347837c1177d3d0593b121c5bbd
