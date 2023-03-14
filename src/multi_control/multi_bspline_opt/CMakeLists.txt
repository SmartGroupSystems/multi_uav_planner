cmake_minimum_required(VERSION 3.0.2)
project(multi_bspline_opt)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  message_generation
  nlopt
  message_filters
  multi_mapping
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
#add_dependencies(bspline_race ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_message_files(
  FILES
  BsplineTraj.msg  
  MultiBsplines.msg
  SendTraj.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES multi_bspline_opt
  CATKIN_DEPENDS roscpp rospy std_msgs visualization_msgs message_runtime
  #DEPENDS system_lib
)
include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
)
add_library( multi_bspline_opt 
   src/bspline_opt.cpp 
   )
   
add_executable( multi_flag_planner 
    src/test_planning.cpp
 )
target_link_libraries(multi_flag_planner 
multi_bspline_opt 
 ${catkin_LIBRARIES} 
)

