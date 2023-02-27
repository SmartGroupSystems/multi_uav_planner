#! /usr/bin/env bash
set -e

# RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch so3_quadrotor_simulator simulator_example.launch" gnome-terminal --title="Simulator" --tab &
# sleep 2;
# RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch mockamap post2d.launch" gnome-terminal --title="Simulator" --tab &
# sleep 2;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav map_build.launch" gnome-terminal --title="Global Map" --tab &
sleep 1 ;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav  mul_simulate.launch" gnome-terminal --title="Simulator" --tab &
sleep 1;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav  mul_state.launch" gnome-terminal --title="Odom" --tab &
sleep 1;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav  mul_map.launch" gnome-terminal --title="Local map" --tab &
sleep 1;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav  mul_a_star.launch" gnome-terminal --title="astar" --tab &
sleep 1;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav  mul_bspline.launch" gnome-terminal --title="bspline" --tab &
sleep 1;
RUN_AFTER_BASHRC="source devel/setup.bash;roslaunch simulate_uav  multi_rviz.launch" gnome-terminal --title="rviz" --tab 
# RUN_AFTER_BASHRC="source devel/setup.bash;rosrun plotjuggler plotjuggler show.xml" gnome-terminal --title="plot" --tab;
wait
exit 0
