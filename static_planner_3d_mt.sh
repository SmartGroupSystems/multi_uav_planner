#! /usr/bin/env bash
set -e

RUN_AFTER_BASHRC="roslaunch so3_quadrotor_simulator simulator_example.launch" gnome-terminal --title="Simulator" --tab &
sleep 2;
RUN_AFTER_BASHRC="roslaunch dynamic_env_generator run_race.launch" gnome-terminal --title="Simulator" --tab &
sleep 2;
# RUN_AFTER_BASHRC="roslaunch mapping mapping.launch" gnome-terminal --title="Local Mapping" --tab &
# sleep 2 ;
RUN_AFTER_BASHRC="roslaunch grid_path_searcher astar_node_3d_mt.launch" gnome-terminal --title="Astar" --tab & 
sleep 2 ;
RUN_AFTER_BASHRC="roslaunch bspline_race traj_testing_sim.launch" gnome-terminal --title="Bspline" --tab &
sleep 2;
RUN_AFTER_BASHRC="rosrun so3_control control_bspline" gnome-terminal --title="controller" --tab;
# RUN_AFTER_BASHRC="rosrun plotjuggler plotjuggler show.xml" gnome-terminal --title="plot" --tab;
wait
exit 0
