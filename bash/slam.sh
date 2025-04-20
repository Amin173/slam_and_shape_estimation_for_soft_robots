#!/usr/bin/env bash

source /opt/ros/overlay_ws/devel/setup.bash
export path="/opt/ros/overlay_ws/src/slam_and_shape_estimation_for_soft_robots/bags/maze4/2021-11-24_15h15/"
export parent=".."
ls $path$parent
export file="*-no-map.bag"
export bag_file=$(find $path$file)

roslaunch slam_and_shape_estimation_for_soft_robots rosbag_replay_set_init.launch

roslaunch slam_and_shape_estimation_for_soft_robots rosbag_replay_slam.launch