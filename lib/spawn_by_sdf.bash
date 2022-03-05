#!/bin/bash

path=/home/ych/ros2_ws/src/my_ros2_robot_gazebo/sdf/mrobot.sdf
ros2 run my_ros2_robot_gazebo spawn_by_sdf $path mrobot 0 0
