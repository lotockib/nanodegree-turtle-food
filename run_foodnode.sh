#!/bin/bash

source /opt/ros/kinetic/setup.bash
dir=$PWD
eval "source $dir/devel/setup.bash"
# echo "$dir"
# echo "$(printenv | grep ROS)"

rosrun turtle_food food_node & 
wait