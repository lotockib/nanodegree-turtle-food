#!/bin/bash

source /opt/ros/kinetic/setup.bash
dir=$PWD
eval "source $dir/devel/setup.bash"
# echo "$dir"
# echo "$(printenv | grep ROS)"

roscore &
sleep 2
rosrun turtle_food turtlesim_node & 
rosrun turtle_food food_node "$@" &
wait

# roslaunch turtle_food sim.launch