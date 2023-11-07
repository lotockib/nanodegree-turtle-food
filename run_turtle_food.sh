#!/bin/bash

# source
dir=$PWD
source /opt/ros/kinetic/setup.bash
eval "source $dir/devel/setup.bash"

# run program
roscore &
sleep 2
rosrun turtle_food turtlesim_node & 
rosrun turtle_food food_node "$@" &
sleep 1
bash $dir/start_teleop.sh
wait