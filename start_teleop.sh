#!/bin/bash

source /opt/ros/kinetic/setup.bash
dir=$PWD
eval "source $dir/devel/setup.bash"
rosrun turtle_food turtle_teleop_key