#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build --packages-select cigarette_dispenser_robot

if [ "$?" = 0 ]; then
source $dir/.bashrc
ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py use_sim_time:='true'

else
echo "build failed"
fi
