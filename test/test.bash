#!/bin/bash

dir=~
[ "$1" != "" ] && dir="$1"

cd $dir/ros2_ws
colcon build --packages-select cigarette_dispenser_robot

#ここにif文(エラーの時)
source $dir/.bashrc
#timeout 10 ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py > /tmp/mypkg.log
echo "start"
#ros2 run cigarette_dispenser_robot detection.py
ros2 launch cigarette_dispenser_robot detection_and_motion.launch.py use_sim_time:='true'

#cat /tmp/mypkg.log
