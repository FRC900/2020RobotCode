#!/usr/bin/env bash

# Setup ROS for Local Development
source /opt/ros/melodic/setup.bash
source /home/ubuntu/2019Offseason/zebROS_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:5802
export ROS_IP=127.0.1
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

exec "$@"
