#!/bin/bash

cd ~/2020RobotCode/zebROS_ws/

if [ -z $ROS_ROOT ]; then
	PATH=$PATH:$HOME/wpilib/2020/roborio/bin
	source ~/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic/setup.bash
elif [[ ! $ROS_ROOT = "$HOME/wpilib/2020/roborio/arm-frc2020-linux-gnueabi/opt/ros/melodic/share/ros" ]]; then
	echo "ROS is not configured for a cross build (maybe set up for a native build instead?)"
	echo "Run ./cross_build.sh in a new terminal window"
	exit 1
fi

catkin config --profile cross -x _isolated --install --blacklist \
	demo_tf_node \
	field_obj_tracker \
	goal_detection \
	pf_localization \
	realsense2_camera \
	realsense2_description \
	robot_visualizer \
	rosbag_scripts \
	rospy_message_converter \
	rqt_driver_station_sim \
	stage_ros \
	template_controller \
	tf_object_detection \
	velocity_controllers \
	visualize_profile \
	zed_nodelets \
	zed_ros \
	zed_wrapper \
	zms_writer
catkin build --profile cross -DCMAKE_TOOLCHAIN_FILE=`pwd`/rostoolchain.cmake -DCATKIN_ENABLE_TESTING=OFF "$@"
