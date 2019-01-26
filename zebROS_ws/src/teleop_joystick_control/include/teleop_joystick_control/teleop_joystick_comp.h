#pragma once

#include "ros/ros.h"
#include "ros_control_boilerplate/JoystickState.h"
#include "frc_msgs/MatchSpecificData.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include <string>
#include <cmath>
#include <math.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float64.h>

void navXCallback(const sensor_msgs::Imu &navXState);

/*realtime_tools::RealtimeBuffer<struct> joystick_state;*/

