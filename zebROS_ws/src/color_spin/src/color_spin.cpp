#include "ros/ros.h"
#include "color_spin/color_algorithm.h"

color_spin::color_algorithm::Request &req,
color_spin::color_algorithm::Response &res
{
  if (sensor_color == 'r') {
	  if (fms_color == 'y') {
		  rotate = .125;
		}
	  else if (fms_color == 'c') {
		  rotate = .25;
		}
	  else if (fms_color == 'g') {
		  rotate = -.125;
		}
	  }
  else if (sensor_color == 'y') {
	  if (fms_color == 'r') {
		  rotate = -.125;
		}
	  else if (fms_color == 'c') {
		  rotate = .125;
		}
	  else if (fms_color == 'g') {
		  rotate = .25;
		}
	  }
  else if (sensor_color == 'c') {
	  if (fms_color == 'r') {
		  rotate = .25;
		}
	  else if (fms_color == 'y') {
		  rotate = -.125;
		}
	  else if (fms_color == 'g') {
		  rotate = -.125;
		}
	  }
  else if (sensor_color == 'g') {
	  if (fms_color == 'r') {
		  rotate = .125;
		}
	  else if (fms_color == 'y') {
		  rotate = .25;
		}
	  else if (fms_color == 'c') {
		  rotate = -.125
  ROS_INFO("request = x%1d, y=%1d", (char)req.fms_color, (char)req.sensor_color.b);
  ROS_INFO("sending response = [x%1d]", (float)res.rotate);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "color_spin_rotation");
	ros::NodeHandle nh;
	ros::ServiceServer service = n.advertiseService("color_spin_rotation", rotate);
	ROS_INFO("Ready to calculate rotation");
	ros::spin();

	return 0;

}

