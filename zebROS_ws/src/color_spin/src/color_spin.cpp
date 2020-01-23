#include "ros/ros.h"
#include "color_spin/color_algorithm.h"

bool rotate(color_spin::color_algorithm::Request &req,
		color_spin::color_algorithm::Response &res)
{
  if (req.sensor_color == 'r') {
	  if (req.fms_color == 'y') {
		  res.rotate = .125;
		}
	  else if (req.fms_color == 'c') {
		  res.rotate = .25;
		}
	  else if (req.fms_color == 'g') {
		  res.rotate = -.125;
		}
	  }
  else if (req.sensor_color == 'y') {
	  if (req.fms_color == 'r') {
		  res.rotate = -.125;
		}
	  else if (req.fms_color == 'c') {
		  res.rotate = .125;
		}
	  else if (req.fms_color == 'g') {
		  res.rotate = .25;
		}
	  }
  else if (req.sensor_color == 'c') {
	  if (req.fms_color == 'r') {
		  res.rotate = .25;
		}
	  else if (req.fms_color == 'y') {
		  res.rotate = -.125;
		}
	  else if (req.fms_color == 'g') {
		  res.rotate = -.125;
		}
	  }
  else if (req.sensor_color == 'g') {
	  if (req.fms_color == 'r') {
		  res.rotate = .125;
		}
	  else if (req.fms_color == 'y') {
		  res.rotate = .25;
		}
	  else if (req.fms_color == 'c') {
		  res.rotate = -.125;
		}
	  }
  ROS_INFO("request = x%1c, y=%1c", (char)req.fms_color, (char)req.sensor_color);
  ROS_INFO("sending response = [x%1f]", (float)res.rotate);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "color_spin_algorithm");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("color_spin_algorithm", rotate);
	ROS_INFO("Ready to calculate rotation");
	ros::spin();

	return 0;

}

