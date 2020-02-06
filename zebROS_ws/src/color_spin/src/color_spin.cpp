#include "ros/ros.h"
#include "string"
#include "color_spin/color_algorithm.h"

bool rotate(color_spin::color_algorithm::Request &req,
		color_spin::color_algorithm::Response &res)
{
  if (req.sensor_color == "r") {
	  if (req.fms_color == "y") {
		  res.rotate = .125;
		}
	  else if (req.fms_color == "c") {
		  res.rotate = .25;
		}
	  else if (req.fms_color == "g") {
		  res.rotate = -.125;
		}
	  }
  else if (req.sensor_color == "y") {
	  if (req.fms_color == "r") {
		  res.rotate = -.125;
		}
	  else if (req.fms_color == "c") {
		  res.rotate = .125;
		}
	  else if (req.fms_color == "g") {
		  res.rotate = .25;
		}
	  }
  else if (req.sensor_color == "c") {
	  if (req.fms_color == "r") {
		  res.rotate = .25;
		}
	  else if (req.fms_color == "y") {
		  res.rotate = -.125;
		}
	  else if (req.fms_color == "g") {
		  res.rotate = -.125;
		}
	  }
  else if (req.sensor_color == "g") {
	  if (req.fms_color == "r") {
		  res.rotate = .125;
		}
	  else if (req.fms_color == "y") {
		  res.rotate = .25;
		}
	  else if (req.fms_color == "c") {
		  res.rotate = -.125;
		}
	  }
  ROS_INFO("request = x=%1s, y=%1s", req.sensor_color.c_str(), req.fms_color.c_str());
  ROS_INFO("sending response = [x%1f]", (float)res.rotate);
  return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "color_algorithm");
	ros::NodeHandle nh;
	ros::ServiceServer service = nh.advertiseService("color_algorithm", rotate);
	ROS_INFO("Ready to calculate rotation");
	ros::spin();

	return 0;

}

