#include "ros/ros.h"
#include "string"
#include "color_spin/color_algorithm.h"

bool rotate(color_spin::color_algorithm::Request &req,
		color_spin::color_algorithm::Response &res)
{

  req.sensor_color = toupper(req.sensor_color[0]);
  req.fms_color = toupper(req.fms_color[0]);
  /*
  if (req.sensor_color == "R") {
	  if (req.fms_color == "Y") {
		  res.rotate = .125;
		}
	  else if (req.fms_color == "C") {
		  res.rotate = .25;
		}
	  else if (req.fms_color == "G") {
		  res.rotate = -.125;
		}
	  }
  else if (req.sensor_color == "Y") {
	  if (req.fms_color == "R") {
		  res.rotate = -.125;
		}
	  else if (req.fms_color == "C") {
		  res.rotate = .125;
		}
	  else if (req.fms_color == "G") {
		  res.rotate = .25;
		}
	  }
  else if (req.sensor_color == "C") {
	  if (req.fms_color == "R") {
		  res.rotate = .25;
		}
	  else if (req.fms_color == "Y") {
		  res.rotate = -.125;
		}
	  else if (req.fms_color == "G") {
		  res.rotate = .125;
		}
	  }
  else if (req.sensor_color == "G") {
	  if (req.fms_color == "R") {
		  res.rotate = .125;
		}
	  else if (req.fms_color == "Y") {
		  res.rotate = .25;
		}
	  else if (req.fms_color == "C") {
		  res.rotate = -.125;
 		}
	  }
  else {
		  ROS_INFO("Invalid Sensor Color Input");
	   }*/

  switch(req.sensor_color[0])
  {
	  case 'R':
		 switch(req.fms_color[0])
		 {
			 case 'Y': res.rotate = .125;
					   break;
			 case 'C': res.rotate = .25;
					   break;
			 case 'G': res.rotate = -.125;
					   break;
			 case 'R': ROS_ERROR("FMS COLOR AND SENSOR COLOR ARE THE SAME");
					   res.rotate = .0;
					   break;
			 default: ROS_ERROR("FMS COLOR IS STRANGE");
					   res.rotate = .0;
					   break;
		 }
		 break;
	 case 'G':
		 switch(req.fms_color[0])
		 {
			 case 'R': res.rotate = .125;
					   break;
			 case 'C': res.rotate = -.125;
					   break;
			 case 'Y': res.rotate = .25;
					   break; 
			 case 'G': ROS_ERROR("FMS COLOR AND SENSOR COLOR ARE THE SAME");
					   res.rotate = .0;
					   break;
			 default: ROS_ERROR("FMS COLOR IS STRANGE");
					   res.rotate = .0;
					   break;
		 }
		 break;
	 case 'C':
		 switch(req.fms_color[0])
		 {
			 case 'R': res.rotate = .25;
					   break;
			 case 'G': res.rotate = .125;
					   break;
			 case 'Y': res.rotate = -.125;
					   break;
			 case 'C': ROS_ERROR("FMS COLOR AND SENSOR COLOR ARE THE SAME");
					   res.rotate = .0;
					   break;
			 default: ROS_ERROR("FMS COLOR IS STRANGE");
					   res.rotate = .0;
					   break;
		 }
		 break;
	 case 'Y':
		 switch(req.fms_color[0])
		 {
			 case 'R': res.rotate = -.125;
					   break;
			 case 'C': res.rotate = .125;
					   break;
			 case 'G': res.rotate = .25;
					   break;
			 case 'Y': ROS_ERROR("FMS COLOR AND SENSOR COLOR ARE THE SAME");
					   res.rotate = .0;
					   break;
			 default: ROS_ERROR("FMS COLOR IS STRANGE");
					   res.rotate = .0;
					   break;
		 }
		 break;
	 default: ROS_ERROR("SENSOR COLOR IS STRANGE");
		 break;
	}
  ROS_INFO("request = Sensor_Color=%1s, FMS_Color=%1s", req.sensor_color.c_str(), req.fms_color.c_str());
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

