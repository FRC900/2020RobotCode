#include <iostream>
#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

#include "zedsvoin.hpp"
#include "zmsin.hpp"
#include "frameticker.hpp"

using namespace cv;
using namespace std;
using namespace sensor_msgs;


int main(int argc, char **argv)
{
	
	MediaIn *cap = NULL;

	string file_path;
	
	const char* video_file = file_path.c_str();

	cap = new ZMSIn(video_file);

	if (cap == NULL)
	{
		cerr << "Error creating input" << endl;
		return -1;
	}

	Mat image;
	Mat depth;
	FrameTicker frameTicker;
	while (cap->getFrame(image, depth))
	{
		frameTicker.mark();

		cv_bridge::CvImage rgb_out;

		try {
	        	rgb_out.encoding = sensor_msgs::image_encodings::BGR8;
			rgb_out.image = image;
		} catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}

		imshow ("Image", image);

		if ((uchar)waitKey(5) == 27)
			break;
	}
	return 0;
}
