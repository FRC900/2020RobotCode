#include <iostream>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

	imshow("stream", image);
	waitKey(30);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
	ros::spin();
}
