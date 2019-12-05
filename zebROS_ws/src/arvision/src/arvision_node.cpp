#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_stream_node");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    ros::Rate loop_rate(5);

    sensor_msgs::ImagePtr msg;
    std_msgs::Header header;

    VideoCapture cap;
    if(!cap.open(0))
    {
        ROS_INFO("CANNOT OPEN VIDEOSTREAM");
        return 0;
    }

    Mat frame;
    while(ros::ok)
    {
        cap >> frame;
        if (!frame.empty())
        {
            msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            pub.publish(msg);
            waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}