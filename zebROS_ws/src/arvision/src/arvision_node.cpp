#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace cv;

int main(int argc, char** argv)
{
    VideoCapture cap;
    if(!cap.open(0))
    {
        ROS_INFO("CANNOT OPEN VIDEOSTREAM");
        return 0;
    }
    for(;;)
    {
        Mat frame;
        cap >> frame;
        if ( frame.empty() ) break;
        imshow("this is you, smile", 255 - frame);
        if( waitKey(10) == 27) break;

    }
    return 0;
}