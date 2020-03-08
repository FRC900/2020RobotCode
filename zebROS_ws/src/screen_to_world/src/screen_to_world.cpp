#include <iostream>
#include <string>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <goal_detection/Utilities.hpp>

#include <field_obj_tracker/convert_coords.h>
#include <field_obj/Object.h>
#include <field_obj/Detection.h>
#include <field_obj/TFObject.h>
#include <field_obj/TFDetection.h>

ros::Publisher pub;
sensor_msgs::CameraInfo camera_info_;

using namespace cv;

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info) {
	camera_info_ = *info;
}

float get_depth(const field_obj::TFObject object, const Mat &depth){
  const Rect mask_bound(object.tl.x, object.tl.y, object.br.x - object.tl.x, object.br.y - object.tl.y);
  static Mat mask(depth.rows, depth.cols, CV_8UC1, Scalar(0));
  rectangle(mask, mask_bound, Scalar(255), CV_FILLED, 8, 0);

  const float average_depth = zv_utils::avgOfDepthMat(depth, mask, mask_bound);
  return average_depth;
}

void screen_to_world_callback(const field_obj::TFDetection::ConstPtr &raw_msg, const sensor_msgs::ImageConstPtr &depth_msg) {
  cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);
  ConvertCoords cc(model);

  float distance;
  field_obj::Detection msg;
  for(field_obj::TFObject object : raw_msg -> objects){
    cv::Rect br(object.tl.x, object.tl.y, object.br.x - object.tl.x, object.br.y - object.tl.y);

    distance = get_depth(object, cvDepth->image);

    Point3f world_coord = cc.screen_to_world(br, object.label, distance);

    field_obj::Object dummy;

    dummy.location.x = world_coord.x;
    dummy.location.y = world_coord.y;
    dummy.location.z = world_coord.z;
    dummy.id = object.label;
    dummy.confidence = object.confidence;

    msg.objects.push_back(dummy);
  }
  pub.publish(msg);
}

void no_depth_callback(const field_obj::TFDetection::ConstPtr &raw_msg) {
  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(camera_info_);
  ConvertCoords cc(model);

  field_obj::Detection msg;
  for(field_obj::TFObject object : raw_msg -> objects){
    cv::Rect br(object.tl.x, object.tl.y, object.br.x - object.tl.x, object.br.y - object.tl.y);

    Point3f world_coord = cc.screen_to_world(br, object.label, 1.0);

    field_obj::Object dummy;

    dummy.location.x = world_coord.x;
    dummy.location.y = world_coord.y;
    dummy.location.z = world_coord.z;
    dummy.id = object.label;
    dummy.confidence = object.confidence;

    msg.objects.push_back(dummy);
  }
  pub.publish(msg);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "screen_to_world_node");
  ros::NodeHandle nh_;
  std::string sub_topic, camera_info_topic, depth_topic, pub_topic;
  bool no_depth = false;

  if (!nh_.getParam("sub_topic", sub_topic)) {
    ROS_ERROR("sub topic not specified");
    return -1;
  }

  if (!nh_.getParam("camera_info_topic", camera_info_topic)) {
    ROS_ERROR("camera info topic not specified");
    return -1;
  }

  if (!nh_.getParam("depth_topic", depth_topic)) {
    ROS_WARN("depth topic not specified");
    no_depth = true;
  }

  if (!nh_.getParam("pub_topic", pub_topic)) {
    ROS_ERROR("pub topic not specified");
    return -1;
  }

  pub = nh_.advertise<field_obj::Detection>(pub_topic, 1);
  ros::Subscriber sub_camera_info = nh_.subscribe(camera_info_topic, 1, camera_info_callback);

  if(!no_depth){
    message_filters::Subscriber<field_obj::TFDetection> tf_sub(nh_, sub_topic, 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh_, depth_topic, 1);

    typedef message_filters::sync_policies::ExactTime<field_obj::TFDetection, sensor_msgs::Image> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), tf_sub, depth_sub);
    sync.registerCallback(boost::bind(&screen_to_world_callback, _1, _2));
  } else {
    ros::Subscriber sub_tf_detection = nh_.subscribe(sub_topic, 1, no_depth_callback);
  }

  ros::spin();
  return 0;
}
