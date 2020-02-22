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
#include "Utilities.hpp"

#include <field_obj/Object.h>
#include <field_obj/Detection.h>
#include <field_obj/TFObject.h>
#include <field_obj/TFDetection.h>

ros::Publisher pub;
image_geometry::PinholeCameraModel model_;
sensor_msgs::CameraInfo camera_info_;

using namespace cv;

cv::Point3f convert_to_world(cv::Point2f uv, float distance){
  const Point3f world_coord_unit = model_.projectPixelTo3dRay(uv);
  //const float distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
  const Point3f world_coord_scaled = world_coord_unit * distance;
  const Point3f adj_world_coord_scaled(world_coord_scaled.x, -world_coord_scaled.y, world_coord_scaled.z);

  return adj_world_coord_scaled;
}

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

  float distance;
  field_obj::Detection msg;
  for(field_obj::TFObject object : raw_msg -> objects){
    double screen_x = (object.tl.x + object.br.x) / 2;
    double screen_y = (object.tl.y + object.br.y) / 2;

    distance = get_depth(object, cvDepth->image);

    Point2f screen_coord(screen_x, screen_y);
    Point3f world_coord = convert_to_world(screen_coord, distance);

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

  pub = nh_.advertise<field_obj::Detection>(pub_topic, 1);
  ros::Subscriber sub_camera_info = nh_.subscribe(camera_info_topic, 1, camera_info_callback);

  message_filters::Subscriber<field_obj::TFDetection> tf_sub(nh_, sub_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh_, depth_topic, 1);

  typedef message_filters::sync_policies::ExactTime<field_obj::TFDetection, sensor_msgs::Image> MySyncPolicy;
  // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), tf_sub, depth_sub);
  sync.registerCallback(boost::bind(&screen_to_world_callback, _1, _2));

  ros::spin();
  return 0;
}
