#include <iostream>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <field_obj/Object.h>
#include <field_obj/Detection.h>

ros::Publisher pub;
image_geometry::PinholeCameraModel model_;
sensor_msgs::CameraInfo camera_info_;

cv::Point3f convert_to_world(cv::Point2f uv, float distance){
  const cv::Point3f world_coord_unit = model_.projectPixelTo3dRay(uv);
  //const float distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
  const cv::Point3f world_coord_scaled = world_coord_unit * distance;
  const cv::Point3f adj_world_coord_scaled(world_coord_scaled.x, -world_coord_scaled.y, world_coord_scaled.z);

  return adj_world_coord_scaled;
}

void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info) {
	camera_info_ = *info;
}

void screen_to_world_callback(field_obj::Detection::ConstPtr& raw_msg){
  float distance;
  field_obj::Detection msg;
  for(field_obj::Detection detection : raw_msg -> objects){
    screen_x = detection.location.x;
    screen_y = detection.location.y;

    cv::Point2f screen_coord(screen_x, screen_y);
    cv::Point3f world_coord = convert_to_world(screen_coord, distance)
    detection.location.x = world_coord.x;
    detection.location.y = world_coord.y;
    detection.location.z = world_coord.z;
  }
}

int main(int argc, char* argv) {
  ros::init(argc, argv, "screen_to_world_node");
  ros::NodeHandle nh_;

  model_.fromCameraInfo(camera_info_);

  pub = node.advertise<field_obj::Detection>(pub_topic, 1);
  ros::Subscriber sub_screen = nh_.subscribe(sub_topic, 1, screen_to_world_callback);
  ros::Subscriber sub_camera_info = nh_.subscribe("/zed_goal/left/camera_info", sub_rate, camera_info_callback);

  ros::spin();
  return 0;
}
