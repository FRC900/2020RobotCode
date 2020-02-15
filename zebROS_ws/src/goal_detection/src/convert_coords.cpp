#include <iostream>

#include "convert_coords.h"
#include <ros/ros.h>

ConvertCoords::ConvertCoords(image_geometry::PinholeCameraModel model) {
  model_ = model;
}

cv::Point3f ConvertCoords::screen_to_world( const cv::Rect &bounding_rect, const std::string &debug_name, const float depth ) const {
  // Center point of left and right bounding rect
  const cv::Point2f uv(
      bounding_rect.tl().x + bounding_rect.width  / 2.0,
      bounding_rect.tl().y + bounding_rect.height / 2.0
      );
  const cv::Point3f world_coord_unit = model_.projectPixelTo3dRay(uv);
  //const float distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
  const cv::Point3f world_coord_scaled = world_coord_unit * depth;

  const cv::Point3f adj_world_coord_scaled(world_coord_scaled.x, world_coord_scaled.z, -world_coord_scaled.y);

#if 0
  ROS_INFO_STREAM("bounding_rect:" << bounding_rect);
  ROS_INFO_STREAM("uv:" << uv);
  ROS_INFO_STREAM("world_coord_unit:" << world_coord_unit);
  ROS_INFO_STREAM("distance:" << distance);
  ROS_INFO_STREAM("world_coord_scaled:" << world_coord_scaled);
  ROS_INFO_STREAM("pos:" << pos);
  ROS_INFO_STREAM("adj_world_coord_scaled:" << adj_world_coord_scaled);
  ROS_INFO_STREAM("adj_distance:" << adj_world_coord_scaled - pos);
  ROS_INFO_STREAM(debug_name << " : uv : " << uv << " gd_pos : " << pos << " model_pos : " << adj_world_coord_scaled << " difference " << adj_world_coord_scaled - pos);
#endif

  return adj_world_coord_scaled;
}

cv::Point3f ConvertCoords::screen_to_world( const cv::Point2f &uv ) const {
  const cv::Point3f world_coord_unit = model_.projectPixelTo3dRay(uv);
  const cv::Point3f adj_world_coord_scaled(world_coord_unit.x, world_coord_unit.z, -world_coord_unit.y);

  return adj_world_coord_scaled;
}

cv::Point2f ConvertCoords::world_to_screen( const cv::Point3f &pos, const std::string &debug_name ) const {
  const cv::Point2f screen_coord = model_.project3dToPixel(pos);

  return screen_coord;
}

double ConvertCoords::angle_from_center( const cv::Point3f &pos, const cv::Point3f &center) const {
  cv::Point2f v1(pos.y, pos.x);
  cv::Point2f v2(center.y, center.x);

  double cos_theta = v1.dot(v2) / (cv::norm(v1) * cv::norm(v2));
    if (cos_theta > 1.0)
        return 0.0;
    else if (cos_theta < -1.0)
        return CV_PI;
    return std::acos(cos_theta);
}
