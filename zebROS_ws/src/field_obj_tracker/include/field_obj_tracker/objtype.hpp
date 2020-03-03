#pragma once

#include <vector>
#include "field_obj_tracker/opencv2_3_shim.hpp"
#include <image_geometry/pinhole_camera_model.h>

enum ObjectNum
{
  UNINITIALIZED,
	POWER_PORT_2020,
	LOADING_BAY_2020,
	TEST_TARGET_2020,
	POWER_CELL,
	RED_POWER_PORT_HIGH_GOAL,
	BLUE_POWER_PORT_HIGH_GOAL,
	RED_POWER_PORT_LOW_GOAL,
	BLUE_POWER_PORT_LOW_GOAL,
	POWER_PORT_YELLOW_GRAPHICS,
	RED_POWER_PORT_FIRST_LOGO,
	BLUE_POWER_PORT_FIRST_LOGO,
	RED_LOADING_BAY_TAPE,
	BLUE_LOADING_BAY_TAPE,
	RED_LOADING_BAY_LEFT_GRAPHICS,
	RED_LOADING_BAY_RIGHT_GRAPHICS,
	BLUE_LOADING_BAY_LEFT_GRAPHICS,
	BLUE_LOADING_BAY_RIGHT_GRAPHICS,
	RED_TAPE_CORNER,
	BLUE_TAPE_CORNER,
	RED_DS_LIGHT,
	BLUE_DS_LIGHT,
	DS_LIGHT,
	CONTROL_PANEL_LIGHT,
	YELLOW_CONTROL_PANEL_LIGHT,
	SHIELD_GENERATOR_LIGHT,
	RED_SHIELD_GENERATOR_LIGHT,
	BLUE_SHIELD_GENERATOR_LIGHT,
	SHIELD_GENERATOR_BACKSTOP,
	SHIELD_GENERATOR_FIRST_LOGO,
	SHIELD_GENERATOR_YELLOW_STRIPE,
	SHIELD_GENERATOR_FLOOR_CENTER_INTERSECTION,
	RED_BLACK_SHIELD_GENERATOR_FLOOR_INTERSECTION,
	BLUE_BLACK_SHIELD_GENERATOR_FLOOR_INTERSECTION,
	RED_BLUE_BLACK_SHIELD_GENERATOR_FLOOR_INTERSECTION,
	RED_SHIELD_PILLAR_INTERSECTION,
	BLUE_SHIELD_PILLAR_INTERSECTION,
	DS_NUMBERS,
	CONTROL_PANEL,
	RED_ROBOT,
	BLUE_ROBOT
};

enum DepthSampleLocation
{
  UNIFORM,
  CENTER,
  EDGES,
  CORNERS,
  TAPE
};

//class to hold the type of object that a detected object is
//main information is the contour that is the shape of the object
//stores properties to make matching easy
class ObjectType
{
	public:
		//in this constructor there are contours prebuilt for game objects

		ObjectType(ObjectNum contour_type_id);

		//this constructor takes a custom contour
		ObjectType(const std::vector< cv::Point2f > &contour_in, const std::string &name_in, const float &depth_in);
		ObjectType(const std::vector< cv::Point > &contour_in, const std::string &name_in, const float &depth_in);

		//get the contour associated with the object type.
		// Useful for shape comparison
		std::vector< cv::Point2f > shape (void) const { return contour_; }

		//get physical characteristics
		cv::Point2f com (void) const { return com_; }
		float width (void) const {return width_; }
		float height (void) const {return height_; }
		float depth (void) const {return depth_; }
		float area (void) const { return area_; }
		float boundingArea (void) const { return width_ * height_; }
		std::string name (void) const { return name_; }
		float real_height (void) const { return real_height_; }

		//comparison operator overload just checks if the contours are equal
		bool operator== (const ObjectType &t1) const;

		//coordinate conversions
		cv::Point3f screenToWorldCoords(const cv::Rect &screen_position, double avg_depth, const image_geometry::PinholeCameraModel &model) const;

		cv::Rect worldToScreenCoords(const cv::Point3f &_position, const image_geometry::PinholeCameraModel &model) const;

		float expectedDepth(const cv::Rect &screen_position, const image_geometry::PinholeCameraModel &model) const;

	private:
		std::vector< cv::Point2f > contour_;

		// properties are computed and stored internally so that they
		// don't have to be recomputed every time the get functions are called
		float width_;
		float height_;
		float depth_;
		float area_;
		float real_height_ = 0;
		std::string name_;
		cv::Point2f com_; //center of mass

    // properties for object
		std::vector<cv::Point3f> positions_;
		DepthSampleLocation depth_sample_location_;

		//called by constructor to compute properties
		void computeProperties(void);
};
