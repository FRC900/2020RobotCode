#pragma once

#include <vector>
#include "field_obj_tracker/opencv2_3_shim.hpp"
#include <image_geometry/pinhole_camera_model.h>

enum ObjectNum
{
    UNINITIALIZED,
    BALL_2017,
    BIN_2016,
    GOAL_2016,
    TOP_TAPE_2017,
    BOTTOM_TAPE_2017,
    SWITCH_2018,
    CUBE_2018,
    LEFT_CARGO_2019,
    RIGHT_CARGO_2019,
	POWER_PORT_2020,
	LOADING_BAY_2020,
	TEST_TARGET_2020,
	power_cell,
	red_power_port_high_goal,
	blue_power_port_high_goal,
	red_power_port_low_goal,
	blue_power_port_low_goal,
	power_port_yellow_graphics,
	red_power_port_first_logo,
	blue_power_port_first_logo,
	red_loading_bay_tape,
	blue_loading_bay_tape,
	red_loading_bay_left_graphics,
	red_loading_bay_right_graphics,
	blue_loading_bay_left_graphics,
	blue_loading_bay_right_graphics,
	red_tape_corner,
	blue_tape_corner,
	red_ds_light,
	blue_ds_light,
	ds_light,
	control_panel_light,
	yellow_control_panel_light,
	shield_generator_light,
	red_shield_generator_light,
	blue_shield_generator_light,
	shield_generator_backstop,
	shield_generator_first_logo,
	shield_generator_yellow_stripe,
	shield_generator_floor_center_intersection,
	red_black_shield_generator_floor_intersection,
	blue_black_shield_generator_floor_intersection,
	red_blue_black_shield_generator_floor_intersection,
	red_shield_pillar_intersection,
	blue_shield_pillar_intersection,
	ds_numbers,
	control_panel,
	red_robot,
	blue_robot
};

//class to hold the type of object that a detected object is
//main information is the contour that is the shape of the object
//stores properties to make matching easy
class ObjectType
{
	public:
		//in this constructor there are contours prebuilt for game objects
		//1 - ball (2016 Game)
		//2 - bin (2015 Game)

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
		enum DepthSampleLocation { UNIFORM, CENTER, EDGES, };

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
		std::vector<std::pair<float, float> > positions_;
		DepthSampleLocation depth_sample_location_;

		//called by constructor to compute properties
		void computeProperties(void);
};
