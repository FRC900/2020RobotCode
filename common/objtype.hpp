#pragma once

#include <vector>
#include "opencv2_3_shim.hpp"

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
	POWER_PORT_2020, //added by me
	LOADING_BAY_2020, //added by me
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
		cv::Point3f screenToWorldCoords(const cv::Rect &screen_position, double avg_depth, const cv::Point2f &fov_size, const cv::Size &frame_size, float cameraElevation) const;

		cv::Rect worldToScreenCoords(const cv::Point3f &_position, const cv::Point2f &fov_size, const cv::Size &frame_size, float cameraElevation) const;

		float expectedDepth(const cv::Rect &screen_position, const cv::Size &frame_size, const float hfov) const;

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

		//called by constructor to compute properties
		void computeProperties(void);
};

