#pragma once
//standard include
#include <math.h>
#include <iostream>

//opencv include
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <boost/circular_buffer.hpp>

#include "field_obj_tracker/track3d.hpp"

struct DepthInfo
{
	float depth;
	bool  error;
};

//this contains all the info we need to decide between goals once we are certain if it is a goal
struct GoalInfo
{
	float confidence;
	float distance;
	cv::Rect rect;
	size_t contour_index;
	bool depth_error;
	cv::Point com;
	cv::Rect br;
	cv::RotatedRect rtRect;
};

//This contains all the necessary info for a goal
struct GoalFound
{
	float distance;
	float confidence;
	size_t contour_index;
	cv::Rect rect;
	cv::RotatedRect rotated_rect;
	std::string id;
};


class GoalDetector
{
	public:
		GoalDetector(void);

		std::vector< GoalFound > return_found(void) const;

		void drawOnFrame(cv::Mat &image,const std::vector< std::vector< cv::Point>> &contours, const std::vector< GoalFound > &goals) const;

		//These are the three functions to call to run GoalDetector
		//they fill in _contours, _infos, _depth_mins, etc
		void clear(void);

		//If your objectypes have the same width it's safe to run
		//getContours and computeConfidences with different types
		void findTargets(const cv::Mat& image, const cv::Mat& depth, const ObjectNum objtype, const image_geometry::PinholeCameraModel &model);
		const std::vector< std::vector< cv::Point > > getContours(const cv::Mat& image);

		bool Valid(void) const;
		void setBlueScale(double blue_scale);
		void setRedScale(double red_scale);
		void setOtsuThreshold(int otsu_threshold);
		void setMinConfidence(double min_valid_confidence);

	private:
		// Save detection info
		bool        _isValid;
		std::vector< GoalFound > _return_found;
		float       _min_valid_confidence;

		int         _otsu_threshold;
		int         _blue_scale;
		int         _red_scale;

		float createConfidence(float expectedVal, float expectedStddev, float actualVal);
		//float distanceUsingFixedHeight(const cv::Rect &rect,const cv::Point &center, float expected_delta_height) const;
		bool generateThresholdAddSubtract(const cv::Mat& imageIn, cv::Mat& imageOut);
		void isValid();
		const std::string getObjectId(ObjectNum type) const;
		const std::vector<DepthInfo> getDepths(const cv::Mat &depth, const std::vector< std::vector< cv::Point > > &contours, const ObjectNum objtype, const image_geometry::PinholeCameraModel &model);
		const std::vector< GoalInfo > getInfo(const cv::Size &frame_size, const std::vector< std::vector< cv::Point > > &contours, const std::vector<DepthInfo> &depth_maxs, ObjectNum objtype, const image_geometry::PinholeCameraModel &model);
};
