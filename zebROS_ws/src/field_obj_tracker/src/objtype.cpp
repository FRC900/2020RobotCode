// A class for defining objects we're trying to
// detect.  The class stores information about shape
// and size of the objects in real-world measurements
#include "field_obj_tracker/objtype.hpp"
#include "field_obj_tracker/convert_coords.h"

using namespace std;
using namespace cv;

ObjectType::ObjectType(ObjectNum contour_type_id=UNINITIALIZED) {
	switch(contour_type_id) {
		//loads one of the preset shapes into the
		//object
		case POWER_PORT_2020: //target on the POWER PORT (2020)
			depth_ = 0;
			real_height_ = 1.4796516; // TODO: Fix this using actual height
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0.2492375,0.4318));
			contour_.push_back(Point2f(0.7477125,0.4318));
			contour_.push_back(Point2f(0.99695,0));
			contour_.push_back(Point2f(0.9382912,0));
			contour_.push_back(Point2f(0.7183831,0.381));
			contour_.push_back(Point2f(0.2785669,0.381));
			contour_.push_back(Point2f(0.0586588,0));
			name_ = "power_port";
			break;
		case LOADING_BAY_2020: //target on the LOADING BAY (2020)
			depth_ = 0;
			real_height_ = -.3046984; // TODO: fix this using actual height
			contour_.push_back(Point2f(0.0508,0.2286));
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.2794));
			contour_.push_back(Point2f(0.1778,0.2794));
			contour_.push_back(Point2f(0.1778,0));
			contour_.push_back(Point2f(0.0508,0.0508));
			contour_.push_back(Point2f(0.127,0.0508));
			contour_.push_back(Point2f(0.127,0.2286));
			name_ = "loading_bay";
			break;
		case TEST_TARGET_2020:
			depth_ = 0;
			real_height_ = 0; // TODO: fix this using actual height
			contour_.push_back(Point2f(0, 0));
			contour_.push_back(Point2f(0, 0.102));
			contour_.push_back(Point2f(0.102, 0));
			contour_.push_back(Point2f(0.102, 0.102));
			name_ = "test_target";
			break;
		default:
			cerr << "error initializing object!" << endl;
	}

	computeProperties();
}

ObjectType::ObjectType(const vector< Point2f > &contour_in, const string &name_in, const float &depth_in) :
	contour_(contour_in),
	depth_(depth_in),
	name_(name_in)
{
	if(contour_in.size() == 0 || name_in.length() == 0 || depth_in < 0)
		throw std::invalid_argument("bad argument to ObjectType Point2f");
	computeProperties();
}

ObjectType::ObjectType(const vector< Point > &contour_in, const string &name_in, const float &depth_in):
	depth_(depth_in),
	name_(name_in)
{
	if(contour_in.size() == 0 || name_in.length() == 0 || depth_in < 0)
		throw std::invalid_argument("bad argument to ObjectType Point");
	for (auto it = contour_in.cbegin(); it != contour_in.cend(); ++it)
		contour_.push_back(Point2f(it->x, it->y));
	computeProperties();
}

void ObjectType::computeProperties()
{
	float min_x = numeric_limits<float>::max();
	float min_y = numeric_limits<float>::max();
	float max_x = -min_x;
	float max_y = -min_y;
	for (auto it = contour_.cbegin(); it != contour_.cend(); ++it)
	{
		min_x = min(min_x, it->x);
		min_y = min(min_y, it->y);
		max_x = max(max_x, it->x);
		max_y = max(max_y, it->y);
	}
	width_ = max_x - min_x;
	height_ = max_y - min_y;
	area_ = contourArea(contour_);

	//compute moments and use them to find center of mass
	Moments mu = moments(contour_, false);
	com_ = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
}

// Given a rectangular bounding box and measured (or estimated) depth, return the world coords of the object
Point3f ObjectType::screenToWorldCoords(const Rect &screen_position, double avg_depth, const image_geometry::PinholeCameraModel &model) const
{
	// avg_depth is to front of object.  Add in half the
	// object's depth to move to the center of it
	avg_depth += depth_ / 2.;
	return ConvertCoords(model).screen_to_world(screen_position, name_, avg_depth);
}

// Given the real-world position of the object, build a rectangle bounding box in screen coords for it
Rect ObjectType::worldToScreenCoords(const Point3f &position, const image_geometry::PinholeCameraModel &model) const
{
	// TODO - handle object depth?
	const Point2f screen_center = ConvertCoords(model).world_to_screen(position, name_);

	// Object distance
	const float r = sqrtf(position.x * position.x + position.y * position.y + position.z * position.z) - depth_ / 2.;
	const Point topLeft(
			cvRound(screen_center.x - model.getDeltaU(width_  / 2.0, r)),
			cvRound(screen_center.y - model.getDeltaV(height_ / 2.0, r)));

	return Rect(topLeft.x, topLeft.y, cvRound(model.getDeltaU(width_, r)), cvRound(model.getDeltaV(height_, r)));
}

float ObjectType::expectedDepth(const Rect &screen_position, const image_geometry::PinholeCameraModel &model) const
{
	// du = fx * dX / Z ==> Z = fx * dX / du
	// where du = size in screen coords
	//       dx = size in real life
	//       fx = focal length in X
	//       Z  = depth
	if (screen_position.width > screen_position.height) // use larger the values for better resolution
		return model.fx() * width_ / screen_position.width;
	return model.fy() * height_ / screen_position.height;
}

bool ObjectType::operator== (const ObjectType &t1) const
{
	return this->shape() == t1.shape();
}
