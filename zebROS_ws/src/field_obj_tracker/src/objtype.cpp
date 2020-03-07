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
                        contour_.push_back(Point2f(0.0508,0));
                        contour_.push_back(Point2f(0,0));
                        contour_.push_back(Point2f(0,0.2794));
                        contour_.push_back(Point2f(0.1778,0.2794));
                        contour_.push_back(Point2f(0.1778,0));
                        contour_.push_back(Point2f(0.0508,0));
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
                case RED_POWER_PORT_HIGH_GOAL:
                        depth_ = 0;
                        height_ = .889;
                        width_ = 1.02;
                        positions_.push_back(Point3f(15.98295,2.404364, 2.49555));
                        name_ = "red_power_port_high_goal";
                        break;
                case BLUE_POWER_PORT_HIGH_GOAL:
                        depth_ = 0;
                        height_ = .889;
                        width_ = 1.02;
                        positions_.push_back(Point3f(0,5.806186, 2.49555));
                        name_ = "blue_power_port_high_goal";
                        break;
                case RED_POWER_PORT_LOW_GOAL:
                        depth_=0;
                        width_=.86;
                        height_ =.254;
                        name_="red_power_port_high_goal";
                        positions_.push_back(Point3f(15.98295,2.404364, 0.5842));
                        break;
                case BLUE_POWER_PORT_LOW_GOAL:
                        depth_=0;
                        width_=.86;
                        height_=.254;
                        name_="blue_power_port_low_goal";
                        positions_.push_back(Point3f(0,5.806186, 0.5842));
                        break;
                case POWER_CELL:
                        height_ = 0.18;
                        width_ = 0.18;
                        depth_ = 0.18;
                        name_="power_cell";
                        break;
                case POWER_PORT_YELLOW_GRAPHICS:
                        height_= .4572;
                        width_= .6604;
                        depth_= 0;
                        name_="power_port_yellow_graphics";
                        positions_.push_back(Point3f(0,5.806186, 1.412875));
                        positions_.push_back(Point3f(15.98295,2.404364, 1.412875));
                        break;
                case RED_POWER_PORT_FIRST_LOGO:
                        height_= .26;
                        width_ = .355;
                        depth_ = 0;
                        name_ = "red_power_port_first_logo";
                        positions_.push_back(Point3f(0,3.369564, 2.2098));
                        positions_.push_back(Point3f(15.80515,1.011936, 2.2098));
                        break;
                case BLUE_POWER_PORT_FIRST_LOGO:
                        height_=.26;
      width_ = .355;
      depth_ = 0;
                        name_ = "blue_power_port_first_logo";
                        positions_.push_back(Point3f(15.98295,4.840986, 2.2098));
      positions_.push_back(Point3f(.1778,7.198614, 2.2098));
                        break;
                case RED_LOADING_BAY_TAPE:
                        height_ = .533;
                        width_ = 0.62;
                        depth_ = 0;
                        name_ = "red_loading_bay_tape";
                        positions_.push_back(Point3f(0,2.081784, 0.4572));
                        break;
                case BLUE_LOADING_BAY_TAPE:
                        height_ = .533;
                        width_ = .62;
                        depth_ = 0;
                        name_ = "blue_loading_bay_tape";
                        positions_.push_back(Point3f(15.98295,6.128766,0.4572));
                        break;
                case RED_LOADING_BAY_LEFT_GRAPHICS:
                        height_ = 0.4318;
                        width_ = 0.1651;
                        depth_ = 0;
                        name_ = "red_loading_bay_left_graphics";
                        positions_.push_back(Point3f(0,1.947164,0.508));
                        break;
                case RED_LOADING_BAY_RIGHT_GRAPHICS:
                        height_ = 0.654;
                        width_ = .1651;
                        depth_ = 0;
                        name_ = "red_loading_bay_right_graphics";
                        positions_.push_back(Point3f(0,3.166364,0.6096));
                        break;
                case BLUE_LOADING_BAY_LEFT_GRAPHICS:
                        height_ = .4318;
                        width_ = .1651;
                        depth_ = 0;
                        name_ = "blue_loading_bay_left_graphics";
                        positions_.push_back(Point3f(15.983204,6.263386,0.508));
                        break;
                case BLUE_LOADING_BAY_RIGHT_GRAPHICS:
                        height_ = 0.654;
                        width_ = .1651;
                        depth_ = 0;
                        name_ = "blue_loading_bay_right_graphics";
                        positions_.push_back(Point3f(15.983204,5.044186,0.6096));
                        break;
                case RED_TAPE_CORNER:
                        height_ = 0;                    //need the other dimensions
                        name_ = "red_tape_corner";
                        positions_.push_back(Point3f(.762,2.55651,0));
                        positions_.push_back(Point3f(15.221204,2.40411,0));
                        positions_.push_back(Point3f(5.246878,1.4097,0));
                        positions_.push_back(Point3f(10.73658,1.4097,0));
                        break;
                case BLUE_TAPE_CORNER:
                        height_ = 0;                    //need the other dimensions
                        name_ = "blue_tape_corner";
                        positions_.push_back(Point3f(0.762,5.806186,0));
                        positions_.push_back(Point3f(15.221204,5.653786,0));
                        positions_.push_back(Point3f(10.73658,6.80085,0));
                        positions_.push_back(Point3f(5.246878,6.80085,0));
                        break;
                case RED_DS_LIGHT:
                        height_ = 0.214;
                        width_ = 0.0762;
                        depth_ = 0.0762;
                        positions_.push_back(Point3f(0.381,7.550658,1.9304));
                        positions_.push_back(Point3f(0,4.257675,1.9304));
                        positions_.push_back(Point3f(0.381,0.659892,1.9304));
                        name_ = "red_ds_light";
                        break;
                case BLUE_DS_LIGHT:
                        height_ = .214;
                        width_ = 0.0762;
                        depth_ = 0.0762;
                        name_ = "blue_ds_light";
                        positions_.push_back(Point3f(15.60195,7.550658, 1.9304));
                        positions_.push_back(Point3f(15.98295,3.925875, 1.9304));
                        positions_.push_back(Point3f(15.60195,0.659892, 1.9304));
                        break;
                case DS_LIGHT:
                        height_ = .214;
      width_ = 0.0762;
      depth_ = 0.0762;
      positions_.push_back(Point3f(.381,7.550658, 1.9304));
      positions_.push_back(Point3f(0,4.257675, 1.9304));
      positions_.push_back(Point3f(.381,.659892, 1.9304));
                        positions_.push_back(Point3f(15.60195,7.550658, 1.9304));
      positions_.push_back(Point3f(15.98295,3.952875, 1.9304));
      positions_.push_back(Point3f(15.60195,.659892, 1.9304));
                        name_ = "ds_light";
      break;
                case CONTROL_PANEL_LIGHT:
                        height_ = 0.214;
                        width_ = 0.0762;
                        depth_ = 0.0762;
                        name_ = "control_panel_light";
                        positions_.push_back(Point3f(6.88975,1.3589,1.003));
                        positions_.push_back(Point3f(9.09828,6.83895,1.003));
                        break;
                case YELLOW_CONTROL_PANEL_LIGHT:
                        height_ = 0.214;
      width_ = 0.0762;
      depth_ = 0.0762;
      name_ = "yellow_control_panel_light";
      positions_.push_back(Point3f(6.88975,1.3589,1.003));
      positions_.push_back(Point3f(9.09828,6.83895,1.003));
      break;
                case RED_SHIELD_GENERATOR_LIGHT:
                        height_ = .214;
                        width_ = .0762;
                        depth_ = .0762;
                        name_ = "red_shield_generator_light";
                        positions_.push_back(Point3f(10.103993,4.980305,2.39395));
                        break;
                case BLUE_SHIELD_GENERATOR_LIGHT:
                        height_ = .214;
                        width_ = .0762;
                        depth_ = .0762;
                        name_ = "blue_shield_generator_light";
                        positions_.push_back(Point3f(5.879719,3.230245,2.39395));
                        break;
                case SHIELD_GENERATOR_LIGHT:
                        height_ = .214;
      width_ = .0762;
      depth_ = .0762;
      name_ = "shield_generator_light";
      positions_.push_back(Point3f(5.879719,3.230245,2.39395));
      positions_.push_back(Point3f(10.103993,4.980305,2.39395));
                        break;
                case SHIELD_GENERATOR_BACKSTOP:
                        height_ = .1651;
                        width_ = .2286;
                        depth_ = .0635;
                        name_ = "shield_generator_backstop";
                        positions_.push_back(Point3f(10.753852,4.0645588,2.220722));
                        positions_.push_back(Point3f(7.241286,2.3144988,2.220722));
                        positions_.push_back(Point3f(8.741918,5.8960512,2.220722));
                        positions_.push_back(Point3f(5.230368,4.1459912,2.220722));
                        break;
                case SHIELD_GENERATOR_FIRST_LOGO:
                        height_ = .203;
                        width_ = .203;
                        depth_ = 0;
                        name_ = "shield_generator_first_logo";
                case SHIELD_GENERATOR_YELLOW_STRIPE:
                        height_ = .5334;
                        width_ = .508;
                        depth_ = 0;
                        name_ = "shield_generator_yellow_stripe";
                        positions_.push_back(Point3f(8.86333,6.627368,2.54635));
                        positions_.push_back(Point3f(9.195308,6.489954,2.54635));
                        positions_.push_back(Point3f(5.1089306,5.071872,2.54635));
                        positions_.push_back(Point3f(4.971542,4.7399194,2.54635));
                        positions_.push_back(Point3f(6.787896,1.7205706,2.54635));
                        positions_.push_back(Point3f(7.119874,1.583182,2.54635));
                        positions_.push_back(Point3f(10.8752894,3.138678,2.54635));
                        positions_.push_back(Point3f(11.012678,3.470656,2.54635));
                        break;
                case SHIELD_GENERATOR_FLOOR_CENTER_INTERSECTION:
                        height_ = 0;
                        //no object dimensions
                        name_ = "shield_generator_floor_center_intersection";
                        positions_.push_back(Point3f(7.991856,4.105275,0));
                        break;
                case RED_BLUE_BLACK_SHIELD_GENERATOR_FLOOR_INTERSECTION:
                        height_ = 0;
                        //no object dimensions
                        name_ = "red_blue_black_shield_generator_floor_intersection";
                        positions_.push_back(Point3f(5.879719,3.230245,0));
                        positions_.push_back(Point3f(10.103993,4.980305,0));
                        break;
                case BLUE_BLACK_SHIELD_GENERATOR_FLOOR_INTERSECTION:
                        height_ = 0;
                        // no object dimensions
                        name_ = "blue_black_shield_generator_floor_intersection";
                        positions_.push_back(Point3f(6.986143,5.84962,0));
                        break;
                case RED_BLACK_SHIELD_GENERATOR_FLOOR_INTERSECTION:
                        height_ = 0;
                        // no object dimensions
                        name_ = "red_black_shield_generator_floor_intersection";
                        positions_.push_back(Point3f(8.997569,2.36093,0));
                        break;
                case RED_SHIELD_PILLAR_INTERSECTION:
                        height_ = 0;
                        // no object dimensions
                        name_ = "red_shield_pillar_intersection";
                        positions_.push_back(Point3f(11.10996,3.23596,0));
                        positions_.push_back(Point3f(6.885178,1.4859,0));
                        break;
                case BLUE_SHIELD_PILLAR_INTERSECTION:
                        height_ = 0;
                        // no object dimensions
                        name_ = "blue_shield_pillar_intersection";
                        positions_.push_back(Point3f(9.098026,6.72465,0));
                        positions_.push_back(Point3f(4.87426,4.97459,0));
                        break;
                case CONTROL_PANEL:
                        height_ = 0.05;
                        width_ = 0.81;
                        depth_ = 0.81;
                        name_ = "control_panel";
                        positions_.push_back(Point3f(9.26465,7.5057,0.9144));
                        positions_.push_back(Point3f(6.7183,.70485,0.9144));
                        break;
                case DS_NUMBERS:
                        //needs dimensions
                        name_ = "ds_numbers";
                        positions_.push_back(Point3f(15.79245,0.989838,1.9304));
                        positions_.push_back(Point3f(15.98298,4.659757,1.9304));
                        positions_.push_back(Point3f(15.98298,3.245993,1.9304));
                        positions_.push_back(Point3f(15.79245,7.220712,1.9304));
                        positions_.push_back(Point3f(.1905,3.55092,1.9304));
                        positions_.push_back(Point3f(0,3.55092,1.9304));
                        positions_.push_back(Point3f(0,4.964557,1.9304));
                        positions_.push_back(Point3f(0.1905,7.220712,1.9304));
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
 
