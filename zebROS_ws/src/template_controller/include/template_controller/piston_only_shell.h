#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller

//REMEMBER TO INCLUDE CUSTOM MESSAGE

namespace mech_controller
{

//this is the controller class - it's used to make a controller
class MechController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            MechController()
            {
            }

            //The four essential functions for a controller to live
			virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;


        private:




}; //class

} //namespace

