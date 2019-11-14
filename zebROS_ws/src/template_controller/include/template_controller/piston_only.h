#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller

//REMEMBER TO INCLUDE CUSTOM SERVICE

namespace mech_controller
{

//this is the controller class, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, can do controller_interface::Controller<TalonCommandInterface or PositionJointInterface> here
class MechController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface> //including both talons and pistons so can copy paste this w/o having to change it if you want to add a talon
{
        public:
            MechController()
            {
            }

			//the four essential functions for a controller: init, starting, update, stopping

            //if just doing a one-type-of-interface controller (PositionJointInterface vs. TalonCommandInterface), can pass e.g
			//hardware_interface::PositionJointInterface *pos_joint_iface
			//to this function and not have to get the interface in the src file
			virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

			//ROS server callback function
            virtual bool cmdService(mech_controller::MechSrv::Request &req,
                                    mech_controller::MechSrv::Response &res);

        private:
            //variable for piston joint
			/* Ex:
			hardware_interface::JointHandle push_joint_; //interface for the piston joint
			*/

			//set up your ROS server and buffer
			/* Ex:
            ros::ServiceServer mech_service_; //service for receiving commands
            realtime_tools::RealtimeBuffer<bool> cmd_buffer_; //buffer for commands
			*/

}; //class

} //namespace

