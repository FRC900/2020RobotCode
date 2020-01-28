//REPLACE "package" with the name of the package this is in



#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include <pluginlib/class_list_macros.h> //to compile as a controller

#include <controllers_2020_msgs/TurretSrv.h>
//REMEMBER TO INCLUDE CUSTOM SERVICE

namespace turret_controller
{

//this is the controller class, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, can do controller_interface::Controller<TalonCommandInterface or PositionJointInterface> here
class TurretController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface> //including both talons and pistons so can copy paste this w/o having to change it if you want to add a talon
{
        public:
            TurretController()
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
            virtual bool cmdService(controllers_2020_msgs::TurretSrv::Request &req,
                                    controllers_2020_msgs::TurretSrv::Response &res);

        private:
            //variable for piston joint
			/* Ex:
			hardware_interface::JointHandle push_joint_; //interface for the piston joint
			*/

			//variable for motor joint
			/* Ex:
			talon_controllers::TalonPercentOutputControllerInterface motor_name_joint_; //other types exist FYI
			*/

			//set up your ROS server and buffer
			/* Ex:
            ros::ServiceServer mech_service_; //service for receiving commands
            realtime_tools::RealtimeBuffer<bool> cmd_buffer_; //buffer for commands
			*/

			talon_controllers::TalonMotionMagicCloseLoopControllerInterface turret_joint_;
			hardware_interface::JointHandle turret_arm_joint_;
			realtime_tools::RealtimeBuffer<TurretCommand> turret_cmd_;
}; //class

} //namespace

