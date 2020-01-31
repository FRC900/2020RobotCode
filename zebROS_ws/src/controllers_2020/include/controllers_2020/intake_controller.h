
#pragma once

#include <ros/ros.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time

//controller interfaces
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <talon_controllers/talon_controller_interface.h>

#include <pluginlib/class_list_macros.h> //to compile as a controller

#include <controllers_2020_msgs/IntakeSrv.h>

//REMEMBER TO INCLUDE CUSTOM SERVICE

namespace intake_controller
{


class IntakeCommand
{
	public:
	IntakeCommand()
		: set_power_(0.0)
		, intake_arm_extend_(false)
	{
	}
	IntakeCommand(double set_power, bool intake_arm_extend)
	{
		set_power_ = set_power;
		intake_arm_extend_ = intake_arm_extend;
	}
	double set_power_;
	bool intake_arm_extend_;
};

//this is the controller class, used to make a controller
class IntakeController : public controller_interface::MultiInterfaceController<hardware_interface::PositionJointInterface, hardware_interface::TalonCommandInterface>
{
        public:
            IntakeController()
            {
            }

			//the four essential functions for a controller: init, starting, update, stopping

			virtual bool init(hardware_interface::RobotHW *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;
	    bool cmdService (controllers_2020_msgs::IntakeSrv::Request &req, controllers_2020_msgs::IntakeSrv::Response &);
        private:
	    	talon_controllers::TalonPercentOutputControllerInterface intake_joint_;//intake for intake motor
		hardware_interface::JointHandle intake_arm_joint_;//interface for intake arm solenoid
		realtime_tools::RealtimeBuffer<IntakeCommand> intake_cmd_;
		ros::ServiceServer intake_service_;
		double roller_diameter_;



}; //class

} //namespace

