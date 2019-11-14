#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h> //code for real-time buffer - stop multple things writing to same variable at same time
#include <controller_interface/controller.h> //for writing controllers
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_msgs/Bool.h>
#include "mech_controller/MechSrv.h"

namespace mech_controller
{

//this is the actual controller, so it stores all of the  update() functions and the actual handle from the joint interface
//if it was only one type, controller_interface::Controller<TalonCommandInterface> here
class MechController : public controller_interface::Controller<hardware_interface::PositionJointInterface>
{
        public:
            MechController()
            {
            }

            //should this be hardware_interface::TalonCommandInterface instead? What's the reason to import RobotHW then get CommandInterface from that instead of just importing TalonCommandIface?
            //answer to my question: the TalonCommandInterface is passed in if it's not a multiInterfaceController, and just one kind of joint is made!
            virtual bool init(hardware_interface::PositionJointInterface *hw,
                              ros::NodeHandle             &root_nh,
                              ros::NodeHandle             &controller_nh) override;
            virtual void starting(const ros::Time &time) override;
            virtual void update(const ros::Time & time, const ros::Duration& period) override;
            virtual void stopping(const ros::Time &time) override;

            virtual bool cmdService(mech_controller::MechSrv::Request &req,
                                    mech_controller::MechSrv::Response &res);

        private:
            hardware_interface::JointHandle extend_joint_; //interface for the in/out solenoid joint

            realtime_tools::RealtimeBuffer<bool> cmd_buffer_; //buffer for clamp and extend commands

            ros::ServiceServer mech_service_; //service for receiving commands
}; //class

} //namespace

