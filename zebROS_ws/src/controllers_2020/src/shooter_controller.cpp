#include "controllers_2020/shooter_controller.h"
#include <pluginlib/class_list_macros.h> //to compile as a controller
#include <std_msgs/Bool.h>

namespace shooter_controller
{
    bool ShooterController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {

        //get interface
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();
        hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

        //Initialize piston joints
        shooter_hood_joint_ = pos_joint_iface->getHandle("shooter_hood_joint"); //joint name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml

        //Initialize motor joints
        XmlRpc::XmlRpcValue shooter_motor_params;
        if ( !controller_nh.getParam("shooter_joint", shooter_motor_params)) //grabbing the config value under the controller's section in the main config file
        {
            ROS_ERROR_STREAM("Could not read shooter_motor_params");
            return false;
        }
        //initialize motor joint using those config values
        if ( !shooter_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, shooter_motor_params)) {
            ROS_ERROR("Cannot initialize shooter_joint!");
            return false;
        }

        //ros::NodeHandle n_shooter_params(controller_nh, "shooter_params");
        if (!controller_nh.getParam("time_to_raise_hood", time_to_raise_hood_))
        {
            ROS_ERROR("Could not find time_to_raise_hood");
            return false;
        }

        if (!controller_nh.getParam("speed_threshhold", speed_threshhold_))
        {
            ROS_ERROR("Could not find speed_threshhold");
            return false;
        }

        //Initialize your ROS server
        shooter_service_ = controller_nh.advertiseService("shooter_command", &ShooterController::cmdService, this);
        ready_to_shoot_pub_ = controller_nh.advertise<std_msgs::Bool>("ready_to_shoot", 1);
        return true;
    }

    void ShooterController::starting(const ros::Time &/*time*/) {
        //give command buffer(s) an initial value
        cmd_buffer_.writeFromNonRT(ShooterCommand(0.0, false));
        last_command_time_ = ros::Time::now();
    }

    void ShooterController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
        const ShooterCommand shooter_cmd = *(cmd_buffer_.readFromRT());

        shooter_joint_.setCommand(shooter_cmd.set_velocity_);
        //Set values of the pistons based on the command. Can be 1.0, 0.0, or -1.0. -1.0 is only used with double solenoids
        if(shooter_cmd.shooter_hood_raise_ == true)
        {
            shooter_hood_joint_.setCommand(1.0);
        }
        else
        {
            shooter_hood_joint_.setCommand(0.0);
        }

        if(((ros::Time::now() - last_command_time_).toSec() > time_to_raise_hood_) &&
                fabs(shooter_joint_.getSpeed() - shooter_cmd.set_velocity_) < speed_threshhold_ &&
                fabs(shooter_cmd.set_velocity_) > 1e-6)
        {
            std_msgs::Bool msg;
            msg.data = true;
           ready_to_shoot_pub_.publish(msg); 
        }
        else
        {
            std_msgs::Bool msg;
            msg.data = false;
            ready_to_shoot_pub_.publish(msg);
        }
    }

    void ShooterController::stopping(const ros::Time &/*time*/) {
    }
    bool ShooterController::cmdService(controllers_2020_msgs::ShooterSrv::Request &req, controllers_2020_msgs::ShooterSrv::Response &/*response*/) {
        if(isRunning())
        {
            //assign request value to command buffer(s)
            cmd_buffer_.writeFromNonRT(ShooterCommand(req.set_velocity, req.shooter_hood_raise));
            last_command_time_ = ros::Time::now();
        }
        else
        {
            ROS_ERROR_STREAM("Can't accept new commands. ShooterController is not running.");
            return false;
        }
        return true;
    }

}//namespace

PLUGINLIB_EXPORT_CLASS(shooter_controller::ShooterController, controller_interface::ControllerBase)

