#include "controllers_2020/turret_controller.h"

namespace turret_controller
{
	bool TurretController::init(hardware_interface::RobotHW *hw,
									 ros::NodeHandle                 &/*root_nh*/,
									 ros::NodeHandle                 &controller_nh)
	{
		hardware_interface::TalonCommandInterface *const talon_command_iface = hw->get<hardware_interface::TalonCommandInterface>();

		//Initialize piston joints
		/* Ex:
		push_joint_ = pos_joint_iface->getHandle("joint_name"); //joint_name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml
		*/

		XmlRpc::XmlRpcValue turret_params;
		if ( !controller_nh.getParam("turret_joint", turret_params)) //grabbing the config value under the controller's section in the main config file
		{
			ROS_ERROR_STREAM("Could not read turret_params");
			return false;
		}
		//initialize motor joint using those config values
		if ( !turret_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, turret_params) {
			ROS_ERROR("Cannot initialize turret_joint!");
			return false;
		}

		//Initialize your ROS server
		turret_service = controller_nh.advertiseService("turret_command", &TurretController::cmdService, this);

		return true;
	}

	void TurretController::starting(const ros::Time &/*time*/) {
		//give command buffer(s) an initial value
		/* Ex:
		cmd_buffer_.writeFromNonRT(true);
		*/
		turret_cmd_.writeFromNonRT(true);
	}

	void TurretController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		//grab value from command buffer(s)
		/* Ex:
		const bool extend_cmd = *(cmd_buffer_.readFromRT());
		*/
		const TurretCommand turret_cmd = *(turret_cmd_.readFromRT());

		//Set values of the pistons based on the command. Can be 1.0, 0.0, or -1.0. -1.0 is only used with double solenoids
		/* Syntax: push_joint_.setCommand(1.0); */

		//for motors, it's the same syntax, but the meaning of the argument passed to setCommand() differs based on what motor mode you're using
	}

	void TurretController::stopping(const ros::Time &/*time*/) {
	}

	bool TurretController::cmdService(package::MechSrv::Request &req, package::MechSrv::Response &/*response*/) {
		if(isRunning())
		{
			//assign request value to command buffer(s)
			/* Ex:
			cmd_buffer_.writeFromNonRT(req.claw_release);
			*/
			turret_cmd_.writeFromNonRT(TurretCommand());
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. TurretController is not running.");
			return false;
		}
		return true;
	}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(turret_controller::TurretController, controller_interface::ControllerBase)
