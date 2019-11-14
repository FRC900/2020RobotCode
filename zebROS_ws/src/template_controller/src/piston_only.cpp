//TEMPLATE FOR WRITING A CONTROLLER
//replace "mech" with the name of your mechanism, words_separated_by_underscores
//replace "Mech" with the name of your mechanism, ThisIsTheFormatForThat


#include "mech_controller/mech_controller.h"

namespace mech_controller
{
	bool MechController::init(hardware_interface::RobotHW *hw,
									 ros::NodeHandle                 &/*root_nh*/,
									 ros::NodeHandle                 &controller_nh)
	{
		//get interface
		hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

		//Give the piston joints a value
		/* Ex:
		push_joint_ = pos_joint_iface->getHandle("joint_name"); //joint_name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml
		*/

		//Initialize your ROS server
		/* Ex:
		mech_service_ = controller_nh.advertiseService("mech_command", &MechController::cmdService, this);
		*/

		return true;
	}

	void MechController::starting(const ros::Time &/*time*/) {
		//give command buffer(s) an initial value
		/* Ex:
		cmd_buffer_.writeFromNonRT(true);
		*/
	}

	void MechController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
		//grab value from command buffer(s)
		/* Ex:
		const bool extend_cmd = *(cmd_buffer_.readFromRT());
		*/


		//Set values of the pistons. Can be 1.0, 0.0, or -1.0. -1.0 is only used with double solenoids
		/* Syntax: push_joint_.setCommand(1.0); */
	}

	void MechController::stopping(const ros::Time &/*time*/) {
	}

	bool MechController::cmdService(mech_controller::MechSrv::Request &req, mech_controller::MechSrv::Response &/*response*/) {
		if(isRunning())
		{
			//assign request value to command buffer(s)
			/* Ex:
			cmd_buffer_.writeFromNonRT(PanelCommand(!req.claw_release, req.push_extend));
			*/
		}
		else
		{
			ROS_ERROR_STREAM("Can't accept new commands. MechController is not running.");
			return false;
		}
		return true;
	}

}//namespace

//DON'T FORGET TO EXPORT THE CLASS SO CONTROLLER_MANAGER RECOGNIZES THIS AS A TYPE
PLUGINLIB_EXPORT_CLASS(mech_controller::MechController, controller_interface::ControllerBase)
