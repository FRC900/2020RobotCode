#include "mech_controller/mech_controller.h"

namespace mech_controller
{
	bool MechController::init(hardware_interface::PositionJointInterface *hw,
									 ros::NodeHandle                 &/*root_nh*/,
									 ros::NodeHandle                 &controller_nh)
	{

		return true;
	}

	void MechController::starting(const ros::Time &/*time*/) {

	}

	void MechController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {

	}

	void MechController::stopping(const ros::Time &/*time*/) {
	}

	bool MechController::cmdService(mech_controller::MechSrv::Request &req,
									mech_controller::MechSrv::Response &/*response*/)
	{
		if(isRunning())
		{

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
