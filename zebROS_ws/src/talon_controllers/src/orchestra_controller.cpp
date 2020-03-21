#include "talon_controllers/orchestra_controller.h"

namespace orchestra_controller
{

bool OrchestraController::init(hardware_interface::OrchestraCommandInterface *hw,
								ros::NodeHandle 							&root_nh,
								ros::NodeHandle 							&controller_nh)
{
	ROS_INFO_STREAM("Orchestra controller init");

	std::vector<std::string> orchestra_names = hw->getNames();
	if (orchestra_names.size() > 1) {
		ROS_ERROR_STREAM("Cannot initialize multiple orchestras");
		return false;
        }
	else if (orchestra_names.size() < 1) {
		ROS_ERROR_STREAM("Cannot initialize zero orchestras");
		return false; 
        }
	const std::string orchestra_name = orchestra_names[0];

        orchestra_command_handle_ = hw->getHandle(orchestra_name);  // throws on failure
        return true;
}

void OrchestraController::starting(const ros::Time &time)
{
    //read in the default instruments from an orcehstra config file
}

void OrchestraController::update(const ros::Time &time, const ros::Duration &)
{
    //set play/pause/stop
    //clear/add instruments if service is called
    //load new music
}

void OrchestraController::stopping(const ros::Time & )
{}

}

PLUGINLIB_EXPORT_CLASS( orchestra_controller::OrchestraController, controller_interface::ControllerBase)
