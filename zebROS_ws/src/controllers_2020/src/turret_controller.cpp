#include "controllers_2020/control_panel_controller.h"

namespace turret_controller
{
    bool TurretController::init(hardware_interface::RobotHW *hw,
                                     ros::NodeHandle                 &/*root_nh*/,
                                     ros::NodeHandle                 &controller_nh)
    {
        //get interface
        //hardware_interface::PositionJointInterface *const pos_joint_iface = hw->get<hardware_interface::PositionJointInterface>();

        //Initialize piston joints
        /* Ex:
        push_joint_ = pos_joint_iface->getHandle("joint_name"); //joint_name comes from ros_control_boilerplate/config/[insert_year]_compbot_base_jetson.yaml
        */

        //Initialize motor joints
        /* Ex:
        //get params from config file
        XmlRpc::XmlRpcValue intake_motor_params;
        if ( !controller_nh.getParam("config_value_name", intake_motor_params)) //grabbing the config value under the controller's section in the main config file
        {
            ROS_ERROR_STREAM("Could not read _______ params");
            return false;
        }
        //initialize motor joint using those config values
        if ( !motor_name_joint_.initWithNode(talon_command_iface, nullptr, controller_nh, intake_motor_params) {
            ROS_ERROR("Cannot initialize ______ joint!");
            return false;
        }
        */


        //Initialize your ROS server
        /* Ex:
        control_panel_service_ = controller_nh.advertiseService("control_panel_command", &ControlPanelController::cmdService, this);
        */

        return true;
    }

    void TurretController::starting(const ros::Time &/*time*/) {
        //give command buffer(s) an initial value
        /* Ex:
        cmd_buffer_.writeFromNonRT(true);
        */	
    }

    void TurretController::update(const ros::Time &/*time*/, const ros::Duration &/*period*/) {
        //grab value from command buffer(s)
