#include <ros_control_boilerplate/frcrobot_phoenixsim_interface.h>

/* additional include for sim */
#include <ctre/phoenix/cci/Platform_CCI.h> // c_SimCreate
#include <ctre/phoenix/cci/Unmanaged_CCI.h> // c_FeedEnable

#include <HALInitializer.h>

namespace frcrobot_control
{
void FRCRobotPhoenixSimInterface::init(ros::NodeHandle &nh, urdf::Model *urdf_model)
{
	// Need to get count of CTRE can devices and ids
	ros::NodeHandle rpnh(nh, "hardware_interface");
	XmlRpc::XmlRpcValue joint_param_list;
	if (!rpnh.getParam("joints", joint_param_list))
		throw std::runtime_error("No joints were specified.");
	for (int i = 0; i < joint_param_list.size(); i++)
	{
		XmlRpc::XmlRpcValue &joint_params = joint_param_list[i];
		if (!joint_params.hasMember("name"))
			throw std::runtime_error("A joint name was not specified");
		XmlRpc::XmlRpcValue &xml_joint_name = joint_params["name"];
		if (!xml_joint_name.valid() ||
			xml_joint_name.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint name was specified (expecting a string)");
		const std::string joint_name = xml_joint_name;

		if (!joint_params.hasMember("type"))
			throw std::runtime_error("A joint type was not specified for joint " + joint_name);
		XmlRpc::XmlRpcValue &xml_joint_type = joint_params["type"];
		if (!xml_joint_type.valid() ||
			xml_joint_type.getType() != XmlRpc::XmlRpcValue::TypeString)
			throw std::runtime_error("An invalid joint type was specified (expecting a string) for joint " + joint_name);
		const std::string joint_type = xml_joint_type;
		bool saw_local_keyword = false;
		bool local = true;
		bool local_update;
		bool local_hardware;
		if ((joint_type == "can_talon_srx") || (joint_type == "can_victor_spx") )
		{
			readJointLocalParams(joint_params, local, saw_local_keyword, local_update, local_hardware);

			const bool has_can_id = joint_params.hasMember("can_id");
			if (!local_hardware && has_can_id)
				throw std::runtime_error("A CAN Talon SRX / Victor SPX can_id was specified with local_hardware == false for joint " + joint_name);

			int can_id = 0;
			if (local_hardware)
			{
				if (!has_can_id)
					throw std::runtime_error("A CAN Talon SRX / Victor SPX can_id was not specified");
				XmlRpc::XmlRpcValue &xml_can_id = joint_params["can_id"];
				if (!xml_can_id.valid() ||
						xml_can_id.getType() != XmlRpc::XmlRpcValue::TypeInt)
					throw std::runtime_error("An invalid joint can_id was specified (expecting an int) for joint " + joint_name);
				can_id = xml_can_id;
				auto it = std::find(can_ctre_mc_can_ids_.cbegin(), can_ctre_mc_can_ids_.cend(), can_id);
				if (it != can_ctre_mc_can_ids_.cend())
					throw std::runtime_error("A duplicate can_id was specified for joint " + joint_name);

				// TODO : fix device type when victor is supported
				c_SimCreate(DeviceType::TalonSRXType, can_id);
				ROS_INFO_STREAM("phoenixsim : creating DeviceType::TalonSRXType name=" << joint_name << " id=" << can_id);
			}
		}
	}

	// for now we need a delay so backend can properly setup device properties
	// before we attempt creating API objects
	// this may not be necessary anymore
	ros::Duration(1.0).sleep();

	hal::init::InitializeHAL();
	FRCRobotHWInterface::init(nh, urdf_model);
}


// Write should grab the motor output and use it to run 1 timestep
// of the underlying motor sim
void FRCRobotPhoenixSimInterface::write(ros::Duration &elapsed_time)
{
	c_FeedEnable(500);
	FRCRobotHWInterface::write(elapsed_time);

	// Update the motor connected to each Talon here?
}
} // namespace
