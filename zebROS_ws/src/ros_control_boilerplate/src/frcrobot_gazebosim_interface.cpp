
#include <ros_control_boilerplate/frcrobot_gazebosim_interface.h>

namespace frcrobot_control
{
	bool FRCRobotGazeboSim::initSim(const std::string& robot_namespace,
				ros::NodeHandle model_nh,
				gazebo::physics::ModelPtr parent_model,
				const urdf::Model *const urdf_model,
				std::vector<transmission_interface::TransmissionInfo> transmissions)
	{
		FRCRobotSimInterface::init(model_nh, nullptr);
		e_stop_active_ = false;

		RobotHWSim::registerInterface(&talon_state_interface_);
		RobotHWSim::registerInterface(&talon_remote_state_interface_);
		RobotHWSim::registerInterface(&joint_state_interface_);
		RobotHWSim::registerInterface(&talon_command_interface_);
		RobotHWSim::registerInterface(&joint_command_interface_);
		RobotHWSim::registerInterface(&joint_position_interface_);
		RobotHWSim::registerInterface(&joint_velocity_interface_);
		RobotHWSim::registerInterface(&joint_effort_interface_); // empty for now
		RobotHWSim::registerInterface(&imu_interface_);
		RobotHWSim::registerInterface(&pdp_state_interface_);
		RobotHWSim::registerInterface(&pcm_state_interface_);
		RobotHWSim::registerInterface(&robot_controller_state_interface_);
		RobotHWSim::registerInterface(&match_state_interface_);

		RobotHWSim::registerInterface(&joint_remote_interface_); // list of Joints defined as remote
		RobotHWSim::registerInterface(&pdp_remote_state_interface_);
		RobotHWSim::registerInterface(&pcm_remote_state_interface_);
		RobotHWSim::registerInterface(&imu_remote_interface_);
		RobotHWSim::registerInterface(&match_remote_state_interface_);

		for (size_t i = 0; i < can_ctre_mc_names_.size(); i++)
		{
			ROS_INFO_STREAM_NAMED("frcrobot_gazebosim_interface",
								  "Connecting to gazebo : CTRE MC joint" << i << "=" << can_ctre_mc_names_[i] <<
								  (can_ctre_mc_local_updates_[i] ? " local" : " remote") << " update, " <<
								  (can_ctre_mc_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
								  " as " << (can_ctre_mc_is_talon_[i] ? "TalonSRX" : "VictorSPX")
								  << " CAN id " << can_ctre_mc_can_ids_[i]);

			gazebo::physics::JointPtr joint = parent_model->GetJoint(can_ctre_mc_names_[i]);
			if (!joint)
			{
				ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "This robot has a joint named \"" << can_ctre_mc_names_[i]
						<< "\" which is not in the gazebo model.");
			}
			sim_joints_.push_back(joint);
		}
		// get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
		gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
		gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
		physics_type_ = physics->GetType();
		if (physics_type_.empty())
		{
			ROS_WARN_STREAM_NAMED("frcrobot_gazebosim_interface", "No physics type found.");
		}

		ROS_INFO("FRCRobotGazeboSim Ready.");

		return true;
	}

	void FRCRobotGazeboSim::readSim(ros::Time time, ros::Duration period)
	{
		for (size_t i = 0; i < num_can_ctre_mcs_; i++)
		{
			if (sim_joints_[i] && can_ctre_mc_local_hardwares_[i])
			{
				// Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
				const double position = sim_joints_[i]->Position(0);
#else
				const double position = sim_joints_[i]->GetAngle(0).Radian();
#endif
				auto &ts = talon_state_[i];
				ts.setPosition(position);
				if (ts.getTalonMode() !=  hardware_interface::TalonMode_MotionMagic)
				ts.setSpeed(sim_joints_[i]->GetVelocity(0));

#if 0
				if (joint_types_[j] == urdf::Joint::PRISMATIC)
				{
					joint_position_[j] = position;
				}
				else
				{
					joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
							position);
				}
				joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
#endif
			}
		}
	}

	void FRCRobotGazeboSim::writeSim(ros::Time time, ros::Duration period)
	{
		FRCRobotSimInterface::write(period);
		for (size_t i = 0; i < num_can_ctre_mcs_; i++)
		{
			if (sim_joints_[i] && can_ctre_mc_local_hardwares_[i])
			{
				auto &ts = talon_state_[i];
				hardware_interface::TalonMode simulate_mode = ts.getTalonMode();
				double position;
				double velocity;
				bool set_position = false;
				bool set_velocity = false;
				if (simulate_mode == hardware_interface::TalonMode_Position)
				{
					// Assume instant velocity
					position = ts.getSetpoint();
					set_position = true;
					velocity = 0;
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_Velocity)
				{
					// Assume instant acceleration for now
					set_position = false;
					velocity = ts.getSetpoint();
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_MotionMagic)
				{
					position = ts.getPosition();
					set_position = true;
					velocity = ts.getSpeed();
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_Disabled)
				{
					set_position = false;
					velocity = 0;
					set_velocity = true;
				}
				else if (simulate_mode == hardware_interface::TalonMode_PercentOutput)
				{
					position = ts.getPosition();
					set_position = true;
					velocity = ts.getSpeed();
					set_velocity = true;
				}
				ROS_INFO_STREAM("talon " << can_ctre_mc_names_[i] <<
						" setpoint=" << ts.getSetpoint() <<
						" pos=" << position <<
						" set_position=" << set_position <<
						" velocity=" << velocity <<
						" set_velocity=" << set_velocity <<
						" e_stop_active_=" << e_stop_active_);
				if (set_position)
				{
					sim_joints_[i]->SetPosition(0, position, true);
				}
				if (set_velocity)
				{
					//if (physics_type_.compare("ode") == 0)
					//{
				//		sim_joints_[i]->SetParam("vel", 0, e_stop_active_ ? 0 : velocity);
			//		}
			//		else
					{
						sim_joints_[i]->SetVelocity(0, e_stop_active_ ? 0 : velocity);
					}
				}
			}
		}
	}
} // namespace
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(frcrobot_control::FRCRobotGazeboSim, gazebo_ros_control::RobotHWSim)
