#pragma once

#include <gazebo_ros_control/robot_hw_sim.h>
#include <ros_control_boilerplate/frcrobot_sim_interface.h>

namespace frcrobot_control
{
	class FRCRobotGazeboSim : public gazebo_ros_control::RobotHWSim, frcrobot_control::FRCRobotSimInterface
	{
		public:
			/// \brief Initialize the simulated robot hardware
			///
			/// Initialize the simulated robot hardware.
			///
			/// \param robot_namespace  Robot namespace.
			/// \param model_nh  Model node handle.
			/// \param parent_model  Parent model.
			/// \param urdf_model  URDF model.
			/// \param transmissions  Transmissions.
			///
			/// \return  \c true if the simulated robot hardware is initialized successfully, \c false if not.
			virtual bool initSim(
					const std::string& robot_namespace,
					ros::NodeHandle model_nh,
					gazebo::physics::ModelPtr parent_model,
					const urdf::Model *const urdf_model,
					std::vector<transmission_interface::TransmissionInfo> transmissions) override;

			/// \brief Read state data from the simulated robot hardware
			///
			/// Read state data, such as joint positions and velocities, from the simulated robot hardware.
			///
			/// \param time  Simulation time.
			/// \param period  Time since the last simulation step.
			virtual void readSim(ros::Time time, ros::Duration period) override;

			/// \brief Write commands to the simulated robot hardware
			///
			/// Write commands, such as joint position and velocity commands, to the simulated robot hardware.
			///
			/// \param time  Simulation time.
			/// \param period  Time since the last simulation step.
			virtual void writeSim(ros::Time time, ros::Duration period) override;

			/// \brief Set the emergency stop state
			///
			/// Set the simulated robot's emergency stop state. The default implementation of this function does nothing.
			///
			/// \param active  \c true if the emergency stop is active, \c false if not.
			virtual void eStopActive(const bool active) {}
		protected:
			  std::vector<gazebo::physics::JointPtr> sim_joints_;
			  std::string physics_type_;
			  // e_stop_active_ is true if the emergency stop is active.
			  bool e_stop_active_, last_e_stop_active_;
	};
}
