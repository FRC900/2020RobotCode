/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Original Author: Dave Coleman
   Desc:   Example ros_control hardware interface blank template for the FRCRobot
           For a more detailed simulation example, see sim_hw_interface.cpp

	The hardware interface code reads and writes directly from/to hardware
	connected to the RoboRIO. This include DIO, Analog In, pneumatics,
	and CAN Talons, among other things.

	The two main methods are read() and write().

	read() is responsible for reading hardware state and filling in
	a buffered copy of it. This buffered copy of the hardware state
	can be accessed by various controllers to figure out what to do next.

	write() does the opposite. It takes commands that have been buffered
	by various controllers and sends them to the hardware.  The design goal
	here is to minimize redundant writes to the HW.  Previous values written
	are cached, and subsequent writes of the same value are skipped.

	The main read loop actually reads from all hardware except CAN Talons.
	The CAN talon status reads are double buffered. A thread is kicked
	off for each CAN talon.  That thread updates a buffer which is shared
	by the main read loop. The only thing the main read loop does is
	consolidate the data from each thread into a separate state buffer,
	this one externally visible to controllers.  Since reads are the slowest
	part of the process, this decouples hardware read speed from the
	control loop update rate.

	The PDP data also works in a similar way.  There is a thread running
	at a constant rate polling PDP data, and read() picks up the latest
	copy of that data each time through the read/update/write loop
*/

//PURPOSE: File that reads and writes to hardware

#include <cmath>
#include <iostream>

#include <tf2/LinearMath/Matrix3x3.h>
#include "ros_control_boilerplate/frcrobot_hw_interface.h"
#include "ros_control_boilerplate/error_queue.h"

//HAL / wpilib includes
#include <HALInitializer.h>
#include <networktables/NetworkTable.h>
#include <hal/CAN.h>
#include <hal/Compressor.h>
#include <hal/DriverStation.h>
#include <hal/Errors.h>
#include <hal/PDP.h>
#include <hal/Power.h>
#include <hal/Solenoid.h>
#include <frc/Joystick.h>

#include <ctre/phoenix/motorcontrol/SensorCollection.h>
#include <ctre/phoenix/platform/Platform.h>
#include <ctre/phoenix/cci/Unmanaged_CCI.h>

#ifdef __linux__
#include <sched.h>
#endif

//
// digital output, PWM, Pneumatics, compressor, nidec, talons
//    controller on jetson  (local update = true, local hardware = false
//        don't do anything in read
//        random controller updates command in controller
//        set output state var from command in write() on jetson - this will be reflected in joint_states
//          but do not call Set since hardware doesn't exist (local write)
//
//      on rio (local update = false, local hardware = true
//         don't do anything in read
//         update loop needs to read joint_states using joint state listener
//             this writes values from the jetson to each local joint command on the Rio
//         write() sets hardware from those joint commands, and also sets state
//            write needs to set value as - is, don't apply invert,
//            since it was already applied on the remote side
//
//	local_update = true, local hardware = true -> no listener
//
//		This would be for hardware on the Rio which is also modified by controllers running on the Rio
//
//	local_update = false, local hardware = true -> listener to transfer cmd from remote to local
//
//		E.g. config on the Rio if a controller on the Jetson wanted to update hardware on the Rio
//
//	local_update = true, local_hardare = false -> no listener, update local state but don't write to hw
//
//		e.g. config on the Jetson if a controller on the Jetson wanted to update hardware on the rio
//
//	local_update = false, local_hardare = false -> listener to mirror updated state from local?
//
//		nothing is happening on the controller wrt the hardware other than wanting to keep current on status
//		not sure how useful this might be, except in cases like digital in where update==hardware
//		by definition
//
//	So !local_update implies add to remote Interface to run a listener
//
// For analog & digital input and state like PDP, match, joystick, etc, there's only 1 local flag.
// The only cases which make sense are local_update = local_hardware, since the value can only be
// updated by reading the hardware itself.  There, just use a "local" flag.
//
namespace frcrobot_control
{
// Dummy vars are used to create joints which are accessed via variable name
// in the low level control code. So far this is only used for sending data
// to the driver station and back via network tables.

const int pidIdx = 0; //0 for primary closed-loop, 1 for cascaded closed-loop
const int timeoutMs = 0; //If nonzero, function will wait for config success and report an error if it times out. If zero, no blocking or checking is performed

// Constructor. Pass appropriate params to base class constructor,
// initialze robot_ pointer to NULL
FRCRobotHWInterface::FRCRobotHWInterface(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: ros_control_boilerplate::FRCRobotInterface(nh, urdf_model)
	, robot_(nullptr)
	, read_tracer_(nh.getNamespace() + "::read()")
{
}

// Clean up whatever we've created in init()
FRCRobotHWInterface::~FRCRobotHWInterface()
{
	for (size_t i = 0; i < num_can_ctre_mcs_; i++)
	{
		if (can_ctre_mc_local_hardwares_[i])
		{
			ctre_mc_read_threads_[i].join();
		}
	}

	for (size_t i = 0; i < num_solenoids_; i++)
		HAL_FreeSolenoidPort(solenoids_[i]);
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		HAL_FreeSolenoidPort(double_solenoids_[i].forward_);
		HAL_FreeSolenoidPort(double_solenoids_[i].reverse_);
	}

	for (size_t i = 0; i < num_compressors_; i++)
		pcm_thread_[i].join();
	for (size_t i = 0; i < num_pdps_; i++)
		pdp_thread_[i].join();
	for (size_t i = 0; i < num_as726xs_; i++)
		as726x_thread_[i].join();
	for (size_t i = 0; i < num_cancoders_; i++)
		cancoder_read_threads_[i].join();
	for (size_t i = 0; i < num_canifiers_; i++)
		canifier_read_threads_[i].join();
}

bool FRCRobotHWInterface::init(ros::NodeHandle& root_nh, ros::NodeHandle &robot_hw_nh)
{
	// Do base class init. This loads common interface info
	// used by both the real and sim interfaces
	if (!FRCRobotInterface::init(root_nh, robot_hw_nh))
	{
		ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << ": FRCRobotInterface::init() returnd false");
		return false;
	}

	if (run_hal_robot_)
	{
		// Make sure to initialize WPIlib code before creating
		// a CAN Talon object to avoid NIFPGA: Resource not initialized
		// errors? See https://www.chiefdelphi.com/forums/showpost.php?p=1640943&postcount=3
		robot_.reset(new ROSIterativeRobot());
		ds_error_server_ = robot_hw_nh.advertiseService("/frcrobot_rio/ds_error_service", &FRCRobotHWInterface::DSErrorCallback, this);
	}
	else
	{
		// This is for non Rio-based robots.  Call init for the wpilib HAL code
		// we've "borrowed" before using them
		hal::init::InitializeCANAPI();
		hal::init::InitializeCompressor();
		hal::init::InitializePCMInternal();
		hal::init::InitializePDP();
		hal::init::InitializeSolenoid();

		errorQueue = std::make_unique<ErrorQueue>();

		const auto rc = ctre::phoenix::platform::can::SetCANInterface(can_interface_.c_str());
		if (rc != 0)
		{
			HAL_SendError(true, -1, false, "SetCANInterface failed - likely CAN adapter failure", "", "", true);
		}
	}

	can_error_count_ = 0;

	for (size_t i = 0; i < num_can_ctre_mcs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << can_ctre_mc_names_[i] <<
							  (can_ctre_mc_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (can_ctre_mc_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as " << (can_ctre_mc_is_talon_fx_[i] ? "TalonFX" : (can_ctre_mc_is_talon_srx_[i] ? "TalonSRX" : "VictorSPX"))
							  << " CAN id " << can_ctre_mc_can_ids_[i]);

		if (can_ctre_mc_local_hardwares_[i])
		{
			if (can_ctre_mc_is_talon_fx_[i])
				ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonFX>(can_ctre_mc_can_ids_[i]));
			else if (can_ctre_mc_is_talon_srx_[i])
				ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_ctre_mc_can_ids_[i]));
			else
				ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::VictorSPX>(can_ctre_mc_can_ids_[i]));

			ctre_mcs_[i]->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0,
							  ctre::phoenix::motorcontrol::DemandType::DemandType_Neutral, 0);

			// Clear sticky faults
			//safeTalonCall(ctre_mcs_[i]->ClearStickyFaults(timeoutMs), "ClearStickyFaults()");


			// TODO : if the motor controller doesn't initialize - maybe known
			// by -1 from firmware version read - somehow tag
			// the entry in ctre_mcs_[] as uninitialized.
			// This probably should be a fatal error
			ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
								  "\tMotor controller firmware version " << ctre_mcs_[i]->GetFirmwareVersion());

			ctre_mc_read_state_mutexes_.push_back(std::make_shared<std::mutex>());
			ctre_mc_read_thread_states_.push_back(std::make_shared<hardware_interface::TalonHWState>(can_ctre_mc_can_ids_[i]));
			ctre_mc_read_threads_.push_back(std::thread(&FRCRobotHWInterface::ctre_mc_read_thread, this,
										    ctre_mcs_[i], ctre_mc_read_thread_states_[i],
										    ctre_mc_read_state_mutexes_[i],
										    std::make_unique<Tracer>("ctre_mc_read_" + can_ctre_mc_names_[i] + " " + root_nh.getNamespace())));
		}
		else
		{
			// Need to have a CAN talon object created on the Rio
			// for that talon to be enabled.  Don't want to do anything with
			// them, though, so the local flags should be set to false
			// which means both reads and writes will be skipped
			if (run_hal_robot_)
			{
				if (can_ctre_mc_is_talon_fx_[i])
				{
					ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonFX>(can_ctre_mc_can_ids_[i]));
				}
				else if (can_ctre_mc_is_talon_srx_[i])
				{
					ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_ctre_mc_can_ids_[i]));
				}
				else
				{
					ctre_mcs_.push_back(std::make_shared<ctre::phoenix::motorcontrol::can::VictorSPX>(can_ctre_mc_can_ids_[i]));
				}
			}
			else
			{
				// Add a null pointer as the can ctre_mc for this index - no
				// actual local hardware identified for it so nothing to create.
				// Just keep the indexes of all the various can_ctre_mc arrays in sync
				ctre_mcs_.push_back(nullptr);
			}
			ctre_mc_read_state_mutexes_.push_back(nullptr);
			ctre_mc_read_thread_states_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_canifiers_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << canifier_names_[i] <<
							  (canifier_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (canifier_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << canifier_can_ids_[i]);

		if (canifier_local_hardwares_[i])
		{
			canifiers_.push_back(std::make_shared<ctre::phoenix::CANifier>(canifier_can_ids_[i]));
			canifier_read_state_mutexes_.push_back(std::make_shared<std::mutex>());
			canifier_read_thread_states_.push_back(std::make_shared<hardware_interface::canifier::CANifierHWState>(canifier_can_ids_[i]));
			canifier_read_threads_.push_back(std::thread(&FRCRobotHWInterface::canifier_read_thread, this,
										    canifiers_[i], canifier_read_thread_states_[i],
										    canifier_read_state_mutexes_[i],
										    std::make_unique<Tracer>("canifier_read_" + canifier_names_[i] + " " + root_nh.getNamespace())));
		}
		else
		{
			canifiers_.push_back(nullptr);
			canifier_read_state_mutexes_.push_back(nullptr);
			canifier_read_thread_states_.push_back(nullptr);
		}
	}
	for (size_t i = 0; i < num_cancoders_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << cancoder_names_[i] <<
							  (cancoder_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (cancoder_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " at CAN id " << cancoder_can_ids_[i]);

		if (cancoder_local_hardwares_[i])
		{
			cancoders_.push_back(std::make_shared<ctre::phoenix::sensors::CANCoder>(cancoder_can_ids_[i]));
			cancoder_read_state_mutexes_.push_back(std::make_shared<std::mutex>());
			cancoder_read_thread_states_.push_back(std::make_shared<hardware_interface::cancoder::CANCoderHWState>(cancoder_can_ids_[i]));
			cancoder_read_threads_.push_back(std::thread(&FRCRobotHWInterface::cancoder_read_thread, this,
										    cancoders_[i], cancoder_read_thread_states_[i],
										    cancoder_read_state_mutexes_[i],
										    std::make_unique<Tracer>("cancoder_read_" + cancoder_names_[i] + " " + root_nh.getNamespace())));
		}
		else
		{
			cancoders_.push_back(nullptr);
			cancoder_read_state_mutexes_.push_back(nullptr);
			cancoder_read_thread_states_.push_back(nullptr);
		}
	}
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << nidec_brushless_names_[i] <<
							  (nidec_brushless_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (nidec_brushless_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as PWM channel " << nidec_brushless_pwm_channels_[i] <<
							  " / DIO channel " << nidec_brushless_dio_channels_[i] <<
							  " invert " << nidec_brushless_inverts_[i]);

		if (nidec_brushless_local_hardwares_[i])
		{
			nidec_brushlesses_.push_back(std::make_shared<frc::NidecBrushless>(nidec_brushless_pwm_channels_[i], nidec_brushless_dio_channels_[i]));
			nidec_brushlesses_[i]->SetInverted(nidec_brushless_inverts_[i]);
		}
		else
			nidec_brushlesses_.push_back(nullptr);
	}
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << digital_input_names_[i] <<
							  " local = " << digital_input_locals_[i] <<
							  " as Digital Input " << digital_input_dio_channels_[i] <<
							  " invert " << digital_input_inverts_[i]);

		if (digital_input_locals_[i])
		{
			bool need_new_hal_din = true;
			for (size_t j = 0; (j < i) && need_new_hal_din; j++)
			{
				if (digital_input_dio_channels_[i] == digital_input_dio_channels_[j])
				{
					digital_inputs_.push_back(digital_inputs_[j]);
					need_new_hal_din = false;
						ROS_WARN_STREAM("DIn " << digital_input_names_[i] <<
								" uses same channel (" << digital_input_dio_channels_[i] <<
								") as " <<  digital_input_names_[j]);
				}
			}
			if (need_new_hal_din)
				digital_inputs_.push_back(std::make_shared<frc::DigitalInput>(digital_input_dio_channels_[i]));
		}
		else
		{
			digital_inputs_.push_back(nullptr);
		}
	}
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << digital_output_names_[i] <<
							  (digital_output_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (digital_output_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Digital Output " << digital_output_dio_channels_[i] <<
							  " invert " << digital_output_inverts_[i]);

		if (digital_output_local_hardwares_[i])
			digital_outputs_.push_back(std::make_shared<frc::DigitalOutput>(digital_output_dio_channels_[i]));
		else
			digital_outputs_.push_back(nullptr);
	}
	for (size_t i = 0; i < num_pwms_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << pwm_names_[i] <<
							  (pwm_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (pwm_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Digitial Output " << pwm_pwm_channels_[i] <<
							  " invert " << pwm_inverts_[i]);

		if (pwm_local_hardwares_[i])
		{
			PWMs_.push_back(std::make_shared<frc::PWM>(pwm_pwm_channels_[i]));
			PWMs_[i]->SetSafetyEnabled(true);
		}
		else
			PWMs_.push_back(nullptr);
	}
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << solenoid_names_[i] <<
							  (solenoid_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Solenoid " << solenoid_ids_[i]
							  << " with pcm " << solenoid_pcms_[i]);

		// Need to have 1 solenoid instantiated on the Rio to get
		// support for compressor and so on loaded?
		if (solenoid_local_hardwares_[i])
		{
			int32_t status = 0;
			solenoids_.push_back(HAL_InitializeSolenoidPort(HAL_GetPortWithModule(solenoid_pcms_[i], solenoid_ids_[i]), &status));
			if (solenoids_.back() == HAL_kInvalidHandle)
				ROS_ERROR_STREAM("Error intializing solenoid : status=" << status);
			else
				HAL_Report(HALUsageReporting::kResourceType_Solenoid,
						solenoid_ids_[i], solenoid_pcms_[i]);
		}
		else
			solenoids_.push_back(HAL_kInvalidHandle);
	}
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << double_solenoid_names_[i] <<
							  (double_solenoid_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (double_solenoid_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Double Solenoid forward " << double_solenoid_forward_ids_[i] <<
							  " reverse " << double_solenoid_reverse_ids_[i]
							  << " with pcm " << double_solenoid_pcms_[i]);

		if (double_solenoid_local_hardwares_[i])
		{
			int32_t forward_status = 0;
			int32_t reverse_status = 0;
			auto forward_handle = HAL_InitializeSolenoidPort(
					HAL_GetPortWithModule(double_solenoid_pcms_[i], double_solenoid_forward_ids_[i]),
					&forward_status);
			auto reverse_handle = HAL_InitializeSolenoidPort(
					HAL_GetPortWithModule(double_solenoid_pcms_[i], double_solenoid_reverse_ids_[i]),
					&reverse_status);
			if ((forward_handle != HAL_kInvalidHandle) &&
			    (reverse_handle != HAL_kInvalidHandle) )
			{
				double_solenoids_.push_back(DoubleSolenoidHandle(forward_handle, reverse_handle));
				HAL_Report(HALUsageReporting::kResourceType_Solenoid,
						double_solenoid_forward_ids_[i], double_solenoid_pcms_[i]);
				HAL_Report(HALUsageReporting::kResourceType_Solenoid,
						double_solenoid_reverse_ids_[i], double_solenoid_pcms_[i]);
			}
			else
			{
				ROS_ERROR_STREAM("Error intializing double solenoid : status=" << forward_status << " : " << reverse_status);
				double_solenoids_.push_back(DoubleSolenoidHandle(HAL_kInvalidHandle, HAL_kInvalidHandle));
				HAL_FreeSolenoidPort(forward_handle);
				HAL_FreeSolenoidPort(reverse_handle);
			}
		}
		else
		{
			double_solenoids_.push_back(DoubleSolenoidHandle(HAL_kInvalidHandle, HAL_kInvalidHandle));
		}
	}

	//RIGHT NOW THIS WILL ONLY WORK IF THERE IS ONLY ONE NAVX INSTANTIATED
	for(size_t i = 0; i < num_navX_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
				"Loading joint " << i << "=" << navX_names_[i] <<
				" as navX id " << navX_ids_[i] <<
				" local = " << navX_locals_[i]);
		//TODO: fix how we use ids

		if (navX_locals_[i])
			navXs_.push_back(std::make_shared<AHRS>(frc::SPI::Port::kMXP));
		else
			navXs_.push_back(nullptr);

		// This is a guess so TODO : get better estimates
		imu_orientation_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		imu_angular_velocity_covariances_[i] = {0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		imu_linear_acceleration_covariances_[i] ={0.0015, 0.0, 0.0, 0.0, 0.0015, 0.0, 0.0, 0.0, 0.0015};
		break; // TODO : only support 1 for now - if we need more, need to define
		       // the interface in config files somehow
	}
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << analog_input_names_[i] <<
							  " local = " << analog_input_locals_[i] <<
							  " as Analog Input " << analog_input_analog_channels_[i]);
		if (analog_input_locals_[i])
		{
			bool need_new_hal_ain = true;
			for (size_t j = 0; (j < i) && need_new_hal_ain; j++)
			{
				if (analog_input_analog_channels_[i] == analog_input_analog_channels_[j])
				{
					analog_inputs_.push_back(analog_inputs_[j]);
					need_new_hal_ain = false;
						ROS_WARN_STREAM("AIn " << analog_input_names_[i] <<
								" uses same channel (" << analog_input_analog_channels_[i] <<
								") as " <<  analog_input_names_[j]);
				}
			}
			if (need_new_hal_ain)
				analog_inputs_.push_back(std::make_shared<frc::AnalogInput>(analog_input_analog_channels_[i]));
		}
		else
			analog_inputs_.push_back(nullptr);
	}
	for (size_t i = 0; i < num_compressors_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << compressor_names_[i] <<
							  (compressor_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (compressor_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Compressor with pcm " << compressor_pcm_ids_[i]);

		pcm_read_thread_state_.push_back(std::make_shared<hardware_interface::PCMState>(compressor_pcm_ids_[i]));
		if (compressor_local_hardwares_[i])
		{
			if (!HAL_CheckCompressorModule(compressor_pcm_ids_[i]))
			{
				ROS_ERROR("Invalid Compressor PCM ID");
				compressors_.push_back(HAL_kInvalidHandle);
			}
			else
			{
				int32_t status = 0;
				compressors_.push_back(HAL_InitializeCompressor(compressor_pcm_ids_[i], &status));
				if (!status && (compressors_[i] != HAL_kInvalidHandle))
				{
					pcm_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
					pcm_thread_.push_back(std::thread(&FRCRobotHWInterface::pcm_read_thread, this,
								compressors_[i], compressor_pcm_ids_[i], pcm_read_thread_state_[i],
								pcm_read_thread_mutexes_[i],
								std::make_unique<Tracer>("PCM " + compressor_names_[i] + " " + root_nh.getNamespace())));
					HAL_Report(HALUsageReporting::kResourceType_Compressor, compressor_pcm_ids_[i]);
				}
				else
				{
					ROS_ERROR_STREAM("compressor init error : status = "
							<< status << ":" << HAL_GetErrorMessage(status));
				}
			}
		}
		else
			compressors_.push_back(HAL_kInvalidHandle);
	}

	// No real init needed here, just report the config loaded for them
	for (size_t i = 0; i < num_rumbles_; i++)
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << rumble_names_[i] <<
							  (rumble_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (rumble_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as Rumble with port" << rumble_ports_[i]);

	for (size_t i = 0; i < num_pdps_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << pdp_names_[i] <<
							  " local = " << pdp_locals_[i] <<
							  " as PDP");

		if (pdp_locals_[i])
		{
			if (!HAL_CheckPDPModule(pdp_modules_[i]))
			{
				ROS_ERROR("Invalid PDP module number");
				pdps_.push_back(HAL_kInvalidHandle);
			}
			else
			{
				int32_t status = 0;
				pdps_.push_back(HAL_InitializePDP(pdp_modules_[i], &status));
				pdp_read_thread_state_.push_back(std::make_shared<hardware_interface::PDPHWState>());
				if (pdps_[i] == HAL_kInvalidHandle)
				{
					ROS_ERROR_STREAM("Could not initialize PDP module, status = " << status);
				}
				else
				{
					pdp_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
					pdp_thread_.push_back(std::thread(&FRCRobotHWInterface::pdp_read_thread, this,
										  pdps_[i], pdp_read_thread_state_[i], pdp_read_thread_mutexes_[i],
										  std::make_unique<Tracer>("PDP " + pdp_names_[i] + " " + root_nh.getNamespace())));
					HAL_Report(HALUsageReporting::kResourceType_PDP, pdp_modules_[i]);
				}
			}
		}
		else
			pdps_.push_back(HAL_kInvalidHandle);
	}

	for (size_t i = 0; i < num_joysticks_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << joystick_names_[i] <<
							  " local = " << joystick_locals_[i] <<
							  " as joystick with ID " << joystick_ids_[i]);
		if (joystick_locals_[i])
		{
			joysticks_.push_back(std::make_shared<frc::Joystick>(joystick_ids_[i]));
			std::stringstream pub_name;
			// TODO : maybe use pub_names instead, or joy id unconditionally?
			pub_name << "js";
			pub_name << joystick_ids_[i];
			realtime_pub_joysticks_.push_back(std::make_unique<realtime_tools::RealtimePublisher<sensor_msgs::Joy>>(root_nh, pub_name.str(), 1));
		}
		else
		{
			joysticks_.push_back(nullptr);
			realtime_pub_joysticks_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_as726xs_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading as726x joint " << i << "=" << as726x_names_[i] <<
							  (as726x_local_updates_[i] ? " local" : " remote") << " update, " <<
							  (as726x_local_hardwares_[i] ? "local" : "remote") << " hardware" <<
							  " as as726x with port=" << as726x_ports_[i] <<
							  " address=" << as726x_addresses_[i]);
		if (as726x_local_hardwares_[i])
		{
			frc::I2C::Port port;
			if (as726x_ports_[i] == "onboard")
				port = frc::I2C::Port::kOnboard;
			else if (as726x_ports_[i] == "mxp")
				port = frc::I2C::Port::kMXP;
			else
			{
				ROS_ERROR_STREAM("Invalid port specified for as726x - " <<
						as726x_ports_[i] << "valid options are onboard and mxp");
				return false;
			}

			as726xs_.push_back(std::make_shared<as726x::roboRIO_AS726x>(port, as726x_addresses_[i]));
			if (as726xs_.back()->begin())
			{
					as726x_read_thread_state_.push_back(std::make_shared<hardware_interface::as726x::AS726xState>(as726x_ports_[i], as726x_addresses_[i]));
					as726x_read_thread_mutexes_.push_back(std::make_shared<std::mutex>());
					as726x_thread_.push_back(std::thread(&FRCRobotHWInterface::as726x_read_thread, this,
								as726xs_[i],
								as726x_read_thread_state_[i],
								as726x_read_thread_mutexes_[i],
								std::make_unique<Tracer>("AS726x:" + as726x_names_[i] + " " + root_nh.getNamespace())));
			}
			else
			{
				ROS_ERROR_STREAM("Error intializing as726x");
				as726xs_.push_back(nullptr);
				as726x_read_thread_state_.push_back(nullptr);
				as726x_read_thread_mutexes_.push_back(nullptr);
			}
		}
		else
		{
			as726xs_.push_back(nullptr);
			as726x_read_thread_state_.push_back(nullptr);
			as726x_read_thread_mutexes_.push_back(nullptr);
		}
	}

	for (size_t i = 0; i < num_talon_orchestras_; i++)
	{
		ROS_INFO_STREAM_NAMED("frcrobot_hw_interface",
							  "Loading joint " << i << "=" << talon_orchestra_names_[i]);

                talon_orchestras_.push_back(std::make_shared<ctre::phoenix::music::Orchestra>());
	}

	const double t_now = ros::Time::now().toSec();

	t_prev_robot_iteration_ = t_now;
	if(! root_nh.getParam("generic_hw_control_loop/robot_iteration_hz", robot_iteration_hz_)) {
		ROS_ERROR("Failed to read robot_iteration_hz in frcrobot_hw_interface");
		robot_iteration_hz_ = 20;
	}

	t_prev_joystick_read_ = t_now;
	if(! root_nh.getParam("generic_hw_control_loop/joystick_read_hz", joystick_read_hz_)) {
		ROS_ERROR("Failed to read joystick_read_hz in frcrobot_hw_interface");
		joystick_read_hz_ = 50;
	}

	t_prev_match_data_read_ = t_now;
	if(! root_nh.getParam("generic_hw_control_loop/match_data_read_hz", match_data_read_hz_)) {
		ROS_ERROR("Failed to read match_data_read_hz in frcrobot_hw_interface");
		match_data_read_hz_ = 2;
	}

	t_prev_robot_controller_read_ = t_now;
	if(! root_nh.getParam("generic_hw_control_loop/robot_controller_read_hz", robot_controller_read_hz_)) {
		ROS_ERROR("Failed to read robot_controller_read_hz in frcrobot_hw_interface");
		robot_controller_read_hz_ = 20;
	}

#ifdef __linux__
	struct sched_param schedParam{};

	schedParam.sched_priority = sched_get_priority_min(SCHED_RR);
	auto rc = pthread_setschedparam(pthread_self(), SCHED_RR, &schedParam);
	ROS_INFO_STREAM("pthread_setschedparam() returned " << rc
			<< " priority = " << schedParam.sched_priority
			<< " errno = " << errno << " (" << strerror(errno) << ")");
	pthread_setname_np(pthread_self(), "hwi_main_loop");
#endif

	ROS_INFO_STREAM(robot_hw_nh.getNamespace() << " : FRCRobotHWInterface Ready.");
	HAL_SendError(true, 0, false, std::string("(Not an error) " + robot_hw_nh.getNamespace() + " : FRCRobotHWInterface Ready").c_str(), "", "", true);
	return true;
}

// Each talon/victor gets their own read thread. The thread loops at a fixed rate
// reading all state from that talon/victor. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotHWInterface::ctre_mc_read_thread(std::shared_ptr<ctre::phoenix::motorcontrol::IMotorController> ctre_mc,
											std::shared_ptr<hardware_interface::TalonHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "ctre_mc_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate rate(100); // TODO : configure me from a file or
						 // be smart enough to run at the rate of the fastest status update?

	auto talon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonSRX>(ctre_mc);
	auto victor = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorController>(ctre_mc);

	while(ros::ok())
	{
		tracer->start("talon read main_loop");

		hardware_interface::TalonMode talon_mode;
		hardware_interface::FeedbackDevice encoder_feedback;
		int encoder_ticks_per_rotation;
		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			if (!state->getEnableReadThread())
				return;
			talon_mode = state->getTalonMode();
			encoder_feedback = state->getEncoderFeedback();
			encoder_ticks_per_rotation = state->getEncoderTicksPerRotation();
			conversion_factor = state->getConversionFactor();
		}

		// TODO : in main read() loop copy status from talon being followed
		// into follower talon state?
		if (talon_mode == hardware_interface::TalonMode_Follower)
			return;

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, hardware_interface::TalonMode_Velocity) * conversion_factor;

		const double motor_output_percent = victor->GetMotorOutputPercent();
		safeTalonCall(victor->GetLastError(), "GetMotorOutputPercent");

		ctre::phoenix::motorcontrol::Faults faults;
		safeTalonCall(victor->GetFaults(faults), "GetFaults");

		// applied control mode - cached
		// soft limit and limit switch override - cached

		const double position = victor->GetSelectedSensorPosition(pidIdx) * radians_scale;
		safeTalonCall(victor->GetLastError(), "GetSelectedSensorPosition");

		const double velocity = victor->GetSelectedSensorVelocity(pidIdx) * radians_per_second_scale;
		safeTalonCall(victor->GetLastError(), "GetSelectedSensorVelocity");

		double output_current = -1;
		if (talon)
		{
			output_current = talon->GetOutputCurrent();
			safeTalonCall(victor->GetLastError(), "GetOutputCurrent");
		}

		ctre::phoenix::motorcontrol::StickyFaults sticky_faults;
		safeTalonCall(victor->GetStickyFaults(sticky_faults), "GetStickyFault");

		// Temp / Voltage status 4 == 160 mSec default
		const double bus_voltage = victor->GetBusVoltage();
		safeTalonCall(victor->GetLastError(), "GetBusVoltage");

		const double temperature = victor->GetTemperature(); //returns in Celsius
		safeTalonCall(victor->GetLastError(), "GetTemperature");

		// TODO : not sure about this one being in status 4
		const double output_voltage = victor->GetMotorOutputVoltage();
		safeTalonCall(victor->GetLastError(), "GetMotorOutputVoltage");

		double closed_loop_error = 0;
		double integral_accumulator = 0;
		double error_derivative = 0;
		double closed_loop_target = 0;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
			(talon_mode == hardware_interface::TalonMode_Velocity) ||
			(talon_mode == hardware_interface::TalonMode_Current ) ||
			(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
			(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, encoder_feedback, talon_mode) * conversion_factor;

			closed_loop_error = victor->GetClosedLoopError(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetClosedLoopError");

			integral_accumulator = victor->GetIntegralAccumulator(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetIntegralAccumulator");

			error_derivative = victor->GetErrorDerivative(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetErrorDerivative");

			closed_loop_target = victor->GetClosedLoopTarget(pidIdx) * closed_loop_scale;
			safeTalonCall(victor->GetLastError(), "GetClosedLoopTarget");
			state->setClosedLoopTarget(closed_loop_target);

			// Reverse engineer the individual P,I,D,F components used
			// to generate closed-loop control signals to the motor
			// This is just for debugging PIDF tuning
			const int pidf_slot = state->getSlot();
			const double kp = state->getPidfP(pidf_slot);
			const double ki = state->getPidfI(pidf_slot);
			const double kd = state->getPidfD(pidf_slot);
			const double kf = state->getPidfF(pidf_slot);

			const double native_closed_loop_error = closed_loop_error / closed_loop_scale;
			state->setPTerm(native_closed_loop_error * kp);
			state->setITerm(integral_accumulator * ki);
			state->setDTerm(error_derivative * kd);
			state->setFTerm(closed_loop_target / closed_loop_scale * kf);
		}

		// Targets Status 10 - 160 mSec default
		double active_trajectory_position = 0.0;
		double active_trajectory_velocity = 0.0;
		double active_trajectory_heading = 0.0;
		if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
			(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
			(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			active_trajectory_position = victor->GetActiveTrajectoryPosition() * radians_scale;
			safeTalonCall(victor->GetLastError(), "GetActiveTrajectoryPosition");

			active_trajectory_velocity = victor->GetActiveTrajectoryVelocity() * radians_per_second_scale;
			safeTalonCall(victor->GetLastError(), "GetActiveTrajectoryVelocity");

			if (talon_mode == hardware_interface::TalonMode_MotionProfileArc)
			{
				active_trajectory_heading = victor->GetActiveTrajectoryPosition(1) * 2. * M_PI / 360.; //returns in degrees
				safeTalonCall(victor->GetLastError(), "GetActiveTrajectoryHeading");
			}
		}

		int mp_top_level_buffer_count = 0;
		hardware_interface::MotionProfileStatus internal_status;
		if (talon_mode == hardware_interface::TalonMode_MotionProfile)
		{
			mp_top_level_buffer_count = victor->GetMotionProfileTopLevelBufferCount();
			ctre::phoenix::motion::MotionProfileStatus talon_status;
			safeTalonCall(victor->GetMotionProfileStatus(talon_status), "GetMotionProfileStatus");

			internal_status.topBufferRem = talon_status.topBufferRem;
			internal_status.topBufferCnt = talon_status.topBufferCnt;
			internal_status.btmBufferCnt = talon_status.btmBufferCnt;
			internal_status.hasUnderrun = talon_status.hasUnderrun;
			internal_status.isUnderrun = talon_status.isUnderrun;
			internal_status.activePointValid = talon_status.activePointValid;
			internal_status.isLast = talon_status.isLast;
			internal_status.profileSlotSelect0 = talon_status.profileSlotSelect0;
			internal_status.profileSlotSelect1 = talon_status.profileSlotSelect1;
			internal_status.outputEnable = static_cast<hardware_interface::SetValueMotionProfile>(talon_status.outputEnable);
			internal_status.timeDurMs = talon_status.timeDurMs;
		}

		// SensorCollection - 160msec(?) default
		bool forward_limit_switch = false;
		bool reverse_limit_switch = false;
		if (talon)
		{
			auto sensor_collection = talon->GetSensorCollection();

			forward_limit_switch = sensor_collection.IsFwdLimitSwitchClosed();
			reverse_limit_switch = sensor_collection.IsRevLimitSwitchClosed();
		}

		const int firmware_version = victor->GetFirmwareVersion();

		// Actually update the TalonHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);

			if (talon_mode == hardware_interface::TalonMode_MotionProfile)
			{
				state->setMotionProfileStatus(internal_status);
				state->setMotionProfileTopLevelBufferCount(mp_top_level_buffer_count);
			}

			state->setMotorOutputPercent(motor_output_percent);
			state->setFaults(faults.ToBitfield());

			state->setForwardSoftlimitHit(faults.ForwardSoftLimit);
			state->setReverseSoftlimitHit(faults.ReverseSoftLimit);

			state->setPosition(position);
			state->setSpeed(velocity);
			if (talon)
				state->setOutputCurrent(output_current);
			state->setStickyFaults(sticky_faults.ToBitfield());

			state->setBusVoltage(bus_voltage);
			state->setTemperature(temperature);
			state->setOutputVoltage(output_voltage);

			if ((talon_mode == hardware_interface::TalonMode_Position) ||
				(talon_mode == hardware_interface::TalonMode_Velocity) ||
				(talon_mode == hardware_interface::TalonMode_Current ) ||
				(talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
			{
				state->setClosedLoopError(closed_loop_error);
				state->setIntegralAccumulator(integral_accumulator);
				state->setErrorDerivative(error_derivative);
				if ((talon_mode != hardware_interface::TalonMode_MotionProfile) &&
					(talon_mode != hardware_interface::TalonMode_MotionMagic) &&
					(talon_mode != hardware_interface::TalonMode_MotionProfileArc))
				{
					state->setClosedLoopTarget(closed_loop_target);
				}
			}

			if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
			{
				state->setActiveTrajectoryPosition(active_trajectory_position);
				state->setActiveTrajectoryVelocity(active_trajectory_velocity);
				if (talon_mode == hardware_interface::TalonMode_MotionProfileArc)
				{
					state->setActiveTrajectoryHeading(active_trajectory_heading);
				}
			}

			if (talon)
			{
				state->setForwardLimitSwitch(forward_limit_switch);
				state->setReverseLimitSwitch(reverse_limit_switch);
			}

			state->setFirmwareVersion(firmware_version);
		}
		tracer->stop();
		ROS_INFO_STREAM_THROTTLE(60, tracer->report());
		rate.sleep();
	}
}

// Each canifier gets their own read thread. The thread loops at a fixed rate
// reading all state from that canifier. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotHWInterface::canifier_read_thread(std::shared_ptr<ctre::phoenix::CANifier> canifier,
											std::shared_ptr<hardware_interface::canifier::CANifierHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "canifier_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate rate(100); // TODO : configure me from a file or
						 // be smart enough to run at the rate of the fastest status update?

	while(ros::ok())
	{
		tracer->start("canifier read main_loop");

		int encoder_ticks_per_rotation;
		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			encoder_ticks_per_rotation = state->getEncoderTicksPerRotation();
			conversion_factor = state->getConversionFactor();
		}

		std::array<bool, hardware_interface::canifier::GeneralPin::GeneralPin_LAST> general_pins;
		for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
		{
			ctre::phoenix::CANifier::GeneralPin ctre_general_pin;
			if (canifier_convert_.generalPin(static_cast<hardware_interface::canifier::GeneralPin>(i), ctre_general_pin))
				general_pins[i] = canifier->GetGeneralInput(ctre_general_pin);
		}

		// Use FeedbackDevice_QuadEncoder to force getConversionFactor to use the encoder_ticks_per_rotation
		// variable to calculate these values
		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Velocity) * conversion_factor;
		const double quadrature_position = canifier->GetQuadraturePosition() * radians_scale;
		const double quadrature_velocity = canifier->GetQuadratureVelocity() * radians_per_second_scale;
		const double bus_voltage      = canifier->GetBusVoltage();

		std::array<std::array<double, 2>, hardware_interface::canifier::PWMChannel::PWMChannelLast> pwm_inputs;
		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			if (canifier_convert_.PWMChannel(static_cast<hardware_interface::canifier::PWMChannel>(i), ctre_pwm_channel))
				canifier->GetPWMInput(ctre_pwm_channel, &pwm_inputs[i][0]);
		}

		ctre::phoenix::CANifierFaults ctre_faults;
		canifier->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::CANifierStickyFaults ctre_sticky_faults;
		canifier->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		// Actually update the CANifierHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);

			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
			{
				state->setGeneralPinInput(static_cast<hardware_interface::canifier::GeneralPin>(i), general_pins[i]);
			}

			state->setQuadraturePosition(quadrature_position);
			state->setQuadratureVelocity(quadrature_velocity);
			state->setBusVoltage(bus_voltage);

			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				state->setPWMInput(static_cast<hardware_interface::canifier::PWMChannel>(i), pwm_inputs[i]);
			}

			state->setFaults(faults);
			state->setStickyFaults(sticky_faults);
		}
		tracer->stop();
		ROS_INFO_STREAM_THROTTLE(60, tracer->report());
		rate.sleep();
	}
}

// Each cancoder gets their own read thread. The thread loops at a fixed rate
// reading all state from that cancoder. The state is copied to a shared buffer
// at the end of each iteration of the loop.
// The code tries to only read status when we expect there to be new
// data given the update rate of various CAN messages.
void FRCRobotHWInterface::cancoder_read_thread(std::shared_ptr<ctre::phoenix::sensors::CANCoder> cancoder,
											std::shared_ptr<hardware_interface::cancoder::CANCoderHWState> state,
											std::shared_ptr<std::mutex> mutex,
											std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "cancoder_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate rate(100); // TODO : configure me from a file or
						 // be smart enough to run at the rate of the fastest status update?

	while(ros::ok())
	{
		tracer->start("cancoder read main_loop");

		double conversion_factor;

		// Update local status with relevant global config
		// values set by write(). This way, items configured
		// by controllers will be reflected in the state here
		// used when reading from talons.
		// Realistically they won't change much (except maybe mode)
		// but unless it causes performance problems reading them
		// each time through the loop is easier than waiting until
		// they've been correctly set by write() before using them
		// here.
		// Note that this isn't a complete list - only the values
		// used by the read thread are copied over.  Update
		// as needed when more are read
		{
			std::lock_guard<std::mutex> l(*mutex);
			conversion_factor = state->getConversionFactor();
		}
		//TODO redo using feedback coefficent

		// Use FeedbackDevice_QuadEncoder to force getConversionFactor to use the encoder_ticks_per_rotation
		// variable to calculate these values
		const double position = cancoder->GetPosition() * conversion_factor;
		const double velocity = cancoder->GetVelocity() * conversion_factor;
		const double absolute_position = cancoder->GetAbsolutePosition() * conversion_factor;
		const double bus_voltage = cancoder->GetBusVoltage();
		const auto ctre_magnet_field_strength = cancoder->GetMagnetFieldStrength();
		hardware_interface::cancoder::MagnetFieldStrength magnet_field_strength;
		cancoder_convert_.magnetFieldStrength(ctre_magnet_field_strength, magnet_field_strength);
		const double last_timestamp = cancoder->GetLastTimestamp();
		const int firmware_version = cancoder->GetFirmwareVersion();

		ctre::phoenix::sensors::CANCoderFaults ctre_faults;
		cancoder->GetFaults(ctre_faults);
		const unsigned faults = ctre_faults.ToBitfield();
		ctre::phoenix::sensors::CANCoderStickyFaults ctre_sticky_faults;
		cancoder->GetStickyFaults(ctre_sticky_faults);
		const unsigned sticky_faults = ctre_sticky_faults.ToBitfield();

		// Actually update the CANCoderHWState shared between
		// this thread and read()
		// Do this all at once so the code minimizes the amount
		// of time with mutex locked
		{
			// Lock the state entry to make sure writes
			// are atomic - reads won't grab data in
			// the middle of a write
			std::lock_guard<std::mutex> l(*mutex);
			state->setPosition(position);
			state->setVelocity(velocity);
			state->setAbsolutePosition(absolute_position);
			state->setBusVoltage(bus_voltage);
			state->setMagnetFieldStrength(magnet_field_strength);
			state->setLastTimestamp(last_timestamp);
			state->setFirmwareVersion(firmware_version);
			state->setFaults(faults);
			state->setStickyFaults(sticky_faults);
		}
		tracer->stop();
		ROS_INFO_STREAM_THROTTLE(60, tracer->report());
		rate.sleep();
	}
}
// The PDP reads happen in their own thread. This thread
// loops at 20Hz to match the update rate of PDP CAN
// status messages.  Each iteration, data read from the
// PDP is copied to a state buffer shared with the main read
// thread.
void FRCRobotHWInterface::pdp_read_thread(int32_t pdp,
		std::shared_ptr<hardware_interface::PDPHWState> state,
		std::shared_ptr<std::mutex> mutex,
		std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "pdp_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate r(20); // TODO : Tune me?
	int32_t status = 0;
	HAL_ClearPDPStickyFaults(pdp, &status);
	HAL_ResetPDPTotalEnergy(pdp, &status);
	if (status)
		ROS_ERROR_STREAM("pdp_read_thread error clearing sticky faults : status = " << status);
	while (ros::ok())
	{
		tracer->start("main loop");

		//read info from the PDP hardware
		status = 0;
		hardware_interface::PDPHWState pdp_state;
		pdp_state.setVoltage(HAL_GetPDPVoltage(pdp, &status));
		pdp_state.setTemperature(HAL_GetPDPTemperature(pdp, &status));
		pdp_state.setTotalCurrent(HAL_GetPDPTotalCurrent(pdp, &status));
		pdp_state.setTotalPower(HAL_GetPDPTotalPower(pdp, &status));
		pdp_state.setTotalEnergy(HAL_GetPDPTotalEnergy(pdp, &status));
		for (int channel = 0; channel <= 15; channel++)
		{
			pdp_state.setCurrent(HAL_GetPDPChannelCurrent(pdp, channel, &status), channel);
		}
		if (status)
			ROS_ERROR_STREAM("pdp_read_thread error : status = " << status << ":" << HAL_GetErrorMessage(status));

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard<std::mutex> l(*mutex);
			*state = pdp_state;
		}

		tracer->stop();
		ROS_INFO_STREAM_THROTTLE(60, tracer->report());
		r.sleep();
	}
}

// The PCM state reads happen in their own thread. This thread
// loops at 20Hz to match the update rate of PCM CAN
// status messages.  Each iteration, data read from the
// PCM is copied to a state buffer shared with the main read
// thread.
void FRCRobotHWInterface::pcm_read_thread(HAL_CompressorHandle compressor_handle, int32_t pcm_id,
										  std::shared_ptr<hardware_interface::PCMState> state,
										  std::shared_ptr<std::mutex> mutex,
										  std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "pcm_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let CAN start up
	ros::Rate r(20); // TODO : Tune me?
	int32_t status = 0;
	HAL_ClearAllPCMStickyFaults(pcm_id, &status);
	if (status)
	{
		ROS_ERROR_STREAM("pcm_read_thread error clearing sticky faults : status = "
				<< status << ":" << HAL_GetErrorMessage(status));
	}
	while (ros::ok())
	{
		tracer->start("main loop");

		hardware_interface::PCMState pcm_state(pcm_id);
		status = 0;
		pcm_state.setEnabled(HAL_GetCompressor(compressor_handle, &status));
		pcm_state.setPressureSwitch(HAL_GetCompressorPressureSwitch(compressor_handle, &status));
		pcm_state.setCompressorCurrent(HAL_GetCompressorCurrent(compressor_handle, &status));
		pcm_state.setClosedLoopControl(HAL_GetCompressorClosedLoopControl(compressor_handle, &status));
		pcm_state.setCurrentTooHigh(HAL_GetCompressorCurrentTooHighFault(compressor_handle, &status));
		pcm_state.setCurrentTooHighSticky(HAL_GetCompressorCurrentTooHighStickyFault(compressor_handle, &status));

		pcm_state.setShorted(HAL_GetCompressorShortedFault(compressor_handle, &status));
		pcm_state.setShortedSticky(HAL_GetCompressorShortedStickyFault(compressor_handle, &status));
		pcm_state.setNotConntected(HAL_GetCompressorNotConnectedFault(compressor_handle, &status));
		pcm_state.setNotConnecteSticky(HAL_GetCompressorNotConnectedStickyFault(compressor_handle, &status));
		pcm_state.setVoltageFault(HAL_GetPCMSolenoidVoltageFault(pcm_id, &status));
		pcm_state.setVoltageStickFault(HAL_GetPCMSolenoidVoltageStickyFault(pcm_id, &status));
		pcm_state.setSolenoidBlacklist(HAL_GetPCMSolenoidBlackList(pcm_id, &status));

		if (status)
			ROS_ERROR_STREAM("pcm_read_thread : status = " << status << ":" << HAL_GetErrorMessage(status));

		{
			// Copy to state shared with read() thread
			// Put this in a separate scope so lock_guard is released
			// as soon as the state is finished copying
			std::lock_guard<std::mutex> l(*mutex);
			*state = pcm_state;
		}

		tracer->stop();
		ROS_INFO_STREAM_THROTTLE(60, tracer->report());
		r.sleep();
	}
}

// The AS726x state reads happen in their own thread. This thread
// loops at XHz to match the update rate of PCM CAN
// status messages.  Each iteration, data read from the
// AS726x color sensor is copied to a state buffer shared with the main read
// thread.
void FRCRobotHWInterface::as726x_read_thread(
		std::shared_ptr<as726x::roboRIO_AS726x> as726x,
		std::shared_ptr<hardware_interface::as726x::AS726xState> state,
		std::shared_ptr<std::mutex> mutex,
		std::unique_ptr<Tracer> tracer)
{
#ifdef __linux__
	pthread_setname_np(pthread_self(), "as726x_read");
#endif
	ros::Duration(2).sleep(); // Sleep for a few seconds to let I2C start up
	ros::Rate r(7); // TODO : Tune me?

	uint16_t temperature;
	std::array<uint16_t, 6> raw_channel_data;
	std::array<float, 6> calibrated_channel_data;
	while (ros::ok())
	{
		tracer->start("main loop");
		// Create a scope for the lock protecting hardware accesses
		{
			std::lock_guard<std::mutex> l(*(as726x->getMutex()));
			temperature = as726x->readTemperature();
			as726x->startMeasurement();
			auto data_ready_start = ros::Time::now();
			bool timeout = false;
			while (!as726x->dataReady() && !timeout)
			{
				ros::Duration(0.01).sleep();
				if ((ros::Time::now() - data_ready_start).toSec() > 1.5)
				{
					timeout = true;
				}
			}
			if (timeout)
			{
				ROS_WARN("Timeout waiting for as726 data ready");
				tracer->stop();
				continue;
			}

			as726x->readRawValues(&raw_channel_data[0], 6);
			as726x->readCalibratedValues(&calibrated_channel_data[0], 6);
		}
		// Create a new scope for the lock protecting the shared state
		{
			std::lock_guard<std::mutex> l(*mutex);
			state->setTemperature(temperature);
			state->setRawChannelData(raw_channel_data);
			state->setCalibratedChannelData(calibrated_channel_data);
		}
		tracer->stop();
		ROS_INFO_STREAM_THROTTLE(60, tracer->report());
		ros::Duration(0.1).sleep(); // allow time for write() to run
		r.sleep();
	}
}

void FRCRobotHWInterface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
	read_tracer_.start_unique("Check for ready");
	if (run_hal_robot_ && !robot_code_ready_)
	{
		// This will be written by the last controller to be
		// spawned - waiting here prevents the robot from
		// reporting robot code ready to the field until
		// all other controllers are started
		if (std::all_of(robot_ready_signals_.cbegin(),
						robot_ready_signals_.cend(),
						[](double d) { return d != 0.0;} ))
		{
			robot_->StartCompetition();
			robot_code_ready_ = true;
		}
	}

	if (robot_code_ready_)
	{
		read_tracer_.start_unique("OneIteration");
		//check if sufficient time has passed since last read
		if(ros::Time::now().toSec() - t_prev_robot_iteration_ > (1/robot_iteration_hz_))
		{
			robot_->OneIteration();

			t_prev_robot_iteration_ += 1/robot_iteration_hz_;
		}

		read_tracer_.start_unique("joysticks");
		//check if sufficient time has passed since last read
		if(ros::Time::now().toSec() - t_prev_joystick_read_ > (1/joystick_read_hz_))
		{
			t_prev_joystick_read_ += 1/joystick_read_hz_;

			auto time_now_t = ros::Time::now();
			for (size_t i = 0; i < num_joysticks_; i++)
			{
				if (realtime_pub_joysticks_[i]->trylock())
				{
					auto &m = realtime_pub_joysticks_[i]->msg_;
					m.header.stamp = time_now_t;

					m.axes.clear();
					m.buttons.clear();

					for(int j = 0; j < joysticks_[i]->GetAxisCount(); j++)
					{
						m.axes.push_back(joysticks_[i]->GetRawAxis(j));
					}

					for(int j = 0; j < joysticks_[i]->GetButtonCount(); j++)
					{
						m.buttons.push_back(joysticks_[i]->GetRawButton(j+1));
					}

					bool direction_up = false;
					bool direction_down = false;
					bool direction_left = false;
					bool direction_right = false;
					switch (joysticks_[i]->GetPOV(0))
					{
						case 0 :
							direction_up = true;
							break;
						case 45:
							direction_up = true;
							direction_right = true;
							break;
						case 90:
							direction_right = true;
							break;
						case 135:
							direction_down = true;
							direction_right = true;
							break;
						case 180:
							direction_down = true;
							break;
						case 225:
							direction_down = true;
							direction_left = true;
							break;
						case 270:
							direction_left = true;
							break;
						case 315:
							direction_up = true;
							direction_left = true;
							break;
					}

					if(direction_left)
					{
						m.axes.push_back(1.0);
					}
					else if (direction_right)
					{
						m.axes.push_back(-1.0);
					}
					else
					{
						m.axes.push_back(0.0);
					}

					if(direction_up)
					{
						m.axes.push_back(1.0);
					}
					else if (direction_down)
					{
						m.axes.push_back(-1.0);
					}
					else
					{
						m.axes.push_back(0.0);
					}
					realtime_pub_joysticks_[i]->unlockAndPublish();
				}
			}

		}

		read_tracer_.start_unique("match data");
		int32_t status = 0;
		//check if sufficient time has passed since last read
		if(ros::Time::now().toSec() - t_prev_match_data_read_ > (1/match_data_read_hz_))
		{
			t_prev_match_data_read_ += 1/match_data_read_hz_;

			status = 0;
			match_data_.setMatchTimeRemaining(HAL_GetMatchTime(&status));
			match_data_.setGetMatchTimeStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
			HAL_MatchInfo info;
			HAL_GetMatchInfo(&info);

			match_data_.setGameSpecificData(std::string(reinterpret_cast<char*>(info.gameSpecificMessage),
						info.gameSpecificMessageSize));
			match_data_.setEventName(info.eventName);

			status = 0;
			auto allianceStationID = HAL_GetAllianceStation(&status);
			match_data_.setGetAllianceStationStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
			frc::DriverStation::Alliance color;
			switch (allianceStationID) {
				case HAL_AllianceStationID_kRed1:
				case HAL_AllianceStationID_kRed2:
				case HAL_AllianceStationID_kRed3:
					color = frc::DriverStation::kRed;
					break;
				case HAL_AllianceStationID_kBlue1:
				case HAL_AllianceStationID_kBlue2:
				case HAL_AllianceStationID_kBlue3:
					color = frc::DriverStation::kBlue;
					break;
				default:
					color = frc::DriverStation::kInvalid;
			}
			match_data_.setAllianceColor(color);

			match_data_.setMatchType(static_cast<frc::DriverStation::MatchType>(info.matchType));

			int station_location;
			switch (allianceStationID) {
				case HAL_AllianceStationID_kRed1:
				case HAL_AllianceStationID_kBlue1:
					station_location = 1;
					break;
				case HAL_AllianceStationID_kRed2:
				case HAL_AllianceStationID_kBlue2:
					station_location = 2;
					break;
				case HAL_AllianceStationID_kRed3:
				case HAL_AllianceStationID_kBlue3:
					station_location = 3;
					break;
				default:
					station_location = 0;
			}
			match_data_.setDriverStationLocation(station_location);

			match_data_.setMatchNumber(info.matchNumber);
			match_data_.setReplayNumber(info.replayNumber);
			status = 0;
			match_data_.setBatteryVoltage(HAL_GetVinVoltage(&status));
			match_data_.setGetVinVoltageStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
		}
		//read control word match data at full speed - contains enable info, and reads should be v fast
		HAL_ControlWord controlWord;
		HAL_GetControlWord(&controlWord);
		match_data_.setEnabled(controlWord.enabled && controlWord.dsAttached);
		match_data_.setDisabled(!(controlWord.enabled && controlWord.dsAttached));
		match_data_.setAutonomous(controlWord.autonomous);
		match_data_.setOperatorControl(!(controlWord.autonomous || controlWord.test));
		match_data_.setTest(controlWord.test);
		match_data_.setDSAttached(controlWord.dsAttached);
		match_data_.setFMSAttached(controlWord.fmsAttached);

		read_tracer_.start_unique("robot controller data");
		//check if sufficient time has passed since last read
		if(ros::Time::now().toSec() - t_prev_robot_controller_read_ > (1/robot_controller_read_hz_))
		{
			t_prev_robot_controller_read_ += 1/robot_controller_read_hz_;

			status = 0;
			robot_controller_state_.SetFPGAVersion(HAL_GetFPGAVersion(&status));
			robot_controller_state_.SetFPGAVersionStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFPGARevision(HAL_GetFPGARevision(&status));
			robot_controller_state_.SetFPGARevisionStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFPGATime(HAL_GetFPGATime(&status));
			robot_controller_state_.SetFPGATimeStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetUserButton(HAL_GetFPGAButton(&status));
			robot_controller_state_.SetUserButtonStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetIsSysActive(HAL_GetSystemActive(&status));
			robot_controller_state_.SetIsSysActiveStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetIsBrownedOut(HAL_GetBrownedOut(&status));
			robot_controller_state_.SetIsBrownedOutStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetInputVoltage(HAL_GetVinVoltage(&status));
			robot_controller_state_.SetInputVoltageStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetInputCurrent(HAL_GetVinCurrent(&status));
			robot_controller_state_.SetInputCurrentStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetVoltage3V3(HAL_GetUserVoltage3V3(&status));
			robot_controller_state_.SetVoltage3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetCurrent3V3(HAL_GetUserCurrent3V3(&status));
			robot_controller_state_.SetCurrent3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetEnabled3V3(HAL_GetUserActive3V3(&status));
			robot_controller_state_.SetEnabled3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFaultCount3V3(HAL_GetUserCurrentFaults3V3(&status));
			robot_controller_state_.SetFaultCount3V3Status(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetVoltage5V(HAL_GetUserVoltage5V(&status));
			robot_controller_state_.SetVoltage5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetCurrent5V(HAL_GetUserCurrent5V(&status));
			robot_controller_state_.SetCurrent5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetEnabled5V(HAL_GetUserActive5V(&status));
			robot_controller_state_.SetEnabled5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFaultCount5V(HAL_GetUserCurrentFaults5V(&status));
			robot_controller_state_.SetFaultCount5VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetVoltage6V(HAL_GetUserVoltage6V(&status));
			robot_controller_state_.SetVoltage6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetCurrent6V(HAL_GetUserCurrent6V(&status));
			robot_controller_state_.SetCurrent6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetEnabled6V(HAL_GetUserActive6V(&status));
			robot_controller_state_.SetEnabled6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			status = 0;
			robot_controller_state_.SetFaultCount6V(HAL_GetUserCurrentFaults6V(&status));
			robot_controller_state_.SetFaultCount6VStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));

			float percent_bus_utilization;
			uint32_t bus_off_count;
			uint32_t tx_full_count;
			uint32_t receive_error_count;
			uint32_t transmit_error_count;
			status = 0;
			HAL_CAN_GetCANStatus(&percent_bus_utilization, &bus_off_count,
					&tx_full_count, &receive_error_count,
					&transmit_error_count, &status);

			robot_controller_state_.SetCANPercentBusUtilization(percent_bus_utilization);
			robot_controller_state_.SetCANBusOffCount(bus_off_count);
			robot_controller_state_.SetCANTxFullCount(tx_full_count);
			robot_controller_state_.SetCANReceiveErrorCount(receive_error_count);
			robot_controller_state_.SetCANTransmitErrorCount(transmit_error_count);

			robot_controller_state_.SetCANDataStatus(std::to_string(status) + ": " + HAL_GetErrorMessage(status));
		}
	}

	read_tracer_.start_unique("can talons");
	for (size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		if (can_ctre_mc_local_hardwares_[joint_id])
		{
			std::lock_guard<std::mutex> l(*ctre_mc_read_state_mutexes_[joint_id]);
			auto &ts   = talon_state_[joint_id];
			auto &trts = ctre_mc_read_thread_states_[joint_id];

			// Copy config items from talon state to talon_read_thread_state
			// This makes sure config items set by controllers is
			// eventually reflected in the state unique to the
			// talon_read_thread code
			trts->setTalonMode(ts.getTalonMode());
			trts->setEncoderFeedback(ts.getEncoderFeedback());
			trts->setEncoderTicksPerRotation(ts.getEncoderTicksPerRotation());
			trts->setConversionFactor(ts.getConversionFactor());
			trts->setEnableReadThread(ts.getEnableReadThread());

			// Copy talon state values read in the read thread into the
			// talon state shared globally with the rest of the hardware
			// interface code
			ts.setPosition(trts->getPosition());
			ts.setSpeed(trts->getSpeed());
			ts.setOutputCurrent(trts->getOutputCurrent());
			ts.setBusVoltage(trts->getBusVoltage());
			ts.setMotorOutputPercent(trts->getMotorOutputPercent());
			ts.setOutputVoltage(trts->getOutputVoltage());
			ts.setTemperature(trts->getTemperature());
			ts.setClosedLoopError(trts->getClosedLoopError());
			ts.setIntegralAccumulator(trts->getIntegralAccumulator());
			ts.setErrorDerivative(trts->getErrorDerivative());
			ts.setClosedLoopTarget(trts->getClosedLoopTarget());
			ts.setActiveTrajectoryPosition(trts->getActiveTrajectoryPosition());
			ts.setActiveTrajectoryVelocity(trts->getActiveTrajectoryVelocity());
			ts.setActiveTrajectoryHeading(trts->getActiveTrajectoryHeading());
			ts.setMotionProfileTopLevelBufferCount(trts->getMotionProfileTopLevelBufferCount());
			ts.setMotionProfileStatus(trts->getMotionProfileStatus());
			ts.setFaults(trts->getFaults());
			ts.setForwardLimitSwitch(trts->getForwardLimitSwitch());
			ts.setReverseLimitSwitch(trts->getReverseLimitSwitch());
			ts.setForwardSoftlimitHit(trts->getForwardSoftlimitHit());
			ts.setReverseSoftlimitHit(trts->getReverseSoftlimitHit());
			ts.setStickyFaults(trts->getStickyFaults());
			ts.setFirmwareVersion(trts->getFirmwareVersion());
		}
	}

	read_tracer_.start_unique("canifier");
	for (size_t joint_id = 0; joint_id < num_canifiers_; ++joint_id)
	{
		if (canifier_local_hardwares_[joint_id])
		{
			std::lock_guard<std::mutex> l(*canifier_read_state_mutexes_[joint_id]);
			auto &cs   = canifier_state_[joint_id];
			auto &crts = canifier_read_thread_states_[joint_id];

			// These are used to convert position and velocity units - make sure the
			// read thread's local copy of state is kept up to date
			crts->setEncoderTicksPerRotation(cs.getEncoderTicksPerRotation());
			crts->setConversionFactor(cs.getConversionFactor());

			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
			{
				const auto pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
				cs.setGeneralPinInput(pin, crts->getGeneralPinInput(pin));
			}
			cs.setQuadraturePosition(crts->getQuadraturePosition());
			cs.setQuadratureVelocity(crts->getQuadratureVelocity());
			cs.setBusVoltage(crts->getBusVoltage());

			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
				cs.setPWMInput(pwm_channel, crts->getPWMInput(pwm_channel));
			}
			cs.setFaults(crts->getFaults());
			cs.setStickyFaults(crts->getStickyFaults());
		}
	}

	read_tracer_.start_unique("cancoder");
	for (size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (cancoder_local_hardwares_[joint_id])
		{
			std::lock_guard<std::mutex> l(*cancoder_read_state_mutexes_[joint_id]);
			auto &cs   = cancoder_state_[joint_id];
			auto &crts = cancoder_read_thread_states_[joint_id];

			// These are used to convert position and velocity units - make sure the
			// read thread's local copy of state is kept up to date
			crts->setConversionFactor(cs.getConversionFactor());

			cs.setPosition(crts->getPosition());
			cs.setVelocity(crts->getVelocity());
			cs.setAbsolutePosition(crts->getAbsolutePosition());
			cs.setBusVoltage(crts->getBusVoltage());
			cs.setMagnetFieldStrength(crts->getMagnetFieldStrength());
			cs.setLastTimestamp(crts->getLastTimestamp());
			cs.setFirmwareVersion(crts->getFirmwareVersion());
			cs.setFaults(crts->getFaults());
			cs.setStickyFaults(crts->getStickyFaults());
		}
	}

	read_tracer_.start_unique("nidec");
	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		if (nidec_brushless_local_updates_[i])
			brushless_vel_[i] = nidec_brushlesses_[i]->Get();
	}
	read_tracer_.start_unique("digital in");
	for (size_t i = 0; i < num_digital_inputs_; i++)
	{
		//State should really be a bool - but we're stuck using
		//ROS control code which thinks everything to and from
		//hardware are doubles
		if (digital_input_locals_[i])
			digital_input_state_[i] = (digital_inputs_[i]->Get()^digital_input_inverts_[i]) ? 1 : 0;
	}
#if 0
	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		if (!digital_output_local_updates_[i])
			digital_output_state_[i] = digital_output_command_[i];
	}
	for (size_t i = 0; i < num_pwms_; i++)
	{
		// Just reflect state of output in status
		if (!pwm_local_updates_[i])
			pwm_state_[i] = pwm_command_[i];
	}
	for (size_t i = 0; i < num_solenoids_; i++)
	{
		if (!solenoid_local_updates_[i])
			solenoid_state_[i] = solenoid_command_[i];
	}
	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		if (!double_solenoid_local_updates_[i])
			double_solenoid_state_[i] = double_solenoid_command_[i];
	}
#endif
	read_tracer_.start_unique("analog in");
	for (size_t i = 0; i < num_analog_inputs_; i++)
	{
		if (analog_input_locals_[i])
			analog_input_state_[i] = analog_inputs_[i]->GetValue() *analog_input_a_[i] + analog_input_b_[i];
	}
	read_tracer_.start_unique("navX");
	//navX read here
	for (size_t i = 0; i < num_navX_; i++)
	{
		if (navX_locals_[i])
		{
			// TODO : double check we're reading
			// the correct data

			// navXs_[i]->GetFusedHeading();
			// navXs_[i]->GetPitch();
			// navXs_[i]->GetRoll();

			// TODO : Fill in imu_angular_velocity[i][]

			//navXs_[i]->IsCalibrating();
			//navXs_[i]->IsConnected();
			//navXs_[i]->GetLastSensorTimestamp();
			//
			imu_linear_accelerations_[i][0] = navXs_[i]->GetWorldLinearAccelX();
			imu_linear_accelerations_[i][1] = navXs_[i]->GetWorldLinearAccelY();
			imu_linear_accelerations_[i][2] = navXs_[i]->GetWorldLinearAccelZ();

			//navXs_[i]->IsMoving();
			//navXs_[i]->IsRotating();
			//navXs_[i]->IsMagneticDisturbance();
			//navXs_[i]->IsMagnetometerCalibrated();
			//
			tf2::Quaternion tempQ;
			tempQ.setRPY(navXs_[i]->GetRoll()  / -360. * 2. * M_PI,
						 navXs_[i]->GetPitch() / -360. * 2. * M_PI,
						 navXs_[i]->GetYaw()   /  360. * 2. * M_PI);

			imu_orientations_[i][3] = tempQ.w();
			imu_orientations_[i][0] = tempQ.x();
			imu_orientations_[i][1] = tempQ.y();
			imu_orientations_[i][2] = tempQ.z();

			imu_angular_velocities_[i][0] = navXs_[i]->GetVelocityX();
			imu_angular_velocities_[i][1] = navXs_[i]->GetVelocityY();
			imu_angular_velocities_[i][2] = navXs_[i]->GetVelocityZ();

			//navXs_[i]->GetDisplacementX();
			//navXs_[i]->GetDisplacementY();
			//navXs_[i]->GetDisplacementZ();
			//navXs_[i]->GetAngle(); //continous
			//TODO: add setter functions
		}
	}

	read_tracer_.start_unique("compressors");
	for (size_t i = 0; i < num_compressors_; i++)
	{
		if (compressor_local_updates_[i])
		{
			std::lock_guard<std::mutex> l(*pcm_read_thread_mutexes_[i]);
			pcm_state_[i] = *pcm_read_thread_state_[i];
		}
	}

	read_tracer_.start_unique("pdps");
	for (size_t i = 0; i < num_pdps_; i++)
	{
		if (pdp_locals_[i])
		{
			std::lock_guard<std::mutex> l(*pdp_read_thread_mutexes_[i]);
			pdp_state_[i] = *pdp_read_thread_state_[i];
		}
	}

	read_tracer_.start_unique("as726xs");
	for (size_t i = 0; i < num_as726xs_; i++)
	{
		if (as726x_local_updates_[i])
		{
			std::lock_guard<std::mutex> l(*as726x_read_thread_mutexes_[i]);
			as726x_state_[i].setTemperature(as726x_read_thread_state_[i]->getTemperature());
			as726x_state_[i].setRawChannelData(as726x_read_thread_state_[i]->getRawChannelData());
			as726x_state_[i].setCalibratedChannelData(as726x_read_thread_state_[i]->getCalibratedChannelData());
		}
	}
        
        for(size_t i = 0; i < num_talon_orchestras_; i++)
        {
			if(talon_orchestras_[i]->IsPlaying())
			{
				orchestra_state_[i].setPlaying();
			}
			else
			{
				orchestra_state_[i].setStopped();
			}
        }

	read_tracer_.stop();
	ROS_INFO_STREAM_THROTTLE(60, read_tracer_.report());
}

double FRCRobotHWInterface::getConversionFactor(int encoder_ticks_per_rotation,
		hardware_interface::FeedbackDevice encoder_feedback,
		hardware_interface::TalonMode talon_mode) const
{
	if((talon_mode == hardware_interface::TalonMode_Position) ||
	   (talon_mode == hardware_interface::TalonMode_MotionMagic)) // TODO - maybe motion profile as well?
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2. * M_PI / encoder_ticks_per_rotation;
			case hardware_interface::FeedbackDevice_Analog:
				return 2. * M_PI / 1024.;
			case hardware_interface::FeedbackDevice_IntegratedSensor:
				return 2. * M_PI / 2048.;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
				return 1.;
		}
	}
	else if(talon_mode == hardware_interface::TalonMode_Velocity)
	{
		switch (encoder_feedback)
		{
			case hardware_interface::FeedbackDevice_Uninitialized:
				return 1.;
			case hardware_interface::FeedbackDevice_QuadEncoder:
			case hardware_interface::FeedbackDevice_PulseWidthEncodedPosition:
				return 2. * M_PI / encoder_ticks_per_rotation / .1;
			case hardware_interface::FeedbackDevice_Analog:
				return 2. * M_PI / 1024. / .1;
			case hardware_interface::FeedbackDevice_IntegratedSensor:
				return 2. * M_PI / 2048. / .1;
			case hardware_interface::FeedbackDevice_Tachometer:
			case hardware_interface::FeedbackDevice_SensorSum:
			case hardware_interface::FeedbackDevice_SensorDifference:
			case hardware_interface::FeedbackDevice_RemoteSensor0:
			case hardware_interface::FeedbackDevice_RemoteSensor1:
			case hardware_interface::FeedbackDevice_SoftwareEmulatedSensor:
				//ROS_WARN_STREAM("Unable to convert units.");
				return 1.;
			default:
				ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
				return 1.;
		}
	}
	else
	{
		//ROS_WARN_STREAM("Invalid encoder feedback device (mode = " << talon_mode << " feedback = " << encoder_feedback << ". Unable to convert units.");
		return 1.;
	}
}

bool FRCRobotHWInterface::safeTalonCall(ctre::phoenix::ErrorCode error_code, const std::string &talon_method_name)
{
	//ROS_INFO_STREAM("safeTalonCall(" << talon_method_name << ") = " << error_code);
	std::string error_name;
	static bool error_sent = false;
	switch (error_code)
	{
		case ctre::phoenix::OK :
			can_error_count_ = 0;
			error_sent = false;
			return true; // Yay us!

		case ctre::phoenix::CAN_MSG_STALE :
			error_name = "CAN_MSG_STALE/CAN_TX_FULL/TxFailed";
			break;
		case ctre::phoenix::InvalidParamValue :
			error_name = "InvalidParamValue/CAN_INVALID_PARAM";
			break;

		case ctre::phoenix::RxTimeout :
			error_name = "RxTimeout/CAN_MSG_NOT_FOUND";
			break;
		case ctre::phoenix::TxTimeout :
			error_name = "TxTimeout/CAN_NO_MORE_TX_JOBS";
			break;
		case ctre::phoenix::UnexpectedArbId :
			error_name = "UnexpectedArbId/CAN_NO_SESSIONS_AVAIL";
			break;
		case ctre::phoenix::BufferFull :
			error_name = "BufferFull";
			break;
		case ctre::phoenix::CAN_OVERFLOW:
			error_name = "CAN_OVERFLOW";
			break;
		case ctre::phoenix::SensorNotPresent :
			error_name = "SensorNotPresent";
			break;
		case ctre::phoenix::FirmwareTooOld :
			error_name = "FirmwareTooOld";
			break;
		case ctre::phoenix::CouldNotChangePeriod :
			error_name = "CouldNotChangePeriod";
			break;
		case ctre::phoenix::BufferFailure :
			error_name = "BufferFailure";
			break;

		case ctre::phoenix::GENERAL_ERROR :
			error_name = "GENERAL_ERROR";
			break;

		case ctre::phoenix::SIG_NOT_UPDATED :
			error_name = "SIG_NOT_UPDATED";
			break;
		case ctre::phoenix::NotAllPIDValuesUpdated :
			error_name = "NotAllPIDValuesUpdated";
			break;

		case ctre::phoenix::GEN_PORT_ERROR :
			error_name = "GEN_PORT_ERROR";
			break;
		case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH :
			error_name = "PORT_MODULE_TYPE_MISMATCH";
			break;

		case ctre::phoenix::GEN_MODULE_ERROR :
			error_name = "GEN_MODULE_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR :
			error_name = "MODULE_NOT_INIT_SET_ERROR";
			break;
		case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR :
			error_name = "MODULE_NOT_INIT_GET_ERROR";
			break;

		case ctre::phoenix::WheelRadiusTooSmall :
			error_name = "WheelRadiusTooSmall";
			break;
		case ctre::phoenix::TicksPerRevZero :
			error_name = "TicksPerRevZero";
			break;
		case ctre::phoenix::DistanceBetweenWheelsTooSmall :
			error_name = "DistanceBetweenWheelsTooSmall";
			break;
		case ctre::phoenix::GainsAreNotSet :
			error_name = "GainsAreNotSet";
			break;
		case ctre::phoenix::WrongRemoteLimitSwitchSource :
			error_name = "WrongRemoteLimitSwitchSource";
			break;
		case ctre::phoenix::DoubleVoltageCompensatingWPI :
			error_name = "DoubleVoltageCompensatingWPI";
			break;

		case ctre::phoenix::IncompatibleMode :
			error_name = "IncompatibleMode";
			break;
		case ctre::phoenix::InvalidHandle :
			error_name = "InvalidHandle";
			break;

		case ctre::phoenix::FeatureRequiresHigherFirm:
			error_name = "FeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::TalonFeatureRequiresHigherFirm:
			error_name = "TalonFeatureRequiresHigherFirm";
			break;
		case ctre::phoenix::ConfigFactoryDefaultRequiresHigherFirm:
			error_name = "ConfigFactoryDefaultRequiresHigherFirm";
			break;
		case ctre::phoenix::ConfigMotionSCurveRequiresHigherFirm:
			error_name = "TalonFXFirmwarePreVBatDetect";
			break;
		case ctre::phoenix::TalonFXFirmwarePreVBatDetect:
			error_name = "TalonFXFirmwarePreVBatDetect";
			break;
		case ctre::phoenix::LibraryCouldNotBeLoaded :
			error_name = "LibraryCouldNotBeLoaded";
			break;
		case ctre::phoenix::MissingRoutineInLibrary :
			error_name = "MissingRoutineInLibrary";
			break;
		case ctre::phoenix::ResourceNotAvailable :
			error_name = "ResourceNotAvailable";
			break;

		case ctre::phoenix::PulseWidthSensorNotPresent :
			error_name = "PulseWidthSensorNotPresent";
			break;
		case ctre::phoenix::GeneralWarning :
			error_name = "GeneralWarning";
			break;
		case ctre::phoenix::FeatureNotSupported :
			error_name = "FeatureNotSupported";
			break;
		case ctre::phoenix::NotImplemented :
			error_name = "NotImplemented";
			break;
		case ctre::phoenix::FirmVersionCouldNotBeRetrieved :
			error_name = "FirmVersionCouldNotBeRetrieved";
			break;
		case ctre::phoenix::FeaturesNotAvailableYet :
			error_name = "FeaturesNotAvailableYet";
			break;
		case ctre::phoenix::ControlModeNotValid :
			error_name = "ControlModeNotValid";
			break;

		case ctre::phoenix::ControlModeNotSupportedYet :
			error_name = "ConrolModeNotSupportedYet";
			break;
		case ctre::phoenix::CascadedPIDNotSupporteYet:
			error_name = "CascadedPIDNotSupporteYet/AuxiliaryPIDNotSupportedYet";
			break;
		case ctre::phoenix::RemoteSensorsNotSupportedYet:
			error_name = "RemoteSensorsNotSupportedYet";
			break;
		case ctre::phoenix::MotProfFirmThreshold:
			error_name = "MotProfFirmThreshold";
			break;
		case ctre::phoenix::MotProfFirmThreshold2:
			error_name = "MotProfFirmThreshold2";
                        break;

                case ctre::phoenix::MusicFileNotFound:
                        error_name = "MusicFileNotFound";
                        break;
                case ctre::phoenix::MusicFileWrongSize:
                        error_name = "MusicFileWrongSize";
                        break;
                case ctre::phoenix::MusicFileTooNew:
                        error_name = "MusicFileTooNew";
                        break;
                case ctre::phoenix::MusicFileInvalid:
                        error_name = "MusicFileInvalid";
                        break;
                case ctre::phoenix::InvalidOrchestraAction:
                        error_name = "InvalidOrchestraAction";
                        break;
                case ctre::phoenix::MusicFileTooOld:
                        error_name = "MusicFileTooOld";
                        break;
                case ctre::phoenix::MusicInterrupted:
                        error_name = "MusicInterrupted";
                        break;
                case ctre::phoenix::MusicNotSupported:
                        error_name = "MusicNotSupported";
                        break;

		default:
			{
				std::stringstream s;
				s << "Unknown Talon error " << error_code;
				error_name = s.str();
				break;
			}

	}
	ROS_ERROR_STREAM("Error calling " << talon_method_name << " : " << error_name);
	can_error_count_++;
	if ((can_error_count_> 1000) && !error_sent)
	{
		HAL_SendError(true, -1, false, "safeTalonCall - too many CAN bus errors!", "", "", true);
		error_sent = true;
	}
	return false;
}

//#define DEBUG_WRITE
void FRCRobotHWInterface::write(const ros::Time& /*time*/, const ros::Duration& period)
{
	// Was the robot enabled last time write was run?
	static bool last_robot_enabled = false;

#if 0
	if (!run_hal_robot_ && num_can_ctre_mcs_)
	{
		c_FeedEnable(100);
	}
#endif

	for (size_t joint_id = 0; joint_id < num_can_ctre_mcs_; ++joint_id)
	{
		if (!can_ctre_mc_local_hardwares_[joint_id])
			continue;
		if (!talon_command_[joint_id].try_lock())
			continue;

		custom_profile_write(joint_id);

		//TODO : skip over most or all of this if the talon is in follower mode
		//       Only do the Set() call and then never do anything else?

		// If the original object was a talon, both talon and victor will be valid
		// The original object was a victor, talon will be nullptr
		auto mc_enhanced = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorControllerEnhanced>(ctre_mcs_[joint_id]);
		auto falcon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonFX>(ctre_mcs_[joint_id]);
		auto talon = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonSRX>(ctre_mcs_[joint_id]);
		auto victor = std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::IMotorController>(ctre_mcs_[joint_id]);
#if 0
		if (mc_enhanced)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "mc_enhanced OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "mc_enhanced NOT OK for id " << joint_id);
		}
		if (falcon)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "falcon OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "falcon NOT OK for id " << joint_id);
		}
		if (talon)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "talon OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "talon NOT OK for id " << joint_id);
		}
		if (victor)
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "victor OK for id " << joint_id);
		}
		else
		{
			ROS_ERROR_STREAM_THROTTLE(5.0, "victor NOT OK for id " << joint_id);
		}
#endif

		if (!victor && !talon && !mc_enhanced && !falcon) // skip unintialized Talons
		{
			talon_command_[joint_id].unlock();
			continue;
		}

		// Save some typing by making references to commonly
		// used variables
		auto &ts = talon_state_[joint_id];
		auto &tc = talon_command_[joint_id];

		// If the motor controller has been reset since the last write()
		// call, reset all of the flags indicating that commands have been
		// written to the controllers. This will force the config data
		// to be re-written by the rest of the write() function
		if (victor->HasResetOccurred())
		{
			for (size_t i = 0; i < hardware_interface::TALON_PIDF_SLOTS; i++)
				tc.resetPIDF(i);
			tc.resetAuxPidPolarity();
			tc.resetIntegralAccumulator();
			tc.resetMode();
			tc.resetDemand1();
			tc.resetPidfSlot();
			tc.resetEncoderFeedback();
			tc.resetRemoteEncoderFeedback();
			tc.resetRemoteFeedbackFilters();
			tc.resetSensorTerms();
			tc.resetOutputShaping();
			tc.resetVoltageCompensation();
			tc.resetVelocityMeasurement();
			tc.resetSensorPosition();
			tc.resetLimitSwitchesSource();
			tc.resetRemoteLimitSwitchesSource();
			tc.resetSoftLimit();
			tc.resetCurrentLimit();
			tc.resetMotionCruise();
			for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
				tc.resetStatusFramePeriod(static_cast<hardware_interface::StatusFrame>(i));
			for (int i = hardware_interface::Control_3_General; i < hardware_interface::Control_Last; i++)
				tc.resetControlFramePeriod(static_cast<hardware_interface::ControlFrame>(i));
			tc.resetMotionProfileTrajectoryPeriod();
			tc.resetSupplyCurrentLimit();
			tc.resetStatorCurrentLimit();
			tc.resetMotorCommutation();
			tc.resetAbsoluteSensorRange();
			tc.resetSensorInitializationStrategy();
		}

		bool enable_read_thread;
		if (tc.enableReadThreadChanged(enable_read_thread))
			ts.setEnableReadThread(enable_read_thread);

		hardware_interface::FeedbackDevice internal_feedback_device = hardware_interface::FeedbackDevice_Uninitialized;
		double feedback_coefficient;

		ctre::phoenix::motorcontrol::FeedbackDevice talon_feedback_device;
		if (tc.encoderFeedbackChanged(internal_feedback_device, feedback_coefficient) &&
			talon_convert_.feedbackDevice(internal_feedback_device, talon_feedback_device))
		{
			// Check for errors on Talon writes. If it fails, used the reset() call to
			// set the changed var for the config items to true. This will trigger a re-try
			// the next time through the loop.
			bool rc = true;
			// Only actually set this on the hardware for Talon devices. But set it in
			// talon_states for both types of motor controllers. This allows the conversion
			// functions to work properly?
			if (mc_enhanced)
			{
				rc &= safeTalonCall(mc_enhanced->ConfigSelectedFeedbackSensor(talon_feedback_device, pidIdx, timeoutMs),"ConfigSelectedFeedbackSensor");
				rc &= safeTalonCall(mc_enhanced->ConfigSelectedFeedbackCoefficient(feedback_coefficient, pidIdx, timeoutMs),"ConfigSelectedFeedbackCoefficient");
			}
			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " feedback");
				ts.setEncoderFeedback(internal_feedback_device);
				ts.setFeedbackCoefficient(feedback_coefficient);
			}
			else
			{
				tc.resetEncoderFeedback();
			}
		}

		ctre::phoenix::motorcontrol::RemoteFeedbackDevice talon_remote_feedback_device;
		hardware_interface::RemoteFeedbackDevice internal_remote_feedback_device;
		if (tc.remoteEncoderFeedbackChanged(internal_remote_feedback_device) &&
				talon_convert_.remoteFeedbackDevice(internal_remote_feedback_device, talon_remote_feedback_device))
		{
			// Check for errors on Talon writes. If it fails, used the reset() call to
			// set the changed var for the config items to true. This will trigger a re-try
			// the next time through the loop.
			if (safeTalonCall(victor->ConfigSelectedFeedbackSensor(talon_remote_feedback_device, pidIdx, timeoutMs), "ConfigSelectedFeedbackSensor (Remote)"))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote feedback sensor");
				ts.setRemoteEncoderFeedback(internal_remote_feedback_device);
			}
			else
			{
				tc.resetRemoteEncoderFeedback();
			}
		}

		std::array<int, 2>                                             remote_feedback_device_ids;
		std::array<hardware_interface::RemoteSensorSource, 2>          internal_remote_feedback_filters;
		std::array<ctre::phoenix::motorcontrol::RemoteSensorSource, 2> victor_remote_feedback_filters;
		if (tc.remoteFeedbackFiltersChanged(remote_feedback_device_ids, internal_remote_feedback_filters) &&
			talon_convert_.remoteSensorSource(internal_remote_feedback_filters[0], victor_remote_feedback_filters[0]) &&
			talon_convert_.remoteSensorSource(internal_remote_feedback_filters[1], victor_remote_feedback_filters[1]))
		{
			if (safeTalonCall(victor->ConfigRemoteFeedbackFilter(remote_feedback_device_ids[0], victor_remote_feedback_filters[0], 0, timeoutMs), "ConfigRemoteFeedbackFilter (0)") &&
				safeTalonCall(victor->ConfigRemoteFeedbackFilter(remote_feedback_device_ids[1], victor_remote_feedback_filters[1], 1, timeoutMs), "ConfigRemoteFeedbackFilter (1)"))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " remote feedback filters");
				ts.setRemoteFeedbackDeviceIds(remote_feedback_device_ids);
				ts.setRemoteFeedbackFilters(internal_remote_feedback_filters);
			}
			else
			{
				tc.resetRemoteFeedbackFilters();
			}
		}

		std::array<hardware_interface::FeedbackDevice, hardware_interface::SensorTerm_Last> internal_sensor_terms;
		std::array<ctre::phoenix::motorcontrol::FeedbackDevice, hardware_interface::SensorTerm_Last> victor_sensor_terms;
		if (tc.sensorTermsChanged(internal_sensor_terms) &&
				talon_convert_.feedbackDevice(internal_sensor_terms[0], victor_sensor_terms[0]) &&
				talon_convert_.feedbackDevice(internal_sensor_terms[1], victor_sensor_terms[1]) &&
				talon_convert_.feedbackDevice(internal_sensor_terms[2], victor_sensor_terms[2]) &&
				talon_convert_.feedbackDevice(internal_sensor_terms[3], victor_sensor_terms[3]))
		{
			// Check for errors on Talon writes. If it fails, used the reset() call to
			// set the changed var for the config items to true. This will trigger a re-try
			// the next time through the loop.
			if (safeTalonCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Sum0, victor_sensor_terms[0], timeoutMs),"ConfigSensorTerm Sum0") &&
				safeTalonCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Sum1, victor_sensor_terms[1], timeoutMs),"ConfigSensorTerm Sum1") &&
				safeTalonCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Diff0, victor_sensor_terms[2], timeoutMs),"ConfigSensorTerm Diff0") &&
				safeTalonCall(victor->ConfigSensorTerm(ctre::phoenix::motorcontrol::SensorTerm::SensorTerm_Diff1, victor_sensor_terms[3], timeoutMs),"ConfigSensorTerm Diff1"))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " sensor terms");
				ts.setSensorTerms(internal_sensor_terms);
			}
			else
			{
				tc.resetSensorTerms();
			}
		}

		// Get mode that is about to be commanded
		const hardware_interface::TalonMode talon_mode = tc.getMode();
		const int encoder_ticks_per_rotation = tc.getEncoderTicksPerRotation();
		ts.setEncoderTicksPerRotation(encoder_ticks_per_rotation);

		const double conversion_factor = tc.getConversionFactor();
		// No point doing changed() since it's quicker just to just copy a double
		// rather than query a bool and do it conditionally
		ts.setConversionFactor(conversion_factor);

		const double radians_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Position) * conversion_factor;
		const double radians_per_second_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, hardware_interface::TalonMode_Velocity) * conversion_factor;
		const double closed_loop_scale = getConversionFactor(encoder_ticks_per_rotation, internal_feedback_device, talon_mode) * conversion_factor;

		bool close_loop_mode = false;
		bool motion_profile_mode = false;

		if ((talon_mode == hardware_interface::TalonMode_Position) ||
			(talon_mode == hardware_interface::TalonMode_Velocity) ||
			(talon_mode == hardware_interface::TalonMode_Current ))
		{
			close_loop_mode = true;
		}
		else if ((talon_mode == hardware_interface::TalonMode_MotionProfile) ||
				(talon_mode == hardware_interface::TalonMode_MotionMagic)   ||
				(talon_mode == hardware_interface::TalonMode_MotionProfileArc))
		{
			close_loop_mode = true;
			motion_profile_mode = true;
		}

		if (close_loop_mode)
		{
			int slot;
			const bool slot_changed = tc.slotChanged(slot);

			double p;
			double i;
			double d;
			double f;
			int    iz;
			int    allowable_closed_loop_error;
			double max_integral_accumulator;
			double closed_loop_peak_output;
			int    closed_loop_period;

			if (tc.pidfChanged(p, i, d, f, iz, allowable_closed_loop_error, max_integral_accumulator, closed_loop_peak_output, closed_loop_period, slot))
			{
				bool rc = true;
				rc &= safeTalonCall(victor->Config_kP(slot, p, timeoutMs), "Config_kP");
				rc &= safeTalonCall(victor->Config_kI(slot, i, timeoutMs), "Config_kI");
				rc &= safeTalonCall(victor->Config_kD(slot, d, timeoutMs), "Config_kD");
				rc &= safeTalonCall(victor->Config_kF(slot, f, timeoutMs), "Config_kF");
				rc &= safeTalonCall(victor->Config_IntegralZone(slot, iz, timeoutMs), "Config_IntegralZone");
				// TODO : Scale these two?
				rc &= safeTalonCall(victor->ConfigAllowableClosedloopError(slot, allowable_closed_loop_error, timeoutMs), "ConfigAllowableClosedloopError");
				rc &= safeTalonCall(victor->ConfigMaxIntegralAccumulator(slot, max_integral_accumulator, timeoutMs), "ConfigMaxIntegralAccumulator");
				rc &= safeTalonCall(victor->ConfigClosedLoopPeakOutput(slot, closed_loop_peak_output, timeoutMs), "ConfigClosedLoopPeakOutput");
				rc &= safeTalonCall(victor->ConfigClosedLoopPeriod(slot, closed_loop_period, timeoutMs), "ConfigClosedLoopPeriod");

				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " PIDF slot " << slot << " config values");
					ts.setPidfP(p, slot);
					ts.setPidfI(i, slot);
					ts.setPidfD(d, slot);
					ts.setPidfF(f, slot);
					ts.setPidfIzone(iz, slot);
					ts.setAllowableClosedLoopError(allowable_closed_loop_error, slot);
					ts.setMaxIntegralAccumulator(max_integral_accumulator, slot);
					ts.setClosedLoopPeakOutput(closed_loop_peak_output, slot);
					ts.setClosedLoopPeriod(closed_loop_period, slot);
				}
				else
				{
					tc.resetPIDF(slot);
				}
			}

			bool aux_pid_polarity;
			if (tc.auxPidPolarityChanged(aux_pid_polarity))
			{
				if (safeTalonCall(victor->ConfigAuxPIDPolarity(aux_pid_polarity, timeoutMs), "ConfigAuxPIDPolarity"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF polarity to " << aux_pid_polarity << std::endl);
					ts.setAuxPidPolarity(aux_pid_polarity);
				}
				else
				{
					tc.resetAuxPidPolarity();
				}
			}

			if (slot_changed)
			{
				if (safeTalonCall(victor->SelectProfileSlot(slot, pidIdx), "SelectProfileSlot"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << " PIDF slot to " << slot << std::endl);
					ts.setSlot(slot);
				}
				else
				{
					tc.resetPidfSlot();
				}
			}
		}

		bool invert;
		bool sensor_phase;
		if (tc.invertChanged(invert, sensor_phase))
		{
			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<
					" invert = " << invert << " phase = " << sensor_phase);
			// TODO : can these calls fail. If so, what to do if they do?
			victor->SetInverted(invert);
			safeTalonCall(victor->GetLastError(), "SetInverted");
			victor->SetSensorPhase(sensor_phase);
			safeTalonCall(victor->GetLastError(), "SetSensorPhase");
			ts.setInvert(invert);
			ts.setSensorPhase(sensor_phase);
		}

		hardware_interface::NeutralMode neutral_mode;
		ctre::phoenix::motorcontrol::NeutralMode ctre_neutral_mode;
		if (tc.neutralModeChanged(neutral_mode) &&
				talon_convert_.neutralMode(neutral_mode, ctre_neutral_mode))
		{

			ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " neutral mode");
			victor->SetNeutralMode(ctre_neutral_mode);
			safeTalonCall(victor->GetLastError(), "SetNeutralMode");
			ts.setNeutralMode(neutral_mode);
		}

		if (tc.neutralOutputChanged())
		{
			ROS_INFO_STREAM("Set joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " neutral output");
			victor->NeutralOutput();
			safeTalonCall(victor->GetLastError(), "NeutralOutput");
			ts.setNeutralOutput(true);
		}

		double iaccum;
		if (close_loop_mode && tc.integralAccumulatorChanged(iaccum))
		{
			//The units on this aren't really right?
			if (safeTalonCall(victor->SetIntegralAccumulator(iaccum / closed_loop_scale, pidIdx, timeoutMs), "SetIntegralAccumulator"))
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " integral accumulator");
				// Do not set talon state - this changes
				// dynamically so read it in read() above instead
			}
			else
				tc.resetIntegralAccumulator();

		}

		double closed_loop_ramp;
		double open_loop_ramp;
		double peak_output_forward;
		double peak_output_reverse;
		double nominal_output_forward;
		double nominal_output_reverse;
		double neutral_deadband;
		if (tc.outputShapingChanged(closed_loop_ramp,
					open_loop_ramp,
					peak_output_forward,
					peak_output_reverse,
					nominal_output_forward,
					nominal_output_reverse,
					neutral_deadband))
		{
			bool rc = true;
			rc &= safeTalonCall(victor->ConfigOpenloopRamp(open_loop_ramp, timeoutMs),"ConfigOpenloopRamp");
			rc &= safeTalonCall(victor->ConfigClosedloopRamp(closed_loop_ramp, timeoutMs),"ConfigClosedloopRamp");
			rc &= safeTalonCall(victor->ConfigPeakOutputForward(peak_output_forward, timeoutMs),"ConfigPeakOutputForward");          // 100
			rc &= safeTalonCall(victor->ConfigPeakOutputReverse(peak_output_reverse, timeoutMs),"ConfigPeakOutputReverse");          // -100
			rc &= safeTalonCall(victor->ConfigNominalOutputForward(nominal_output_forward, timeoutMs),"ConfigNominalOutputForward"); // 0
			rc &= safeTalonCall(victor->ConfigNominalOutputReverse(nominal_output_reverse, timeoutMs),"ConfigNominalOutputReverse"); // 0
			rc &= safeTalonCall(victor->ConfigNeutralDeadband(neutral_deadband, timeoutMs),"ConfigNeutralDeadband");                 // 0

			if (rc)
			{
				ts.setOpenloopRamp(open_loop_ramp);
				ts.setClosedloopRamp(closed_loop_ramp);
				ts.setPeakOutputForward(peak_output_forward);
				ts.setPeakOutputReverse(peak_output_reverse);
				ts.setNominalOutputForward(nominal_output_forward);
				ts.setNominalOutputReverse(nominal_output_reverse);
				ts.setNeutralDeadband(neutral_deadband);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " output shaping");
			}
			else
			{
				tc.resetOutputShaping();
			}
		}

		double v_c_saturation;
		int v_measurement_filter;
		bool v_c_enable;
		if (tc.voltageCompensationChanged(v_c_saturation,
					v_measurement_filter,
					v_c_enable))
		{
			bool rc = true;
			rc &= safeTalonCall(victor->ConfigVoltageCompSaturation(v_c_saturation, timeoutMs),"ConfigVoltageCompSaturation");
			rc &= safeTalonCall(victor->ConfigVoltageMeasurementFilter(v_measurement_filter, timeoutMs),"ConfigVoltageMeasurementFilter");

			if (rc)
			{
				// Only enable once settings are correctly written to the Talon
				victor->EnableVoltageCompensation(v_c_enable);
				rc &= safeTalonCall(victor->GetLastError(), "EnableVoltageCompensation");
			}
			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " voltage compensation");

				ts.setVoltageCompensationSaturation(v_c_saturation);
				ts.setVoltageMeasurementFilter(v_measurement_filter);
				ts.setVoltageCompensationEnable(v_c_enable);
			}
			else
			{
				tc.resetVoltageCompensation();
			}
		}

		if (mc_enhanced)
		{
			hardware_interface::VelocityMeasurementPeriod internal_v_m_period;
			ctre::phoenix::motorcontrol::VelocityMeasPeriod phoenix_v_m_period;
			int v_m_window;

			if (tc.velocityMeasurementChanged(internal_v_m_period, v_m_window) &&
				talon_convert_.velocityMeasurementPeriod(internal_v_m_period, phoenix_v_m_period))
			{
				bool rc = true;
				rc &= safeTalonCall(mc_enhanced->ConfigVelocityMeasurementPeriod(phoenix_v_m_period, timeoutMs),"ConfigVelocityMeasurementPeriod");
				rc &= safeTalonCall(mc_enhanced->ConfigVelocityMeasurementWindow(v_m_window, timeoutMs),"ConfigVelocityMeasurementWindow");

				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " velocity measurement period / window");
					ts.setVelocityMeasurementPeriod(internal_v_m_period);
					ts.setVelocityMeasurementWindow(v_m_window);
				}
				else
				{
					tc.resetVelocityMeasurement();
				}
			}
		}

		double sensor_position;
		if (tc.sensorPositionChanged(sensor_position))
		{
			if (safeTalonCall(victor->SetSelectedSensorPosition(sensor_position / radians_scale, pidIdx, timeoutMs),
						"SetSelectedSensorPosition"))
			{
				ROS_INFO_STREAM_THROTTLE(2, "Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " selected sensor position");
			}
			else
			{
				tc.resetSensorPosition();
			}
		}

		if (mc_enhanced)
		{
			hardware_interface::LimitSwitchSource internal_local_forward_source;
			hardware_interface::LimitSwitchNormal internal_local_forward_normal;
			hardware_interface::LimitSwitchSource internal_local_reverse_source;
			hardware_interface::LimitSwitchNormal internal_local_reverse_normal;
			ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_forward_source;
			ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_forward_normal;
			ctre::phoenix::motorcontrol::LimitSwitchSource talon_local_reverse_source;
			ctre::phoenix::motorcontrol::LimitSwitchNormal talon_local_reverse_normal;
			if (tc.limitSwitchesSourceChanged(internal_local_forward_source, internal_local_forward_normal,
											  internal_local_reverse_source, internal_local_reverse_normal) &&
				talon_convert_.limitSwitchSource(internal_local_forward_source, talon_local_forward_source) &&
				talon_convert_.limitSwitchNormal(internal_local_forward_normal, talon_local_forward_normal) &&
				talon_convert_.limitSwitchSource(internal_local_reverse_source, talon_local_reverse_source) &&
				talon_convert_.limitSwitchNormal(internal_local_reverse_normal, talon_local_reverse_normal) )
			{
				bool rc = safeTalonCall(mc_enhanced->ConfigForwardLimitSwitchSource(talon_local_forward_source, talon_local_forward_normal, timeoutMs),"ConfigForwardLimitSwitchSource");
				rc &= safeTalonCall(mc_enhanced->ConfigReverseLimitSwitchSource(talon_local_reverse_source, talon_local_reverse_normal, timeoutMs),"ConfigReverseLimitSwitchSource");

				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
							<< " limit switches "
							<< talon_local_forward_source << " " << talon_local_forward_normal << " "
							<< talon_local_reverse_source << " " << talon_local_reverse_normal);
					ts.setForwardLimitSwitchSource(internal_local_forward_source, internal_local_forward_normal);
					ts.setReverseLimitSwitchSource(internal_local_reverse_source, internal_local_reverse_normal);
				}
				else
				{
					tc.resetLimitSwitchesSource();
				}
			}
		}

		hardware_interface::RemoteLimitSwitchSource internal_remote_forward_source;
		hardware_interface::LimitSwitchNormal internal_remote_forward_normal;
		unsigned int remote_forward_id;
		hardware_interface::RemoteLimitSwitchSource internal_remote_reverse_source;
		hardware_interface::LimitSwitchNormal internal_remote_reverse_normal;
		unsigned int remote_reverse_id;
		ctre::phoenix::motorcontrol::RemoteLimitSwitchSource talon_remote_forward_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_remote_forward_normal;
		ctre::phoenix::motorcontrol::RemoteLimitSwitchSource talon_remote_reverse_source;
		ctre::phoenix::motorcontrol::LimitSwitchNormal talon_remote_reverse_normal;
		if (tc.remoteLimitSwitchesSourceChanged(internal_remote_forward_source, internal_remote_forward_normal, remote_forward_id,
											    internal_remote_reverse_source, internal_remote_reverse_normal, remote_reverse_id) &&
			talon_convert_.remoteLimitSwitchSource(internal_remote_forward_source, talon_remote_forward_source) &&
			talon_convert_.limitSwitchNormal(internal_remote_forward_normal, talon_remote_forward_normal) &&
			talon_convert_.remoteLimitSwitchSource(internal_remote_reverse_source, talon_remote_reverse_source) &&
			talon_convert_.limitSwitchNormal(internal_remote_reverse_normal, talon_remote_reverse_normal) )
		{
			bool rc = true;
			rc &= safeTalonCall(victor->ConfigForwardLimitSwitchSource(talon_remote_forward_source, talon_remote_forward_normal, remote_forward_id, timeoutMs),"ConfigForwardLimitSwitchSource(Remote)");
			rc &= safeTalonCall(victor->ConfigReverseLimitSwitchSource(talon_remote_reverse_source, talon_remote_reverse_normal, remote_reverse_id, timeoutMs),"ConfigReverseLimitSwitchSource(Remote)");

			if (rc)
			{
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id]
						<< " remote limit switches "
						<< talon_remote_forward_source << " " << talon_remote_forward_normal << " " << remote_forward_id << " "
						<< talon_remote_reverse_source << " " << talon_remote_reverse_normal << " " << remote_reverse_id);
				ts.setRemoteForwardLimitSwitchSource(internal_remote_forward_source, internal_remote_forward_normal, remote_forward_id);
				ts.setRemoteReverseLimitSwitchSource(internal_remote_reverse_source, internal_remote_reverse_normal, remote_reverse_id);
			}
			else
			{
				tc.resetRemoteLimitSwitchesSource();
			}
		}

		double softlimit_forward_threshold;
		bool softlimit_forward_enable;
		double softlimit_reverse_threshold;
		bool softlimit_reverse_enable;
		bool softlimit_override_enable;
		if (tc.softLimitChanged(softlimit_forward_threshold,
								softlimit_forward_enable,
								softlimit_reverse_threshold,
								softlimit_reverse_enable,
								softlimit_override_enable))
		{
			const double softlimit_forward_threshold_NU = softlimit_forward_threshold / radians_scale; //native units
			const double softlimit_reverse_threshold_NU = softlimit_reverse_threshold / radians_scale;
			victor->OverrideSoftLimitsEnable(softlimit_override_enable);
			bool rc = true;
			rc &= safeTalonCall(victor->GetLastError(), "OverrideSoftLimitsEnable");
			rc &= safeTalonCall(victor->ConfigForwardSoftLimitThreshold(softlimit_forward_threshold_NU, timeoutMs),"ConfigForwardSoftLimitThreshold");
			rc &= safeTalonCall(victor->ConfigForwardSoftLimitEnable(softlimit_forward_enable, timeoutMs),"ConfigForwardSoftLimitEnable");
			rc &= safeTalonCall(victor->ConfigReverseSoftLimitThreshold(softlimit_reverse_threshold_NU, timeoutMs),"ConfigReverseSoftLimitThreshold");
			rc &= safeTalonCall(victor->ConfigReverseSoftLimitEnable(softlimit_reverse_enable, timeoutMs),"ConfigReverseSoftLimitEnable");

			if (rc)
			{
				ts.setOverrideSoftLimitsEnable(softlimit_override_enable);
				ts.setForwardSoftLimitThreshold(softlimit_forward_threshold);
				ts.setForwardSoftLimitEnable(softlimit_forward_enable);
				ts.setReverseSoftLimitThreshold(softlimit_reverse_threshold);
				ts.setReverseSoftLimitEnable(softlimit_reverse_enable);
				ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " soft limits " <<
						std::endl << "\tforward enable=" << softlimit_forward_enable << " forward threshold=" << softlimit_forward_threshold <<
						std::endl << "\treverse enable=" << softlimit_reverse_enable << " reverse threshold=" << softlimit_reverse_threshold <<
						std::endl << "\toverride_enable=" << softlimit_override_enable);
			}
			else
			{
				tc.resetSoftLimit();
			}
		}

		if (talon)
		{
			int peak_amps;
			int peak_msec;
			int continuous_amps;
			bool enable;
			if (tc.currentLimitChanged(peak_amps, peak_msec, continuous_amps, enable))
			{
				bool rc = true;
				rc &= safeTalonCall(talon->ConfigPeakCurrentLimit(peak_amps, timeoutMs),"ConfigPeakCurrentLimit");
				rc &= safeTalonCall(talon->ConfigPeakCurrentDuration(peak_msec, timeoutMs),"ConfigPeakCurrentDuration");
				rc &= safeTalonCall(talon->ConfigContinuousCurrentLimit(continuous_amps, timeoutMs),"ConfigContinuousCurrentLimit");
				if (rc)
				{
					talon->EnableCurrentLimit(enable);
					rc &= safeTalonCall(talon->GetLastError(), "EnableCurrentLimit");
				}
				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " peak current");
					ts.setPeakCurrentLimit(peak_amps);
					ts.setPeakCurrentDuration(peak_msec);
					ts.setContinuousCurrentLimit(continuous_amps);
					ts.setCurrentLimitEnable(enable);
				}
				else
				{
					tc.resetCurrentLimit();
				}
			}
		}

		if (falcon)
		{
			double limit;
			double trigger_threshold_current;
			double trigger_threshold_time;
			double limit_enable;
			if (tc.supplyCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
			{
				if (safeTalonCall(falcon->ConfigSupplyCurrentLimit(ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration(limit_enable, limit, trigger_threshold_current, trigger_threshold_time), timeoutMs), "ConfigSupplyCurrentLimit"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " supply current limit");
					ts.setSupplyCurrentLimit(limit);
					ts.setSupplyCurrentLimitEnable(limit_enable);
					ts.setSupplyCurrentTriggerThresholdCurrent(trigger_threshold_current);
					ts.setSupplyCurrentTriggerThresholdTime(trigger_threshold_time);
				}
				else
				{
					tc.resetSupplyCurrentLimit();
				}
			}
			if (tc.supplyCurrentLimitChanged(limit, trigger_threshold_current, trigger_threshold_time, limit_enable))
			{
				if (safeTalonCall(falcon->ConfigStatorCurrentLimit(ctre::phoenix::motorcontrol::StatorCurrentLimitConfiguration(limit_enable, limit, trigger_threshold_current, trigger_threshold_time), timeoutMs), "ConfigStatorCurrentLimit"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " stator current limit");
					ts.setStatorCurrentLimit(limit);
					ts.setStatorCurrentLimitEnable(limit_enable);
					ts.setStatorCurrentTriggerThresholdCurrent(trigger_threshold_current);
					ts.setStatorCurrentTriggerThresholdTime(trigger_threshold_time);
				}
				else
				{
					tc.resetStatorCurrentLimit();
				}
			}

			hardware_interface::MotorCommutation motor_commutation;
			ctre::phoenix::motorcontrol::MotorCommutation motor_commutation_ctre;
			if (tc.motorCommutationChanged(motor_commutation) &&
				talon_convert_.motorCommutation(motor_commutation, motor_commutation_ctre))
			{
				if (safeTalonCall(falcon->ConfigMotorCommutation(motor_commutation_ctre, timeoutMs), "ConfigMotorCommutation"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motor commutation");
					ts.setMotorCommutation(motor_commutation);
				}
				else
				{
					tc.resetMotorCommutation();
				}
			}

			hardware_interface::AbsoluteSensorRange absolute_sensor_range;
			ctre::phoenix::sensors::AbsoluteSensorRange absolute_sensor_range_ctre;
			if (tc.absoluteSensorRangeChanged(absolute_sensor_range) &&
				talon_convert_.absoluteSensorRange(absolute_sensor_range, absolute_sensor_range_ctre))
			{
				if (safeTalonCall(falcon->ConfigIntegratedSensorAbsoluteRange(absolute_sensor_range_ctre, timeoutMs), "ConfigIntegratedSensorAbsoluteRange"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
					ts.setAbsoluteSensorRange(absolute_sensor_range);
				}
				else
				{
					tc.resetAbsoluteSensorRange();
				}
			}

			hardware_interface::SensorInitializationStrategy sensor_initialization_strategy;
			ctre::phoenix::sensors::SensorInitializationStrategy sensor_initialization_strategy_ctre;
			if (tc.sensorInitializationStrategyChanged(sensor_initialization_strategy) &&
				talon_convert_.sensorInitializationStrategy(sensor_initialization_strategy, sensor_initialization_strategy_ctre))
			{
				if (safeTalonCall(falcon->ConfigIntegratedSensorInitializationStrategy(sensor_initialization_strategy_ctre, timeoutMs), "ConfigIntegratedSensorInitializationStrategy"))
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " absolute sensor range");
					ts.setSensorInitializationStrategy(sensor_initialization_strategy);
				}
				else
				{
					tc.resetSensorInitializationStrategy();
				}
			}
		}

		if (mc_enhanced)
		{
			// TODO : fix for Victor non-enhanced status frames
			for (int i = hardware_interface::Status_1_General; i < hardware_interface::Status_Last; i++)
			{
				uint8_t period;
				const hardware_interface::StatusFrame status_frame = static_cast<hardware_interface::StatusFrame>(i);
				if (tc.statusFramePeriodChanged(status_frame, period) && (period != 0))
				{
					ctre::phoenix::motorcontrol::StatusFrameEnhanced status_frame_enhanced;
					if (talon_convert_.statusFrame(status_frame, status_frame_enhanced))
					{
						if (safeTalonCall(mc_enhanced->SetStatusFramePeriod(status_frame_enhanced, period), "SetStatusFramePeriod"))
						{
							ts.setStatusFramePeriod(status_frame, period);
							ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " status_frame " << i << "=" << static_cast<int>(period) << "mSec");
						}
						else
							tc.resetStatusFramePeriod(status_frame);
					}
				}
			}
		}

		for (int i = hardware_interface::Control_3_General; i < hardware_interface::Control_Last; i++)
		{
			uint8_t period;
			const hardware_interface::ControlFrame control_frame = static_cast<hardware_interface::ControlFrame>(i);
			if (tc.controlFramePeriodChanged(control_frame, period) && (period != 0))
			{
				ctre::phoenix::motorcontrol::ControlFrame control_frame_phoenix;
				if (talon_convert_.controlFrame(control_frame, control_frame_phoenix))
				{
					if (safeTalonCall(victor->SetControlFramePeriod(control_frame_phoenix, period), "SetControlFramePeriod"))
					{
						ts.setControlFramePeriod(control_frame, period);
						ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " control_frame " << i << "=" << static_cast<int>(period) << "mSec");
					}
					else
						tc.setControlFramePeriod(control_frame, period);
				}

			}
		}

		if (motion_profile_mode)
		{
			double motion_cruise_velocity;
			double motion_acceleration;
			unsigned int motion_s_curve_strength;
			if (tc.motionCruiseChanged(motion_cruise_velocity, motion_acceleration, motion_s_curve_strength))
			{
				//converted from rad/sec to native units
				bool rc = safeTalonCall(victor->ConfigMotionCruiseVelocity(motion_cruise_velocity / radians_per_second_scale, timeoutMs),"ConfigMotionCruiseVelocity(");
				rc &= safeTalonCall(victor->ConfigMotionAcceleration(motion_acceleration / radians_per_second_scale, timeoutMs),"ConfigMotionAcceleration(");
				rc &= safeTalonCall(victor->ConfigMotionSCurveStrength(motion_s_curve_strength, timeoutMs), "ConfigMotionSCurveStrength");

				if (rc)
				{
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " cruise velocity / acceleration");
					ts.setMotionCruiseVelocity(motion_cruise_velocity);
					ts.setMotionAcceleration(motion_acceleration);
					ts.setMotionSCurveStrength(motion_s_curve_strength);
				}
				else
				{
					tc.resetMotionCruise();
				}
			}
			int motion_profile_trajectory_period;
			if (tc.motionProfileTrajectoryPeriodChanged(motion_profile_trajectory_period))
			{
				if (safeTalonCall(victor->ConfigMotionProfileTrajectoryPeriod(motion_profile_trajectory_period, timeoutMs),"ConfigMotionProfileTrajectoryPeriod"))
				{
					ts.setMotionProfileTrajectoryPeriod(motion_profile_trajectory_period);
					ROS_INFO_STREAM("Updated joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile trajectory period");
				}
				else
				{
					tc.resetMotionProfileTrajectoryPeriod();
				}
			}

			if (tc.clearMotionProfileTrajectoriesChanged())
			{
				if (safeTalonCall(victor->ClearMotionProfileTrajectories(), "ClearMotionProfileTrajectories"))
				{
					ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile trajectories");
				}
				else
				{
					tc.setClearMotionProfileTrajectories();
				}
			}

			if (tc.clearMotionProfileHasUnderrunChanged())
			{
				if (safeTalonCall(victor->ClearMotionProfileHasUnderrun(timeoutMs),"ClearMotionProfileHasUnderrun"))
				{
					ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " motion profile underrun changed");
				}
				else
				{
					tc.setClearMotionProfileHasUnderrun();
				}
			}

			// TODO : check that Talon motion buffer is not full
			// before writing, communicate how many have been written
			// - and thus should be cleared - from the talon_command
			// list of requests.
		}

		// TODO : rewrite this using BufferedTrajectoryPointStream
		std::vector<hardware_interface::TrajectoryPoint> trajectory_points;
		if (tc.motionProfileTrajectoriesChanged(trajectory_points))
		{
			//int i = 0;
			for (auto it = trajectory_points.cbegin(); it != trajectory_points.cend(); ++it)
			{
				ctre::phoenix::motion::TrajectoryPoint pt;
				pt.position = it->position / radians_scale;
				pt.velocity = it->velocity / radians_per_second_scale;
				pt.headingDeg = it->headingRad * 180. / M_PI;
				pt.arbFeedFwd = it->arbFeedFwd;
				pt.auxiliaryPos = it->auxiliaryPos; // TODO : unit conversion?
				pt.auxiliaryVel = it->auxiliaryVel; // TODO : unit conversion?
				pt.auxiliaryArbFeedFwd = it->auxiliaryArbFeedFwd; // TODO : unit conversion?
				pt.profileSlotSelect0 = it->profileSlotSelect0;
				pt.profileSlotSelect1 = it->profileSlotSelect1;
				pt.isLastPoint = it->isLastPoint;
				pt.zeroPos = it->zeroPos;
				pt.timeDur = it->timeDur;
				pt.useAuxPID = it->useAuxPID;
				//ROS_INFO_STREAM("id: " << joint_id << " pos: " << pt.position << " i: " << i++);
			}
			ROS_INFO_STREAM("Added joint " << joint_id << "=" <<
					can_ctre_mc_names_[joint_id] << " motion profile trajectories");
		}

		// Set new motor setpoint if either the mode or the setpoint has been changed
		if (match_data_.isEnabled())
		{
			double command;
			hardware_interface::TalonMode in_mode;
			hardware_interface::DemandType demand1_type_internal;
			double demand1_value;

			const bool b1 = tc.modeChanged(in_mode);
			const bool b2 = tc.commandChanged(command);
			const bool b3 = tc.demand1Changed(demand1_type_internal, demand1_value);

			// ROS_INFO_STREAM("b1 = " << b1 << " b2 = " << b2 << " b3 = " << b3);
			if (b1 || b2 || b3)
			{
				ctre::phoenix::motorcontrol::ControlMode out_mode;
				ctre::phoenix::motorcontrol::DemandType demand1_type_phoenix;
				if (talon_convert_.controlMode(in_mode, out_mode) &&
					talon_convert_.demand1Type(demand1_type_internal, demand1_type_phoenix))
				{
					ts.setSetpoint(command); // set the state before converting it to native units
					switch (out_mode)
					{
						case ctre::phoenix::motorcontrol::ControlMode::Velocity:
							command /= radians_per_second_scale;
							break;
						case ctre::phoenix::motorcontrol::ControlMode::Position:
							command /= radians_scale;
							break;
						case ctre::phoenix::motorcontrol::ControlMode::MotionMagic:
							command /= radians_scale;
							break;
						default:
							break;
					}

#ifdef DEBUG_WRITE
					ROS_INFO_STREAM("called Set(4) on " << joint_id << "=" << can_ctre_mc_names_[joint_id] <<
							" out_mode = " << static_cast<int>(out_mode) << " command = " << command <<
							" demand1_type_phoenix = " << static_cast<int>(demand1_type_phoenix) <<
							" demand1_value = " << demand1_value);
#endif
					ts.setNeutralOutput(false); // maybe make this a part of setSetpoint?

					ts.setTalonMode(in_mode);
					ts.setDemand1Type(demand1_type_internal);
					ts.setDemand1Value(demand1_value);

					victor->Set(out_mode, command, demand1_type_phoenix, demand1_value);
				}
			}
		}
		else
		{
			// Update talon state with requested setpoints for
			// debugging. Don't actually write them to the physical
			// Talons until the robot is re-enabled, though.
			ts.setSetpoint(tc.get());
			ts.setDemand1Type(tc.getDemand1Type());
			ts.setDemand1Value(tc.getDemand1Value());
			if (last_robot_enabled)
			{
				// On the switch from robot enabled to robot disabled, set Talons to ControlMode::Disabled
				// call resetMode() to queue up a change back to the correct mode / setpoint
				// when the robot switches from disabled back to enabled
				tc.resetMode();    // also forces re-write of setpoint
				tc.resetDemand1(); // make sure demand1 type/value is also written on re-enable
				victor->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0);
				ts.setTalonMode(hardware_interface::TalonMode_Disabled);
				ROS_INFO_STREAM("Robot disabled - called Set(Disabled) on " << joint_id << "=" << can_ctre_mc_names_[joint_id]);
			}
		}

		if (tc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(victor->ClearStickyFaults(timeoutMs), "ClearStickyFaults"))
			{
				ROS_INFO_STREAM("Cleared joint " << joint_id << "=" << can_ctre_mc_names_[joint_id] << " sticky_faults");
			}
			else
			{
				tc.setClearStickyFaults();
			}
		}
		talon_command_[joint_id].unlock();
	}

	last_robot_enabled = match_data_.isEnabled();

	for (size_t joint_id = 0; joint_id < num_canifiers_; ++joint_id)
	{
		if (!canifier_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &canifier = canifiers_[joint_id];
		auto &cs = canifier_state_[joint_id];
		auto &cc = canifier_command_[joint_id];

		if (canifier->HasResetOccurred())
		{
			for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
				cc.resetLEDOutput(static_cast<hardware_interface::canifier::LEDChannel>(i));
			for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
				cc.resetGeneralPinOutput(static_cast<hardware_interface::canifier::GeneralPin>(i));
			cc.resetQuadraturePosition();
			cc.resetVelocityMeasurementPeriod();
			cc.resetVelocityMeasurementWindow();
			cc.resetClearPositionOnLimitF();
			cc.resetClearPositionOnLimitR();
			cc.resetClearPositionOnQuadIdx();
			for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
			{
				const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
				cc.resetPWMOutput(pwm_channel);
				cc.resetPWMOutputEnable(pwm_channel);
			}
			for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
				cc.resetStatusFramePeriod(static_cast<hardware_interface::canifier::CANifierStatusFrame>(i));
			for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
				cc.resetControlFramePeriod(static_cast<hardware_interface::canifier::CANifierControlFrame>(i));
		}

		for (size_t i = hardware_interface::canifier::LEDChannel::LEDChannelFirst + 1; i < hardware_interface::canifier::LEDChannel::LEDChannelLast; i++)
		{
			const auto led_channel = static_cast<hardware_interface::canifier::LEDChannel>(i);
			double percent_output;
			ctre::phoenix::CANifier::LEDChannel ctre_led_channel;
			if (cc.ledOutputChanged(led_channel, percent_output) &&
				canifier_convert_.LEDChannel(led_channel, ctre_led_channel))
			{
				if (safeTalonCall(canifier->SetLEDOutput(percent_output, ctre_led_channel), "canifier->SetLEDOutput"))
				{
					cs.setLEDOutput(led_channel, percent_output);
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set LED channel " << i << " to " << percent_output);
				}
				else
				{
					cc.resetLEDOutput(led_channel);
				}
			}
		}
		for (size_t i = hardware_interface::canifier::GeneralPin::GeneralPin_FIRST + 1; i < hardware_interface::canifier::GeneralPin::GeneralPin_LAST; i++)
		{
			const auto general_pin = static_cast<hardware_interface::canifier::GeneralPin>(i);
			bool value;
			bool output_enable;
			ctre::phoenix::CANifier::GeneralPin ctre_general_pin;
			if (cc.generalPinOutputChanged(general_pin, value, output_enable) &&
					canifier_convert_.generalPin(general_pin, ctre_general_pin))
			{
				if (safeTalonCall(canifier->SetGeneralOutput(ctre_general_pin, value, output_enable), "canifier->SetGeneralOutput"))
				{
					cs.setGeneralPinOutput(general_pin, value);
					cs.setGeneralPinOutputEnable(general_pin, output_enable);
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set General Pin " << i <<
							" to enable=" << output_enable << " value=" << value);
				}
				else
				{
					cc.resetGeneralPinOutput(general_pin);
				}
			}
		}

		// Don't bother with all the changed/reset code here, just copy
		// from command to state each time through the write call
		cs.setEncoderTicksPerRotation(cc.getEncoderTicksPerRotation());
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.quadraturePositionChanged(position))
		{
			const double radians_scale = getConversionFactor(cs.getEncoderTicksPerRotation(), hardware_interface::FeedbackDevice_QuadEncoder, hardware_interface::TalonMode_Position) * cs.getConversionFactor();
			if (safeTalonCall(canifier->SetQuadraturePosition(position / radians_scale), "canifier->SetQuadraturePosition"))
			{
				// Don't set state encoder position, let it be read at the next read() call
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set Quadrature Position to " << position);
			}
			else
			{
				cc.resetQuadraturePosition();
			}
		}

		hardware_interface::canifier::CANifierVelocityMeasPeriod period;
		ctre::phoenix::CANifierVelocityMeasPeriod                ctre_period;
		if (cc.velocityMeasurementPeriodChanged(period) &&
			canifier_convert_.velocityMeasurementPeriod(period, ctre_period))
		{
			if (safeTalonCall(canifier->ConfigVelocityMeasurementPeriod(ctre_period), "canifier->ConfigVelocityMeasurementPeriod"))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set velocity measurement Period to " << period);
				cs.setVelocityMeasurementPeriod(period);
			}
			else
			{
				cc.resetVelocityMeasurementPeriod();
			}
		}

		int window;
		if (cc.velocityMeasurementWindowChanged(window))
		{
			if (safeTalonCall(canifier->ConfigVelocityMeasurementWindow(window), "canifier->ConfigVelocityMeasurementWindow"))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set velocity measurement window to " << window);
				cs.setVelocityMeasurementWindow(window);
			}
			else
			{
				cc.resetVelocityMeasurementWindow();
			}
		}

		bool clear_position_on_limit_f;
		if (cc.clearPositionOnLimitFChanged(clear_position_on_limit_f))
		{
			if (safeTalonCall(canifier->ConfigClearPositionOnLimitF(clear_position_on_limit_f), "canifier->ConfigClearPositionOnLimitF"))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set clear position on limit F to " << clear_position_on_limit_f);
				cs.setClearPositionOnLimitF(clear_position_on_limit_f);
			}
			else
			{
				cc.resetClearPositionOnLimitF();
			}
		}

		bool clear_position_on_limit_r;
		if (cc.clearPositionOnLimitRChanged(clear_position_on_limit_r))
		{
			if (safeTalonCall(canifier->ConfigClearPositionOnLimitR(clear_position_on_limit_r), "canifier->ConfigClearPositionOnLimitR"))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set clear position on limit R to " << clear_position_on_limit_r);
				cs.setClearPositionOnLimitR(clear_position_on_limit_r);
			}
			else
			{
				cc.resetClearPositionOnLimitR();
			}
		}

		bool clear_position_on_quad_idx;
		if (cc.clearPositionOnQuadIdxChanged(clear_position_on_quad_idx))
		{
			if (safeTalonCall(canifier->ConfigClearPositionOnQuadIdx(clear_position_on_quad_idx), "canifier->ConfigClearPositionOnQuadIdx"))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
						<< " : Set clear position on quad idx to " << clear_position_on_quad_idx);
				cs.setClearPositionOnQuadIdx(clear_position_on_quad_idx);
			}
			else
			{
				cc.resetClearPositionOnQuadIdx();
			}
		}

		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			bool output_enable;
			if (cc.pwmOutputEnableChanged(pwm_channel, output_enable) &&
				canifier_convert_.PWMChannel(pwm_channel, ctre_pwm_channel))
			{
				if (safeTalonCall(canifier->EnablePWMOutput(ctre_pwm_channel, output_enable), "canifier->EnablePWMOutput"))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set pwm channel " << pwm_channel << " output enable to " << static_cast<int>(output_enable));
					cs.setPWMOutputEnable(pwm_channel, output_enable);
				}
				else
				{
					cc.resetPWMOutputEnable(pwm_channel);
				}
			}
		}

		for (size_t i = hardware_interface::canifier::PWMChannel::PWMChannelFirst + 1; i < hardware_interface::canifier::PWMChannel::PWMChannelLast; i++)
		{
			const auto pwm_channel = static_cast<hardware_interface::canifier::PWMChannel>(i);
			ctre::phoenix::CANifier::PWMChannel ctre_pwm_channel;
			double output;
			if (cc.pwmOutputChanged(pwm_channel, output) &&
				canifier_convert_.PWMChannel(pwm_channel, ctre_pwm_channel))
			{
				if (safeTalonCall(canifier->SetPWMOutput(ctre_pwm_channel, output), "canifier->SetPWMOutput"))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set pwm channel " << pwm_channel << " output to " << output);
					cs.setPWMOutput(pwm_channel, output);
				}
				else
				{
					cc.resetPWMOutput(pwm_channel);
				}
			}
		}

		for (size_t i = hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_First + 1; i < hardware_interface::canifier::CANifierStatusFrame::CANifierStatusFrame_Last; i++)
		{
			const auto frame_id = static_cast<hardware_interface::canifier::CANifierStatusFrame>(i);
			ctre::phoenix::CANifierStatusFrame ctre_frame_id;
			int period;
			if (cc.statusFramePeriodChanged(frame_id, period) &&
				canifier_convert_.statusFrame(frame_id, ctre_frame_id))
			{
				if (safeTalonCall(canifier->SetStatusFramePeriod(ctre_frame_id, period), "canifier->SetStatusFramePeriod"))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set frame_id " << i << " status period to " << period);
					cs.setStatusFramePeriod(frame_id, period);
				}
				else
				{
					cc.resetStatusFramePeriod(frame_id);
				}
			}
		}

		for (size_t i = hardware_interface::canifier::CANifierControlFrame::CANifier_Control_First + 1; i < hardware_interface::canifier::CANifierControlFrame::CANifier_Control_Last; i++)
		{
			const auto frame_id = static_cast<hardware_interface::canifier::CANifierControlFrame>(i);
			ctre::phoenix::CANifierControlFrame ctre_frame_id;
			int period;
			if (cc.controlFramePeriodChanged(frame_id, period) &&
				canifier_convert_.controlFrame(frame_id, ctre_frame_id))
			{
				if (safeTalonCall(canifier->SetControlFramePeriod(ctre_frame_id, period), "canifier->SetControlFramePeriod,"))
				{
					ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id]
							<< " : Set frame_id " << i << " control period to " << period);
					cs.setControlFramePeriod(frame_id, period);
				}
				else
				{
					cc.resetControlFramePeriod(frame_id);
				}
			}
		}

		if (cc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(canifier->ClearStickyFaults(), "canifier->ClearStickyFaults()"))
			{
				ROS_INFO_STREAM("CANifier " << canifier_names_[joint_id] << " : cleared sticky faults");
				// No corresponding status field
			}
			else
			{
				cc.setClearStickyFaults();
			}
		}
	}

	for (size_t joint_id = 0; joint_id < num_cancoders_; ++joint_id)
	{
		if (!cancoder_local_hardwares_[joint_id])
			continue;

		// Save some typing by making references to commonly
		// used variables
		auto &cancoder = cancoders_[joint_id];
		auto &cs = cancoder_state_[joint_id];
		auto &cc = cancoder_command_[joint_id];
		if (cancoder->HasResetOccurred())
		{
			cc.resetPosition();
			cc.resetVelocityMeasPeriod();
			cc.resetVelocityMeasWindow();
			cc.resetAbsoluteSensorRange();
			cc.resetMagnetOffset();
			cc.resetInitializationStrategy();
			cc.resetFeedbackCoefficient();
			cc.resetDirection();
			cc.resetSensorDataStatusFramePeriod();
			cc.resetVBatAndFaultsStatusFramePeriod();
		}
		cs.setConversionFactor(cc.getConversionFactor());
		double position;
		if (cc.positionChanged(position))
		{
			if (safeTalonCall(cancoder->SetPosition(position / cs.getConversionFactor()), "cancoder->SetPosition"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set position to " << position);
				// Don't set state - it will be updated in next read() loop
			}
			else
			{
				cc.resetPosition();
			}
		}
		if (cc.positionToAbsoluteChanged())
		{
			if (safeTalonCall(cancoder->SetPositionToAbsolute(), "cancoder->SetPositionToAbsolute"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set position to absolute");
				// Don't set state - it will be updated in next read() loop
			}
			else
			{
				cc.setPositionToAbsolute();
			}
		}
		hardware_interface::cancoder::SensorVelocityMeasPeriod velocity_meas_period;
		ctre::phoenix::sensors::SensorVelocityMeasPeriod ctre_velocity_meas_period;
		if (cc.velocityMeasPeriodChanged(velocity_meas_period) &&
			cancoder_convert_.velocityMeasPeriod(velocity_meas_period, ctre_velocity_meas_period))
		{
			if (safeTalonCall(cancoder->ConfigVelocityMeasurementPeriod(ctre_velocity_meas_period), "cancoder->ConfigVelocityMeasurementPeriod"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set velocity measurement period to " << static_cast<int>(ctre_velocity_meas_period));
				cs.setVelocityMeasPeriod(velocity_meas_period);
			}
			else
			{
				cc.resetVelocityMeasPeriod();
			}
		}

		int velocity_meas_window;
		if (cc.velocityMeasWindowChanged(velocity_meas_window))
		{
			if (safeTalonCall(cancoder->ConfigVelocityMeasurementWindow(velocity_meas_window), "cancoder->ConfigVelocityMeasurementWindow"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set velocity measurement window to " << velocity_meas_window);
				cs.setVelocityMeasWindow(velocity_meas_window);
			}
			else
			{
				cc.resetVelocityMeasWindow();
			}
		}
		hardware_interface::cancoder::AbsoluteSensorRange absolute_sensor_range;
		ctre::phoenix::sensors::AbsoluteSensorRange ctre_absolute_sensor_range;
		if (cc.absoluteSensorRangeChanged(absolute_sensor_range) &&
			cancoder_convert_.absoluteSensorRange(absolute_sensor_range, ctre_absolute_sensor_range))
		{
			if (safeTalonCall(cancoder->ConfigAbsoluteSensorRange(ctre_absolute_sensor_range), "cancoder->ConfigAbsoluteSensorRange"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set absolute sensor range to " << absolute_sensor_range);
				cs.setAbsoluteSensorRange(absolute_sensor_range);
			}
			else
			{
				cc.resetAbsoluteSensorRange();
			}
		}

		double magnet_offset;
		if (cc.magnetOffsetChanged(magnet_offset))
		{
			if (safeTalonCall(cancoder->ConfigMagnetOffset(magnet_offset), "cancoder->ConfigMagnetOffset"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set magnet offset to " << magnet_offset);
				cs.setMagnetOffset(magnet_offset);
			}
			else
			{
				cc.resetMagnetOffset();
			}
		}

		hardware_interface::cancoder::SensorInitializationStrategy initialization_strategy;
		ctre::phoenix::sensors::SensorInitializationStrategy ctre_initialization_strategy;
		if (cc.InitializationStrategyChanged(initialization_strategy) &&
			cancoder_convert_.initializationStrategy(initialization_strategy, ctre_initialization_strategy))
		{
			if (safeTalonCall(cancoder->ConfigSensorInitializationStrategy(ctre_initialization_strategy), "cancoder->ConfigSensorInitializationStrategy"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set sensor intitialization strategy to " << initialization_strategy);
				cs.setInitializationStrategy(initialization_strategy);
			}
			else
			{
				cc.resetInitializationStrategy();
			}
		}
		double feedback_coefficient;
		std::string unit_string;
		hardware_interface::cancoder::SensorTimeBase time_base;
		ctre::phoenix::sensors::SensorTimeBase ctre_time_base;
		if (cc.feedbackCoefficientChanged(feedback_coefficient, unit_string, time_base) &&
			cancoder_convert_.timeBase(time_base, ctre_time_base))
		{
			if (safeTalonCall(cancoder->ConfigFeedbackCoefficient(feedback_coefficient, unit_string, ctre_time_base), "cancoder->ConfigFeedbackCoefficient"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set feedback coefficient to  " << feedback_coefficient << " " << unit_string << " " << time_base);
				cs.setFeedbackCoefficient(feedback_coefficient);
				cs.setUnitString(unit_string);
				cs.setTimeBase(time_base);
			}
			else
			{
				cc.resetFeedbackCoefficient();
			}
		}

		bool direction;
		if (cc.directionChanged(direction))
		{
			if (safeTalonCall(cancoder->ConfigSensorDirection(direction), "cancoder->ConfigSensorDirection"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set direction to " << direction);
				cs.setDirection(direction);
			}
			else
			{
				cc.resetDirection();
			}
		}

		int sensor_data_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(sensor_data_status_frame_period))
		{
			if (safeTalonCall(cancoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_SensorData, sensor_data_status_frame_period), "cancoder->SetStatusFramePeriod(SensorData)"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set sensor data status frame period to " << sensor_data_status_frame_period);
				cs.setSensorDataStatusFramePeriod(sensor_data_status_frame_period);
			}
			else
			{
				cc.resetSensorDataStatusFramePeriod();
			}
		}

		int vbat_and_faults_status_frame_period;
		if (cc.sensorDataStatusFramePeriodChanged(vbat_and_faults_status_frame_period))
		{
			if (safeTalonCall(cancoder->SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame::CANCoderStatusFrame_VbatAndFaults, vbat_and_faults_status_frame_period), "cancoder->SetStatusFramePeriod(VbatAndFaults)"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id]
						<< " : Set vbat and fault status frame period to " << vbat_and_faults_status_frame_period);
				cs.setVbatAndFaultsStatusFramePeriod(vbat_and_faults_status_frame_period);
			}
			else
			{
				cc.resetVBatAndFaultsStatusFramePeriod();
			}
		}

		if (cc.clearStickyFaultsChanged())
		{
			if (safeTalonCall(cancoder->ClearStickyFaults(), "cancoder->ClearStickyFaults"))
			{
				ROS_INFO_STREAM("CANcoder " << cancoder_names_[joint_id] << " : Sticky faults cleared");
			}
			else
			{
				cc.setClearStickyFaults();
			}
		}
	}

	for (size_t i = 0; i < num_nidec_brushlesses_; i++)
	{
		if (nidec_brushless_local_hardwares_[i])
			nidec_brushlesses_[i]->Set(brushless_command_[i]);
	}

	for (size_t i = 0; i < num_digital_outputs_; i++)
	{
		// Only invert the desired output once, on the controller
		// where the update originated
		const bool converted_command = (digital_output_command_[i] > 0) ^ (digital_output_inverts_[i] && digital_output_local_updates_[i]);
		if (converted_command != digital_output_state_[i])
		{
			if (digital_output_local_hardwares_[i])
				digital_outputs_[i]->Set(converted_command);
			digital_output_state_[i] = converted_command;
			ROS_INFO_STREAM("Wrote digital output " << i << "=" << converted_command);
		}
	}

	for (size_t i = 0; i < num_pwms_; i++)
	{
		const int setpoint = pwm_command_[i] * ((pwm_inverts_[i] & pwm_local_updates_[i]) ? -1 : 1);
		if (pwm_state_[i] != setpoint)
		{
			if (pwm_local_hardwares_[i])
				PWMs_[i]->SetSpeed(setpoint);
			pwm_state_[i] = setpoint;
			ROS_INFO_STREAM("PWM " << pwm_names_[i] <<
					" at channel" <<  pwm_pwm_channels_[i] <<
					" set to " << pwm_state_[i]);
		}
	}

	for (size_t i = 0; i < num_solenoids_; i++)
	{
		const bool setpoint = solenoid_command_[i] > 0;
		if (solenoid_state_[i] != setpoint)
		{
			if (solenoid_local_hardwares_[i])
			{
				int32_t status = 0;
				HAL_SetSolenoid(solenoids_[i], setpoint, &status);
				if (status != 0)
					ROS_ERROR_STREAM("Error setting solenoid " << solenoid_names_[i] <<
							" to " << setpoint << " status = " << status);
			}
			solenoid_state_[i] = setpoint;
			ROS_INFO_STREAM("Solenoid " << solenoid_names_[i] <<
					" at id " << solenoid_ids_[i] <<
					" / pcm " << solenoid_pcms_[i] <<
					" = " << setpoint);
		}
	}

	for (size_t i = 0; i < num_double_solenoids_; i++)
	{
		frc::DoubleSolenoid::Value setpoint = frc::DoubleSolenoid::Value::kOff;
		if (double_solenoid_command_[i] >= 1.0)
			setpoint = frc::DoubleSolenoid::Value::kForward;
		else if (double_solenoid_command_[i] <= -1.0)
			setpoint = frc::DoubleSolenoid::Value::kReverse;

		// Not sure if it makes sense to store command values
		// in state or wpilib enum values
		if (double_solenoid_state_[i] != double_solenoid_command_[i])
		{
			if (double_solenoid_local_hardwares_[i])
			{
				bool forward = false;
				bool reverse = false;
				if (setpoint == frc::DoubleSolenoid::Value::kForward)
					forward = true;
				else if (setpoint == frc::DoubleSolenoid::Value::kReverse)
					reverse = true;

				int32_t status = 0;
				HAL_SetSolenoid(double_solenoids_[i].forward_, forward, &status);
				if (status != 0)
					ROS_ERROR_STREAM("Error setting double solenoid " << double_solenoid_names_[i] <<
							" forward to " << forward << " status = " << status);
				else
					ROS_INFO_STREAM("Setting double solenoid " << double_solenoid_names_[i] <<
							" forward to " << forward);
				status = 0;
				HAL_SetSolenoid(double_solenoids_[i].reverse_, reverse, &status);
				if (status != 0)
					ROS_ERROR_STREAM("Error setting double solenoid " << double_solenoid_names_[i] <<
							" reverse to " << reverse << " status = " << status);
				else
					ROS_INFO_STREAM("Setting double solenoid " << double_solenoid_names_[i] <<
							" reverse to " << reverse);
			}
			double_solenoid_state_[i] = double_solenoid_command_[i];
			ROS_INFO_STREAM("Double solenoid " << double_solenoid_names_[i] <<
					" at forward id " << double_solenoid_forward_ids_[i] <<
					" / reverse id " << double_solenoid_reverse_ids_[i] <<
					" / pcm " << double_solenoid_pcms_[i] <<
					" = " << setpoint);
		}
	}

	for (size_t i = 0; i < num_rumbles_; i++)
	{
		if (rumble_state_[i] != rumble_command_[i])
		{
			const unsigned int rumbles = *((unsigned int*)(&rumble_command_[i]));
			const unsigned int left_rumble  = (rumbles >> 16) & 0xFFFF;
			const unsigned int right_rumble = (rumbles      ) & 0xFFFF;
			if (rumble_local_hardwares_[i])
				HAL_SetJoystickOutputs(rumble_ports_[i], 0, left_rumble, right_rumble);
			rumble_state_[i] = rumble_command_[i];
			ROS_INFO_STREAM("Wrote rumble " << i << "=" << rumble_command_[i]);
		}
	}

	for (size_t i = 0; i < num_compressors_; i++)
	{
		if (compressor_command_[i] != compressor_state_[i])
		{
			const bool setpoint = compressor_command_[i] > 0;
			if (compressor_local_hardwares_[i])
			{
				int32_t status = 0;
				HAL_SetCompressorClosedLoopControl(compressors_[i], setpoint, &status);
				if (status)
					ROS_ERROR_STREAM("SetCompressorClosedLoopControl status:" << status
							<< " " << HAL_GetErrorMessage(status));
			}
			compressor_state_[i] = compressor_command_[i];
			ROS_INFO_STREAM("Wrote compressor " << i << "=" << setpoint);
		}
	}

	for (size_t i = 0; i < num_as726xs_; i++)
	{
		hardware_interface::as726x::IndLedCurrentLimits ind_led_current_limit;
		bool ind_led_enable;
		hardware_interface::as726x::DrvLedCurrentLimits drv_led_current_limit;
		bool drv_led_enable;
		hardware_interface::as726x::ConversionTypes conversion_type;
		hardware_interface::as726x::ChannelGain gain;
		uint8_t integration_time;

		auto &ac = as726x_command_[i];
		const bool ind_led_current_limit_changed = ac.indLedCurrentLimitChanged(ind_led_current_limit);
		const bool ind_led_enable_changed        = ac.indLedEnableChanged(ind_led_enable);
		const bool drv_led_current_limit_changed = ac.drvLedCurrentLimitChanged(drv_led_current_limit);
		const bool drv_led_enable_changed        = ac.drvLedEnableChanged(drv_led_enable);
		const bool conversion_type_changed       = ac.conversionTypeChanged(conversion_type);
		const bool gain_changed                  = ac.gainChanged(gain);
		const bool integration_time_changed      = ac.integrationTimeChanged(integration_time);
		// Only attempt to lock access to the sensor if there's actually
		// any changed config values to write to it
		if (ind_led_current_limit_changed ||
				ind_led_enable_changed ||
				drv_led_current_limit_changed ||
				drv_led_enable_changed ||
				conversion_type_changed ||
				gain_changed ||
				integration_time_changed)
		{
			// Try to get a lock for this instance of AS726x.  If not available
			// due to the read thread accessing the sensor,
			// continue on to the next sensor in the loop one.  This way the code won't be waiting
			// for a mutex to be released - it'll move on and try again next
			// iteration of write()
			std::unique_lock<std::mutex> l(*(as726xs_[i]->getMutex()), std::defer_lock);
			if (!l.try_lock())
			{
				// If we can't get a lock this iteration, reset and try again next time through
				if (ind_led_current_limit_changed)
					ac.resetIndLedCurrentLimit();
				if (ind_led_enable_changed)
					ac.resetIndLedEnable();
				if (drv_led_current_limit_changed)
					ac.resetDrvLedCurrentLimit();
				if (drv_led_enable_changed)
					ac.resetDrvLedEnable();
				if (conversion_type_changed)
					ac.resetConversionType();
				if (gain_changed)
					ac.resetGain();
				if (integration_time_changed)
					ac.resetIntegrationTime();
				continue;
			}
			auto &as = as726x_state_[i];

			if (ind_led_current_limit_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::ind_led_current_limits as726x_ind_led_current_limit;
					if (as726x_convert_.indLedCurrentLimit(ind_led_current_limit, as726x_ind_led_current_limit))
						as726xs_[i]->setIndicateCurrent(as726x_ind_led_current_limit);
				}
				as.setIndLedCurrentLimit(ind_led_current_limit);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " ind_led_current_limit = " << ind_led_current_limit);
			}

			if (ind_led_enable_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726xs_[i]->indicateLED(ind_led_enable);
				}
				as.setIndLedEnable(ind_led_enable);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]=" << as726x_names_[i] << " ind_led_enable = " << ind_led_enable);
			}

			if (drv_led_current_limit_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::drv_led_current_limits as726x_drv_led_current_limit;
					if (as726x_convert_.drvLedCurrentLimit(drv_led_current_limit, as726x_drv_led_current_limit))
						as726xs_[i]->setDrvCurrent(as726x_drv_led_current_limit);
				}
				as.setDrvLedCurrentLimit(drv_led_current_limit);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " drv_led_current_limit = " << drv_led_current_limit);
			}

			if (drv_led_enable_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					if (drv_led_enable)
					{
						as726xs_[i]->drvOn();
					}
					else
					{
						as726xs_[i]->drvOff();
					}
				}
				as.setDrvLedEnable(drv_led_enable);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " drv_led_enable = " << drv_led_enable);
			}

			if (conversion_type_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::conversion_types as726x_conversion_type;
					if (as726x_convert_.conversionType(conversion_type, as726x_conversion_type))
						as726xs_[i]->setConversionType(as726x_conversion_type);
				}
				as.setConversionType(conversion_type);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " conversion_type = " << conversion_type);
			}

			if (gain_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726x::channel_gain as726x_gain;
					if (as726x_convert_.channelGain(gain, as726x_gain))
						as726xs_[i]->setGain(as726x_gain);
				}
				as.setGain(gain);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] << " channel_gain = " << gain);
			}

			if (integration_time_changed)
			{
				if (as726x_local_hardwares_[i])
				{
					as726xs_[i]->setIntegrationTime(integration_time);
				}
				as.setIntegrationTime(integration_time);
				ROS_INFO_STREAM("Wrote as726x_[" << i << "]= " << as726x_names_[i] <<  " integration_time = "
						<< static_cast<int>(integration_time));
			}
		}
	}

        for(size_t i = 0; i < num_talon_orchestras_; i++)
        {
            auto &oc = orchestra_command_[i];
            auto &os = orchestra_state_[i];
            std::string music_file_path;
            std::vector<std::string> instruments;
            if(oc.clearInstrumentsChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->ClearInstruments(), "ClearInstruments"))
                {
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " cleared instruments.");
                }
                else{
                    ROS_ERROR_STREAM("Failed to clear instruments in orchestra.");
                }
            }
            if(oc.instrumentsChanged(instruments))
            {
                if(safeTalonCall(talon_orchestras_[i]->ClearInstruments(), "ClearInstruments"))
                {
                    for(size_t j = 0; j < instruments.size(); j++)
                    {
                        size_t can_index = std::numeric_limits<size_t>::max();
                        for(size_t k = 0; k < can_ctre_mc_names_.size(); k++)
                        {
                            if(can_ctre_mc_names_[k] == instruments[j])
                            {
                                can_index = k;
                                break;
                            }
                        }
                        if(can_index == std::numeric_limits<size_t>::max())
                        {
                            ROS_ERROR_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " failed to add " << instruments[j] << " because it does not exist");
                        }
                        else if(can_ctre_mc_is_talon_fx_[can_index])
                        {
                            if(safeTalonCall(talon_orchestras_[i]->AddInstrument(*(std::dynamic_pointer_cast<ctre::phoenix::motorcontrol::can::TalonFX>(ctre_mcs_[can_index]))), "AddInstrument"))
                                ROS_INFO_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " added Falcon " << "falcon_name");
                            else{
                                ROS_ERROR_STREAM("Failed to add instrument to orchestra");
                                oc.resetInstruments();
                            }
                        }
                        else
                            ROS_INFO_STREAM("Talon Orchestra " <<  talon_orchestra_names_[i] << " failed to add " << instruments[j] << " because it is not a TalonFX");
                    }
                    os.setInstruments(instruments);
                }
                else{
                    ROS_ERROR_STREAM("Failed to clear instruments in orchestra");
                    oc.resetClearInstruments();
                }
            }
            if(oc.musicChanged(music_file_path))
            {
                if(safeTalonCall(talon_orchestras_[i]->LoadMusic(music_file_path), "LoadMusic"))
                {
                    os.setChirpFilePath(music_file_path);
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " loaded music at " << music_file_path);
                }
                else{
                    ROS_ERROR_STREAM("Failed to load music into orchestra");
                    oc.resetMusic();
                }
            }
            if(oc.pauseChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->Pause(), "Pause"))
                {
                    os.setPaused();
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " pausing");
                }
                else{
                    ROS_ERROR_STREAM("Failed to pause orchestra");
                    oc.pause();
                }
            }
            if(oc.playChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->Play(), "Play"))
                {
                    os.setPlaying();
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " playing");
                }
                else{
                    ROS_ERROR_STREAM("Failed to play orchestra");
                    oc.play();
                }
            }
            if(oc.stopChanged())
            {
                if(safeTalonCall(talon_orchestras_[i]->Stop(), "Stop"))
                {
                    os.setStopped();
                    ROS_INFO_STREAM("Talon Orchestra " << talon_orchestra_names_[i] << " stopping");
                }
                else{
                    ROS_ERROR_STREAM("Failed to stop orchestra");
                    oc.stop();
                }
            }
        }

	// TODO : what to do about this?
	for (size_t i = 0; i < num_dummy_joints_; i++)
	{
		if (dummy_joint_locals_[i])
		{
			// Use dummy joints to communicate info between
			// various controllers and driver station smartdash vars
			{
				dummy_joint_effort_[i] = 0;
				//if (dummy_joint_names_[i].substr(2, std::string::npos) == "_angle")
				{
					// position mode
					dummy_joint_velocity_[i] = (dummy_joint_command_[i] - dummy_joint_position_[i]) / period.toSec();
					dummy_joint_position_[i] = dummy_joint_command_[i];
				}
#if 0
				else if (dummy_joint_names_[i].substr(2, std::string::npos) == "_drive")
				{
					// velocity mode
					dummy_joint_position_[i] += dummy_joint_command_[i] * period.toSec();
					dummy_joint_velocity_[i] = dummy_joint_command_[i];
				}
#endif
			}
		}
	}
}

bool FRCRobotHWInterface::DSErrorCallback(ros_control_boilerplate::DSError::Request &req, ros_control_boilerplate::DSError::Response &res)
{
	ROS_ERROR_STREAM("HWI received DSErrorCallback " << req.details.c_str());
	HAL_SendError(true, req.error_code, false, req.details.c_str(), "", "", true);
	return true;
}

} // namespace
