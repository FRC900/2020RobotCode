#pragma once

#include <string>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{
enum SensorVelocityMeasPeriod
{
	Sensor_Period_1Ms = 1,
	Sensor_Period_2Ms = 2,
	Sensor_Period_5Ms = 5,
	Sensor_Period_10Ms = 10,
	Sensor_Period_20Ms = 20,
	Sensor_Period_25Ms = 25,
	Sensor_Period_50Ms = 50,
	Sensor_Period_100Ms = 100,
};
enum AbsoluteSensorRange
{
	/**
	 * Express the absolute position as an unsigned value.
	 * E.g. [0,+1) rotations or [0,360) deg.
	 */
	Unsigned_0_to_360 = 0,
	/**
	 * Express the absolute position as an signed value.
	 * E.g. [-0.5,+0.5) rotations or [-180,+180) deg.
	 */
	Signed_PlusMinus180 = 1,
};
enum SensorInitializationStrategy
{
	/**
	 * On boot up, set position to zero.
	 */
	BootToZero = 0,
	/**
	 * On boot up, sync to the Absolute Position signal.  The Absolute position signal will be signed according to the selected Absolute Sensor Range.
	 */
	BootToAbsolutePosition = 1,
};
enum SensorTimeBase
{
	/**
	 * Legacy Mode
	 */
	Per100Ms_Legacy = 0,
	/**
	 * Per-Second Velocities
	 */
	PerSecond = 1,
	/**
	 * Per-Minute Velocities
	 */
	PerMinute = 2,
};
enum MagnetFieldStrength
{
	/** Magnet Field strength cannot be determined */
	Invalid_Unknown = 0,
	/** Magnet field is far too low (too far) or far too high (too close). */
	BadRange_RedLED = 1,
	/** Magnet field is adequate, sensor can be used in this range with slightly reduced accuracy. */
	Adequate_OrangeLED = 2,
	/** Magnet field is ideal */
	Good_GreenLED = 3,
};

class CANCoderHWState
{
	public:
		CANCoderHWState(int device_number)
			: device_number_(device_number)
			, position_(0)
			, velocity_(0)
			, absolute_position_(0)
			, velocity_meas_period_(Sensor_Period_100Ms)
			, velocity_meas_window_(64)
			, absolute_sensor_range_(Unsigned_0_to_360)
			, magnet_offset_(0) // NOTE : degrees
			, initialization_strategy_(BootToZero)
			, feedback_coefficient_(0.087890625)
			, unit_string_("deg")
			, time_base_(PerSecond)
			, bus_voltage_(0)
			, magnet_field_strength_(Invalid_Unknown)
			, direction_(false)
			, sensor_data_status_frame_period_(-1)
			, vbat_and_faults_status_frame_period_(-1)
			, firmware_version_(-1)
	{
	}

		int getDeviceNumber(void) const
		{
			return device_number_;
		}

		double getPosition(void) const
		{
			return position_;
		}
		void setPosition(double position)
		{
			position_ = position;
		}

		double getVelocity(void) const
		{
			return velocity_;
		}
		void setVelocity(double velocity)
		{
			velocity_ = velocity;
		}

		double getAbsolutePosition(void) const
		{
			return absolute_position_;
		}
		void setAbsolutePosition(double absolute_position)
		{
			absolute_position_ = absolute_position;
		}

		SensorVelocityMeasPeriod getVelocityMeasPeriod(void) const
		{
			return velocity_meas_period_;
		}
		void setVelocityMeasPeriod(SensorVelocityMeasPeriod velocity_meas_period)
		{
			velocity_meas_period_ = velocity_meas_period;
		}

		int getVelocityMeasWindow(void) const
		{
			return velocity_meas_window_;
		}
		void setVelocityMeasWindow(int velocity_meas_window)
		{
			velocity_meas_window_ = velocity_meas_window;
		}

		AbsoluteSensorRange getAbsoluteSensorRange(void) const
		{
			return absolute_sensor_range_;
		}
		void setAbsoluteSensorRange(AbsoluteSensorRange absolute_sensor_range)
		{
			absolute_sensor_range_ = absolute_sensor_range;
		}

		double getMagnetOffset(void) const
		{
			return magnet_offset_;
		}
		void setMagnetOffset(double magnet_offset)
		{
			magnet_offset_ = magnet_offset;
		}

		SensorInitializationStrategy getInitializationStrategy(void) const
		{
			return initialization_strategy_;
		}
		void setInitializationStrategy(SensorInitializationStrategy initialization_strategy)
		{
			initialization_strategy_ = initialization_strategy;
		}

		double getFeedbackCoefficient(void) const
		{
			return feedback_coefficient_;
		}
		void setFeedbackCoefficient(double feedback_coefficient)
		{
			feedback_coefficient_ = feedback_coefficient;
		}

		std::string getUnitString(void) const
		{
			return unit_string_;
		}
		void setUnitString(const std::string &unit_string)
		{
			unit_string_ = unit_string;
		}

		SensorTimeBase getTimeBase(void) const
		{
			return time_base_;
		}
		void setTimeBase(SensorTimeBase time_base)
		{
			time_base_ = time_base;
		}

		double getBusVoltage(void) const
		{
			return bus_voltage_;
		}
		void setBusVoltage(double bus_voltage)
		{
			bus_voltage_ = bus_voltage;
		}

		MagnetFieldStrength getMagnetFieldStrength(void) const
		{
			return magnet_field_strength_;
		}
		void setMagnetFieldStrength(MagnetFieldStrength magnet_field_strength)
		{
			magnet_field_strength_ = magnet_field_strength;
		}

		bool getDirection(void) const
		{
			return direction_;
		}
		void setDirection(bool direction)
		{
			direction_ = direction;
		}

		int getSensorDataStatusFramePeriod(void) const
		{
			return sensor_data_status_frame_period_;
		}
		void setSensorDataStatusFramePeriod(int sensor_data_status_frame_period)
		{
			sensor_data_status_frame_period_ = sensor_data_status_frame_period;
		}

		int getVbatAndFaultsStatusFramePeriod(void) const
		{
			return vbat_and_faults_status_frame_period_;
		}
		void setVbatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period)
		{
			vbat_and_faults_status_frame_period_ = vbat_and_faults_status_frame_period;
		}

		int getFirmwareVersion(void) const
		{
			return firmware_version_;
		}
		void setFirmwareVersion(int firmware_version)
		{
			firmware_version_ = firmware_version;
		}

		int getFaults(void) const
		{
			return faults_;
		}
		void setFaults(int faults)
		{
			faults_ = faults;
		}
		int getStickyFaults(void) const
		{
			return sticky_faults_;
		}
		void setStickyFaults(int sticky_faults)
		{
			sticky_faults_ = sticky_faults;
		}

	private :
		int                          device_number_;
		double                       position_;
		double                       velocity_;
		double                       absolute_position_;
		SensorVelocityMeasPeriod     velocity_meas_period_;
		int                          velocity_meas_window_;
		AbsoluteSensorRange          absolute_sensor_range_;
		double                       magnet_offset_;
		SensorInitializationStrategy initialization_strategy_;
		double                       feedback_coefficient_;
		std::string                  unit_string_;
		SensorTimeBase               time_base_;
		double                       bus_voltage_;
		MagnetFieldStrength          magnet_field_strength_;
		bool                         direction_;
		int                          sensor_data_status_frame_period_;
		int                          vbat_and_faults_status_frame_period_;
		int                          firmware_version_;
		int                          faults_;
		int                          sticky_faults_;
};
// Glue code to let this be registered in the list of
// hardware resources on the robot.  Since state is
// read-only, allow multiple controllers to register it.
typedef StateHandle<const CANCoderHWState> CANCoderStateHandle;
typedef StateHandle<CANCoderHWState>       CANCoderWritableStateHandle;
class CANCoderStateInterface : public HardwareResourceManager<CANCoderStateHandle> {};

} // namespace
