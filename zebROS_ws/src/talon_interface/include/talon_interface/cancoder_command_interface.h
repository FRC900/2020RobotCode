#pragma once

#include <talon_interface/cancoder_state_interface.h>
namespace hardware_interface
{
class CANCoderHWCommand
{
	public:
		CANCoderHWCommand(void)
		: position_(0)
		, position_changed_(false)
		, set_position_to_absolute_(false)
		, velocity_meas_period_(Sensor_Period_100Ms)
		, velocity_meas_period_changed_(true)
		, velocity_meas_window_(64)
		, velocity_meas_window_changed_(true)
		, absolute_sensor_range_(Unsigned_0_to_360)
		, absolute_sensor_range_changed_(true)
		, magnet_offset_(0)
		, magnet_offset_changed_(false)
		, initialization_strategy_(BootToZero)
		, initialization_strategy_changed_(true)
		, feedback_coefficient_(0.087890625)
		, unit_string_("deg")
		, time_base_(PerSecond)
		, feedback_coefficient_changed_(true)
		, direction_(false)
		, direction_changed_(true)
		, sensor_data_status_frame_period_(100)
		, sensor_data_status_frame_period_changed_(true)
		, vbat_and_faults_status_frame_period_(100)
		, vbat_and_faults_status_frame_period_changed_(true)
		{
		}

		double getPosition(void) const
		{
			return position_;
		}
		void setPosition(double position)
		{
			if (position != position_)
			{
				position_ = position;
				position_changed_ = true;
			}
		}
		bool positionChanged(double &position)
		{
			position = position_;
			bool rc = position_changed_;
			position_changed_ = false;
			return rc;
		}
		void resetPosition(void)
		{
			position_changed_ = true;
		}

		bool getPositionToAbsolute(void) const
		{
			return set_position_to_absolute_;
		}
		void setPositionToAbsolute(void)
		{
			set_position_to_absolute_ = true;
		}
		bool positionToAbsoluteChanged(void)
		{
			bool rc = set_position_to_absolute_;
			set_position_to_absolute_ = false;
			return rc;
		}
		void resetPositionToAbsolute(void)
		{
			set_position_to_absolute_ = true;
		}

		SensorVelocityMeasPeriod getVelocityMeasPeriod(void) const
		{
			return velocity_meas_period_;
		}
		void setVelocityMeasPeriod(SensorVelocityMeasPeriod velocity_meas_period)
		{
			if (velocity_meas_period != velocity_meas_period_)
			{
				velocity_meas_period_ = velocity_meas_period;
				velocity_meas_period_changed_ = true;
			}
		}
		bool velocityMeasPeriodChanged(SensorVelocityMeasPeriod &velocity_meas_period)
		{
			velocity_meas_period = velocity_meas_period_;
			bool rc = velocity_meas_period_changed_;
			velocity_meas_period_changed_ = false;
			return rc;
		}
		void resetVelocityMeasPeriod(void)
		{
			velocity_meas_period_changed_ = true;
		}

		int getVelocityMeasWindow(void) const
		{
			return velocity_meas_window_;
		}
		void setVelocityMeasWindow(int velocity_meas_window)
		{
			if (velocity_meas_window != velocity_meas_window_)
			{
				velocity_meas_window_ = velocity_meas_window;
				velocity_meas_window_changed_ = true;
			}
		}
		bool velocityMeasWindowChanged(int &velocity_meas_window)
		{
			velocity_meas_window = velocity_meas_window_;
			bool rc = velocity_meas_window_changed_;
			velocity_meas_window_changed_ = false;
			return rc;
		}
		void resetVelocityMeasWindow(void)
		{
			velocity_meas_window_changed_ = true;
		}

		AbsoluteSensorRange getAbsoluteSensorRange(void) const
		{
			return absolute_sensor_range_;
		}
		void setAbsoluteSensorRange(AbsoluteSensorRange absolute_sensor_range)
		{
			if (absolute_sensor_range != absolute_sensor_range_)
			{
				absolute_sensor_range_ = absolute_sensor_range;
				absolute_sensor_range_changed_ = true;
			}
		}
		bool absoluteSensorRangeChanged(AbsoluteSensorRange &absolute_sensor_range)
		{
			absolute_sensor_range = absolute_sensor_range_;
			bool rc = absolute_sensor_range_changed_;
			absolute_sensor_range_changed_ = false;
			return rc;
		}
		void resetAbsoluteSensorRange(void)
		{
			absolute_sensor_range_changed_ = true;
		}

		double getMagnetOffset(void) const
		{
			return magnet_offset_;
		}
		void setMagnetOffset(double magnet_offset)
		{
			if (magnet_offset != magnet_offset_)
			{
				magnet_offset_ = magnet_offset;
				magnet_offset_changed_ = true;
			}
		}
		bool magnetOffsetChanged(double &magnet_offset)
		{
			magnet_offset = magnet_offset_;
			bool rc = magnet_offset_changed_;
			magnet_offset_changed_ = false;
			return rc;
		}
		void resetMagnetOffset(void)
		{
			magnet_offset_changed_ = true;
		}

		double getInitializationStrategy(void) const
		{
			return initialization_strategy_;
		}
		void setInitializationStrategy(SensorInitializationStrategy initialization_strategy)
		{
			if (initialization_strategy != initialization_strategy_)
			{
				initialization_strategy_ = initialization_strategy;
				initialization_strategy_changed_ = true;
			}
		}
		bool InitializationStrategyChanged(SensorInitializationStrategy &initialization_strategy)
		{
			initialization_strategy = initialization_strategy_;
			bool rc = initialization_strategy_changed_;
			initialization_strategy_changed_ = false;
			return rc;
		}
		void resetInitializationStrategy(void)
		{
			initialization_strategy_changed_ = true;
		}

		double getFeedbackCoefficient(void) const
		{
			return feedback_coefficient_;
		}
		void setFeedbackCoefficient(double feedback_coefficient)
		{
			if (feedback_coefficient_ != feedback_coefficient_)
			{
				feedback_coefficient_ = feedback_coefficient;
				feedback_coefficient_changed_ = true;
			}
		}

		std::string getUnitString(void) const
		{
			return unit_string_;
		}
		void setUnitString(const std::string &unit_string)
		{
			if (unit_string_ != unit_string)
			{
				unit_string_ = unit_string;
				feedback_coefficient_changed_ = true;
			}
		}
		SensorTimeBase getTimeBase(void) const
		{
			return time_base_;
		}
		void setTimeBase(SensorTimeBase time_base)
		{
			if (time_base_ != time_base)
			{
				time_base_ = time_base;
				feedback_coefficient_changed_ = true;
			}
		}
		bool feedbackCoefficientChanged(
				double feedback_coefficient,
				std::string &unit_string,
				SensorTimeBase &time_base)
		{
			feedback_coefficient = feedback_coefficient_;
			unit_string = unit_string_;
			time_base = time_base_;
			bool rc = feedback_coefficient_changed_;
			feedback_coefficient_changed_ = false;
			return rc;
		}
		void resetFeedbackCoefficient(void)
		{
			feedback_coefficient_changed_ = true;
		}

		double getDirection(void) const
		{
			return direction_;
		}
		void setDirection(double direction)
		{
			if (direction != direction_)
			{
				direction_ = direction;
				direction_changed_ = true;
			}
		}
		bool directionChanged(double &direction)
		{
			direction = direction_;
			bool rc = direction_changed_;
			direction_changed_ = false;
			return rc;
		}
		void resetDirection(void)
		{
			direction_changed_ = true;
		}

		int getSensorDataStatusFramePeriod(void) const
		{
			return sensor_data_status_frame_period_;
		}
		void setSensorDataStatusFramePeriod(int sensor_data_status_frame_period)
		{
			if (sensor_data_status_frame_period != sensor_data_status_frame_period_)
			{
				sensor_data_status_frame_period_ = sensor_data_status_frame_period;
				sensor_data_status_frame_period_changed_ = true;
			}
		}
		bool sensorDataStatusFramePeriodChanged(int &sensor_data_status_frame_period)
		{
			sensor_data_status_frame_period = sensor_data_status_frame_period_;
			bool rc = sensor_data_status_frame_period_changed_;
			sensor_data_status_frame_period_changed_ = false;
			return rc;
		}
		void resetSensorDataStatusFramePeriod(void)
		{
			sensor_data_status_frame_period_changed_ = true;
		}

		int getVBatAndFaultsStatusFramePeriod(void) const
		{
			return vbat_and_faults_status_frame_period_;
		}
		void setVBatAndFaultsStatusFramePeriod(int vbat_and_faults_status_frame_period)
		{
			if (vbat_and_faults_status_frame_period != vbat_and_faults_status_frame_period_)
			{
				vbat_and_faults_status_frame_period_ = vbat_and_faults_status_frame_period;
				vbat_and_faults_status_frame_period_changed_ = true;
			}
		}
		bool vbatAndFaultsStatusFramePeriodChanged(int &vbat_and_faults_status_frame_period)
		{
			vbat_and_faults_status_frame_period = vbat_and_faults_status_frame_period_;
			bool rc = vbat_and_faults_status_frame_period_changed_;
			vbat_and_faults_status_frame_period_changed_ = false;
			return rc;
		}
		void resetVBatAndFaultsStatusFramePeriod(void)
		{
			vbat_and_faults_status_frame_period_changed_ = true;
		}


	private:
		double                       position_;
		bool                         position_changed_;
		bool                         set_position_to_absolute_; // one-shot, no need for changed flag
		SensorVelocityMeasPeriod     velocity_meas_period_;
		bool                         velocity_meas_period_changed_;
		int                          velocity_meas_window_;
		bool                         velocity_meas_window_changed_;
		AbsoluteSensorRange          absolute_sensor_range_;
		bool                         absolute_sensor_range_changed_;
		double                       magnet_offset_;
		bool                         magnet_offset_changed_;
		SensorInitializationStrategy initialization_strategy_;
		bool                         initialization_strategy_changed_;
		double                       feedback_coefficient_;
		std::string                  unit_string_;
		SensorTimeBase               time_base_;
		bool                         feedback_coefficient_changed_;
		bool                         direction_;
		bool                         direction_changed_;
		int                          sensor_data_status_frame_period_;
		bool                         sensor_data_status_frame_period_changed_;
		int                          vbat_and_faults_status_frame_period_;
		bool                         vbat_and_faults_status_frame_period_changed_;
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a CANCoder
class CANCoderCommandHandle: public CANCoderStateHandle
{
	public:
		CANCoderCommandHandle(void) :
			CANCoderStateHandle(),
			cmd_(nullptr)
		{
		}

		CANCoderCommandHandle(const CANCoderStateHandle &sh, CANCoderHWCommand *cmd) :
			CANCoderStateHandle(sh),
			cmd_(cmd)
		{
			if (!cmd_)
				throw HardwareInterfaceException("Cannot create CANCoder handle '" + sh.getName() + "'. command pointer is null.");
		}

		// Operator which allows access to methods from
		// the CANCoderHWCommand member var associated with this
		// handle
		// Note that we could create separate methods in
		// the handle class for every method in the HWState
		// class, e.g.
		//     double getFoo(void) const {assert(_state); return state_->getFoo();}
		// but if each of them just pass things unchanged between
		// the calling code and the HWState method there's no
		// harm in making a single method to do so rather than
		// dozens of getFoo() one-line methods
		//
		CANCoderHWCommand *operator->()
		{
			assert(cmd_);
			return cmd_;
		}

		// Get a pointer to the HW state associated with
		// this CANCoder.  Since CommandHandle is derived
		// from StateHandle, there's a state embedded
		// in each instance of a CommandHandle. Use
		// this method to access it.
		//
		// handle->state()->getCANID();
		//
		const CANCoderHWState *state(void) const
		{
			return CANCoderStateHandle::operator->();
		}

	private:
		CANCoderHWCommand *cmd_;
};


// Use ClaimResources here since we only want 1 controller
// to be able to access a given CANCoder at any particular time
class CANCoderCommandInterface : public HardwareResourceManager<CANCoderCommandHandle, ClaimResources> {};

} // namespace
