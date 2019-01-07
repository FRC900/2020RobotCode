#pragma once
#include <spark_max_interface/spark_max_state_interface.h>

namespace hardware_interface
{

class SparkMaxHWCommand
{
	public:
		SparkMaxHWCommand()
			: set_point_(0)
			, set_point_changed_(false)
			, inverted_(false)
			, inverted_changed_(false)
			, p_gain_{0}
			, i_gain_{0}
			, d_gain_{0}
			, f_gain_{0}
			, i_zone_{0}
			, d_filter_{0}
			, pidf_constants_changed_{false}
			, pidf_output_min_{-1}
			, pidf_output_max_{1}
			, pidf_output_range_changed_{false}
			, pidf_reference_value_{0}
			, pidf_reference_ctrl_{kDutyCycle}
			, pidf_arb_feed_forward_{0}
			, pidf_arb_feed_forward_units_{ArbFFUnits::kVoltage}
			, pidf_reference_changed_{false}
			, pidf_reference_slot_(0)
			, pidf_reference_slot_changed_(false)
			, forward_limit_switch_polarity_(kNormallyOpen)
			, forward_limit_switch_enabled_(false)
			, forward_limit_switch_changed_(false)
			, reverse_limit_switch_polarity_(kNormallyOpen)
			, reverse_limit_switch_enabled_(false)
			, reverse_limit_switch_changed_(false)
			, current_limit_(0) // TODO: Better defaults
			, current_limit_one_changed_(false)
			, current_limit_stall_(0)
			, current_limit_free_(0)
			, current_limit_rpm_(0)
			, current_limit_changed_(false)
			, secondary_current_limit_(0)
			, secondary_current_limit_cycles_(0)
			, secondary_current_limit_changed_(false)

			, idle_mode_(kCoast)
			, idle_mode_changed_(false)

			, voltage_compensation_enable_(false)
			, voltage_compensation_nominal_voltage_(12.)
			, voltage_compensation_changed_(false)

			, open_loop_ramp_rate_(0)
			, open_loop_ramp_rate_changed_(false)
			, closed_loop_ramp_rate_(0)
			, closed_loop_ramp_rate_changed_(false)

			, follower_type_(kFollowerDisabled)
			, follower_id_(-1)
			, follower_invert_(false)
			, follower_changed_(false)

			, forward_softlimit_enable_(false)
			, forward_softlimit_(0.0)
			, forward_softlimit_changed_(false)

			, reverse_softlimit_enable_(false)
			, reverse_softlimit_(0.0)
			, reverse_softlimit_changed_(false)

			, encoder_ticks_per_rotation_(4096)
			, encoder_type_(kNoSensor)
			, encoder_type_changed_(false)
		{
		}

		void setSetPoint(double set_point)
		{
			if (set_point_ != set_point)
			{
				set_point_ = set_point;
				set_point_changed_ = true;
			}
		}
		double getSetPoint(void) const
		{
			return set_point_;
		}
		bool changedSetPoint(double &set_point)
		{
			set_point = set_point_;
			if (set_point_changed_)
			{
				set_point_changed_ = false;
				return true;
			}
			return false;
		}
		void resetSetPoint(void)
		{
			set_point_changed_ = true;
		}

		void setInverted(bool inverted)
		{
			if (inverted_ != inverted)
			{
				inverted_ = inverted;
				inverted_changed_ = true;
			}
		}
		bool getInverted(void) const
		{
			return inverted_;
		}
		bool changedInverted(bool &inverted)
		{
			inverted = inverted_;
			if (inverted_changed_)
			{
				inverted_changed_ = false;
				return true;
			}
			return false;
		}
		void resetInverted(void)
		{
			inverted_changed_ = true;
		}

		void setPGain(size_t slot, double p_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPGain() : invalid slot " << slot);
				return;
			}
			if (p_gain != p_gain_[slot])
			{
				p_gain_[slot] = p_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getPGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPGain() : invalid slot " << slot);
				return -1;
			}
			return p_gain_[slot];
		}

		void setIGain(size_t slot, double i_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setIGain() : invalid slot " << slot);
				return;
			}
			if (i_gain != i_gain_[slot])
			{
				i_gain_[slot] = i_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getIGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getIGain() : invalid slot " << slot);
				return -1;
			}
			return i_gain_[slot];
		}
		void setDGain(size_t slot, double d_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setDGain() : invalid slot " << slot);
				return;
			}
			if (d_gain != d_gain_[slot])
			{
				d_gain_[slot] = d_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getDGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getDGain() : invalid slot " << slot);
				return -1;
			}
			return d_gain_[slot];
		}

		void setFGain(size_t slot, double f_gain)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setFGain() : invalid slot " << slot);
				return;
			}
			if (f_gain != f_gain_[slot])
			{
				f_gain_[slot] = f_gain;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getFGain(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getFGain() : invalid slot " << slot);
				return -1;
			}
			return f_gain_[slot];
		}

		void setIZone(size_t slot, double i_zone)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setIZone() : invalid slot " << slot);
				return;
			}
			if (i_zone != i_zone_[slot])
			{
				i_zone_[slot] = i_zone;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getIZone(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getIZone() : invalid slot " << slot);
				return -1;
			}
			return i_zone_[slot];
		}

		void setDFilter(size_t slot, double d_filter)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setDFilter() : invalid slot " << slot);
				return;
			}
			if (d_filter != d_filter_[slot])
			{
				d_filter_[slot] = d_filter;
				pidf_constants_changed_[slot] = true;
			}
		}
		double getDFilter(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getDFilter() : invalid slot " << slot);
				return -1;
			}
			return d_filter_[slot];
		}

		bool changedPIDFConstants(size_t slot,
				double &p_gain, double &i_gain,
				double &d_gain, double &f_gain,
				double &i_zone, double &d_filter)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDConstants() : invalid slot " << slot);
				return false;
			}
			p_gain = p_gain_[slot];
			i_gain = i_gain_[slot];
			d_gain = d_gain_[slot];
			f_gain = f_gain_[slot];
			i_zone = i_zone_[slot];
			d_filter = d_filter_[slot];
			if (pidf_constants_changed_[slot])
			{
				pidf_constants_changed_[slot] = false;
				return true;
			}
			return false;
		}
		void resetPIDFConstants(size_t slot)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::resetPIDFConstants() : invalid slot " << slot);
				return;
			}
			pidf_constants_changed_[slot] = true;
		}

		void setPIDFOutputMin(size_t slot, double pidf_output_min)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFOutputMin() : invalid slot " << slot);
				return;
			}
			if (pidf_output_min != pidf_output_min_[slot])
			{
				pidf_output_min_[slot] = pidf_output_min;
				pidf_output_range_changed_[slot] = true;
			}
		}
		double getPIDFOutputMin(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFOutputMin() : invalid slot " << slot);
				return std::numeric_limits<double>::max();
			}
			return pidf_output_min_[slot];
		}

		void setPIDFOutputMax(size_t slot, double pidf_output_max)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFOutputMax() : invalid slot " << slot);
				return;
			}
			if (pidf_output_max != pidf_output_max_[slot])
			{
				pidf_output_max_[slot] = pidf_output_max;
				pidf_output_range_changed_[slot] = true;
			}
		}
		double getPIDFOutputMax(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFOutputMax() : invalid slot " << slot);
				return -std::numeric_limits<double>::max();
			}
			return pidf_output_max_[slot];
		}
		bool changedPIDOutputRange(size_t slot,
				double &output_min,
				double &output_max)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::changedPIDOutputRange() : invalid slot " << slot);
				return false;
			}
			output_min = pidf_output_min_[slot];
			output_max = pidf_output_max_[slot];
			if (pidf_output_range_changed_[slot])
			{
				pidf_output_range_changed_[slot] = false;
				return true;
			}
			return false;
		}
		void resetPIDOutputRange(size_t slot)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::resetPIDOutputRange() : invalid slot " << slot);
				return;
			}
			pidf_output_range_changed_[slot] = true;
		}

		void setPIDFReferenceValue(size_t slot, double pidf_reference_value)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceValue() : invalid slot " << slot);
				return;
			}
			if (pidf_reference_value != pidf_reference_value_[slot])
			{
				pidf_reference_value_[slot] = pidf_reference_value;
				pidf_reference_changed_[slot] = true;
			}
		}
		double getPIDFReferenceValue(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceValue() : invalid slot " << slot);
				return -1;
			}
			return pidf_reference_value_[slot];
		}

		void setPIDFReferenceCtrl(size_t slot, ControlType pidf_reference_ctrl)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWState::setPIDFReferenceCtrl() : invalid slot " << slot);
				return;
			}
			if (pidf_reference_ctrl != pidf_reference_ctrl_[slot])
			{
				pidf_reference_ctrl_[slot] = pidf_reference_ctrl;
				pidf_reference_changed_[slot] = true;
			}
		}
		ControlType getPIDFReferenceCtrl(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWState::getPIDFReferenceCtrl() : invalid slot " << slot);
				return kDutyCycle;
			}
			return pidf_reference_ctrl_[slot];
		}
		void setPIDFArbFeedForward(size_t slot, double pidf_arb_feed_forward)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFArbFeedForward() : invalid slot " << slot);
				return;
			}
			if (pidf_arb_feed_forward != pidf_arb_feed_forward_[slot])
			{
				pidf_arb_feed_forward_[slot] = pidf_arb_feed_forward;
				pidf_reference_changed_[slot] = true;
			}
		}
		double getPIDFArbFeedForward(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFArbFeedForward() : invalid slot " << slot);
				return -1;
			}
			return pidf_arb_feed_forward_[slot];
		}
		void setPIDFArbFeedForwardUnits(size_t slot, ArbFFUnits pidf_arb_feed_forward)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFArbFeedForwardUnits() : invalid slot " << slot);
				return;
			}
			if (pidf_arb_feed_forward != pidf_arb_feed_forward_units_[slot])
			{
				pidf_arb_feed_forward_units_[slot] = pidf_arb_feed_forward;
				pidf_reference_changed_[slot] = true;
			}
		}
		ArbFFUnits getPIDFArbFeedForwardUnits(size_t slot) const
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::getPIDFArbFeedForwardUnits() : invalid slot " << slot);
				return ArbFFUnits::kVoltage; // probably should throw something here
			}
			return pidf_arb_feed_forward_units_[slot];
		}
		bool changedPIDFReference(size_t slot,
				double &pidf_reference_value,
				ControlType &pidf_reference_ctrl,
				double &pidf_arb_feed_forward,
				ArbFFUnits &pidf_arb_feed_forward_units)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::changedPIDFReference() : invalid slot " << slot);
				return false;
			}
			pidf_reference_value = pidf_reference_value_[slot];
			pidf_reference_ctrl = pidf_reference_ctrl_[slot];
			pidf_arb_feed_forward = pidf_arb_feed_forward_[slot];
			pidf_arb_feed_forward_units = pidf_arb_feed_forward_units_[slot];
			if (pidf_reference_changed_[slot])
			{
				pidf_reference_changed_[slot] = false;
				return true;
			}
			return false;
		}
		void resetPIDReference(size_t slot)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::resetPIDFReference() : invalid slot " << slot);
				return;
			}
			pidf_reference_changed_[slot] = true;
		}

		void setPIDFReferenceSlot(size_t slot)
		{
			if (slot >= SPARK_MAX_PID_SLOTS)
			{
				ROS_ERROR_STREAM("SparkMaxHWCommand::setPIDFReferenceSlot() : invalid slot " << slot);
				return;
			}
			if (slot != pidf_reference_slot_)
			{
				pidf_reference_slot_ = slot;
				pidf_reference_slot_changed_ = true;
			}
		}
		int getPIDFReferenceSlot(void) const
		{
			return pidf_reference_slot_;
		}

		bool changedPIDFReferenceSlot(size_t &pidf_reference_slot)
		{
			pidf_reference_slot = pidf_reference_slot_;
			if (pidf_reference_slot_changed_)
			{
				pidf_reference_slot_changed_ = false;
				return true;
			}
			return false;
		}
		void resetPIDFReferenceSlot(void)
		{
			pidf_reference_slot_changed_ = true;
		}

		void setForwardLimitSwitchPolarity(LimitSwitchPolarity forward_limit_switch_polarity)
		{
			if (forward_limit_switch_polarity_ != forward_limit_switch_polarity)
			{
				forward_limit_switch_polarity_ = forward_limit_switch_polarity;
				forward_limit_switch_changed_ = true;
			}
		}
		LimitSwitchPolarity getForwardLimitSwitchPolarity(void) const
		{
			return forward_limit_switch_polarity_;
		}

		void setForwardLimitSwitchEnabled(bool forward_limit_switch_enabled)
		{
			if (forward_limit_switch_enabled_ != forward_limit_switch_enabled)
			{
				forward_limit_switch_enabled_ = forward_limit_switch_enabled;
				forward_limit_switch_changed_ = true;
			}
		}
		bool getForwardLimitSwitchEnabled(void) const
		{
			return forward_limit_switch_enabled_;
		}

		bool changedForwardLimitSwitch(
				LimitSwitchPolarity &forward_limit_switch_polarity,
				bool &forward_limit_switch_enabled)
		{
			forward_limit_switch_polarity = forward_limit_switch_polarity_;
			forward_limit_switch_enabled = forward_limit_switch_enabled_;
			if (!forward_limit_switch_changed_)
			{
				forward_limit_switch_changed_ = false;
				return true;
			}
			return false;
		}
		void resetForwardLimitSwitch(void)
		{
			forward_limit_switch_changed_ = true;
		}

		void setReverseLimitSwitchPolarity(LimitSwitchPolarity reverse_limit_switch_polarity)
		{
			if (reverse_limit_switch_polarity_ != reverse_limit_switch_polarity)
			{
				reverse_limit_switch_polarity_ = reverse_limit_switch_polarity;
				reverse_limit_switch_changed_ = true;
			}
		}
		LimitSwitchPolarity getReverseLimitSwitchPolarity(void) const
		{
			return reverse_limit_switch_polarity_;
		}

		void setReverseLimitSwitchEnabled(bool reverse_limit_switch_enabled)
		{
			if (reverse_limit_switch_enabled_ != reverse_limit_switch_enabled)
			{
				reverse_limit_switch_enabled_ = reverse_limit_switch_enabled;
				reverse_limit_switch_changed_ = true;
			}
		}
		bool getReverseLimitSwitchEnabled(void) const
		{
			return reverse_limit_switch_enabled_;
		}

		bool changedReverseLimitSwitch(
				LimitSwitchPolarity &reverse_limit_switch_polarity,
				bool &reverse_limit_switch_enabled)
		{
			reverse_limit_switch_polarity = reverse_limit_switch_polarity_;
			reverse_limit_switch_enabled = reverse_limit_switch_enabled_;
			if (!reverse_limit_switch_changed_)
			{
				reverse_limit_switch_changed_ = false;
				return true;
			}
			return false;
		}
		void resetReverseLimitSwitch(void)
		{
			reverse_limit_switch_changed_ = true;
		}

		void setCurrentLimit(unsigned int current_limit)
		{
			if (current_limit_ != current_limit)
			{
				current_limit_ = current_limit;
				current_limit_one_changed_ = true;
			}
		}
		unsigned int getCurrentLimit(void) const
		{
			return current_limit_;
		}
		bool changedCurrentLimitOne(unsigned int &current_limit)
		{
			current_limit = current_limit_;
			if (current_limit_one_changed_)
			{
				current_limit_one_changed_ = false;
				return true;
			}
			return false;
		}
		void resetCurrentLimitOne(void)
		{
			current_limit_one_changed_ = true;
		}

		void setCurrentLimitStall(unsigned int current_limit_stall)
		{
			if (current_limit_stall_ != current_limit_stall)
			{
				current_limit_stall_ = current_limit_stall;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimitStall(void) const
		{
			return current_limit_stall_;
		}

		void setCurrentLimitFree(unsigned int current_limit_free)
		{
			if (current_limit_free_ != current_limit_free)
			{
				current_limit_free_ = current_limit_free;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimitFree(void) const
		{
			return current_limit_free_;
		}

		void setCurrentLimitRPM(unsigned int current_limit_rpm)
		{
			if (current_limit_rpm_ != current_limit_rpm)
			{
				current_limit_rpm_ = current_limit_rpm;
				current_limit_changed_ = true;
			}
		}
		unsigned int getCurrentLimitRPM(void) const
		{
			return current_limit_rpm_;
		}

		bool changedCurrentLimit(
				unsigned int &current_limit_stall,
				unsigned int &current_limit_free,
				unsigned int &current_limit_rpm)
		{
			current_limit_free = current_limit_free_;
			current_limit_stall = current_limit_stall_;
			current_limit_rpm = current_limit_rpm_;
			if (current_limit_changed_)
			{
				current_limit_changed_ = false;
				return true;
			}
			return false;
		}
		void resetCurrentLimit(void)
		{
			current_limit_changed_ = true;
		}

		void setSecondaryCurrentLimit(double secondary_current_limit)
		{
			if (secondary_current_limit_ != secondary_current_limit)
			{
				secondary_current_limit_ = secondary_current_limit;
				secondary_current_limit_changed_ = true;
			}
		}
		double getSecondaryCurrentLimit(void) const
		{
			return secondary_current_limit_;
		}

		void setSecondaryCurrentLimitCycles(unsigned int secondary_current_limit_cycles)
		{
			if (secondary_current_limit_cycles_ != secondary_current_limit_cycles)
			{
				secondary_current_limit_cycles_ = secondary_current_limit_cycles;
				secondary_current_limit_changed_ = true;
			}
		}
		unsigned int getSecondaryCurrentLimitCycles(void) const
		{
			return secondary_current_limit_cycles_;
		}

		bool changedSecondaryCurrentLimits(double &secondary_current_limit,
				unsigned int &secondary_current_limit_cycles)
		{
			secondary_current_limit = secondary_current_limit_;
			secondary_current_limit_cycles = secondary_current_limit_cycles_;
			if (secondary_current_limit_changed_)
			{
				secondary_current_limit_changed_ = false;
				return true;
			}
			return false;
		}
		void resetSecondaryCurrentLimits(void)
		{
			secondary_current_limit_changed_ = true;
		}

		void setIdleMode(IdleMode idle_mode)
		{
			if (idle_mode_ != idle_mode)
			{
				idle_mode_ = idle_mode;
				idle_mode_changed_ = true;
			}
		}
		IdleMode getIdleMode(void) const
		{
			return idle_mode_;
		}
		bool changedIdleMode(IdleMode &idle_mode)
		{
			idle_mode = idle_mode_;
			if (idle_mode_changed_)
			{
				idle_mode_changed_ = false;
				return true;
			}
			return false;
		}
		void resetIdleMode(void)
		{
			idle_mode_changed_ = true;
		}

		void setVoltageCompensationEnable(bool enable)
		{
			if (voltage_compensation_enable_ != enable)
			{
				voltage_compensation_enable_ = enable;
				voltage_compensation_changed_ = true;
			}
		}
		bool getVoltageCompensationEnable(void)
		{
			return voltage_compensation_enable_;
		}

		void setVoltageCompensationNominalVoltage(double nominal_voltage)
		{
			if (voltage_compensation_nominal_voltage_ != nominal_voltage)
			{
				voltage_compensation_nominal_voltage_ = nominal_voltage;
				voltage_compensation_changed_ = true;
			}
		}
		bool getVoltageCompensationNominalVoltage(void)
		{
			return voltage_compensation_nominal_voltage_;
		}
		bool changedVoltageCompensation(bool &enable,
				double &nominal_voltage)
		{
			enable = voltage_compensation_enable_;
			nominal_voltage = voltage_compensation_nominal_voltage_;
			if (voltage_compensation_changed_)
			{
				voltage_compensation_changed_ = false;
				return true;
			}
			return false;
		}
		void resetVoltageCompensation(void)
		{
			voltage_compensation_changed_ = true;
		}

		void setOpenLoopRampRate(double open_loop_ramp_rate)
		{
			if (open_loop_ramp_rate_ != open_loop_ramp_rate)
			{
				open_loop_ramp_rate_ = open_loop_ramp_rate;
				open_loop_ramp_rate_changed_ = true;
			}
		}
		double getOpenLoopRampRate(void) const
		{
			return open_loop_ramp_rate_;
		}
		bool changedOpenLoopRampRate(double &open_loop_ramp_rate)
		{
			open_loop_ramp_rate = open_loop_ramp_rate_;
			if (open_loop_ramp_rate_changed_)
			{
				open_loop_ramp_rate_changed_ = false;
				return true;
			}
			return false;
		}
		void resetOpenLoopRampRate(void)
		{
			open_loop_ramp_rate_changed_ = true;
		}

		void setClosedLoopRampRate(double closed_loop_ramp_rate)
		{
			if (closed_loop_ramp_rate_ != closed_loop_ramp_rate)
			{
				closed_loop_ramp_rate_ = closed_loop_ramp_rate;
				closed_loop_ramp_rate_changed_ = true;
			}
		}
		double getClosedLoopRampRate(void) const
		{
			return closed_loop_ramp_rate_;
		}
		bool changedClosedLoopRampRate(double &closed_loop_ramp_rate)
		{
			closed_loop_ramp_rate = closed_loop_ramp_rate_;
			if (closed_loop_ramp_rate_changed_)
			{
				closed_loop_ramp_rate_changed_ = false;
				return true;
			}
			return false;
		}
		void resetClosedLoopRampRate(void)
		{
			closed_loop_ramp_rate_changed_ = true;
		}

		void setFollowerType(ExternalFollower follower_type)
		{
			if (follower_type_ != follower_type)
			{
				follower_type_ = follower_type;
				follower_changed_ = true;
			}
		}
		ExternalFollower getFollowerType(void) const
		{
			return follower_type_;
		}

		void setFollowerID(double follower_id)
		{
			if (follower_id_ != follower_id)
			{
				follower_id_ = follower_id;
				follower_changed_ = true;
			}
		}
		double getFollowerID(void) const
		{
			return follower_id_;
		}

		void setFollowerInvert(double follower_invert)
		{
			if (follower_invert_ != follower_invert)
			{
				follower_invert_ = follower_invert;
				follower_changed_ = true;
			}
		}
		double getFollowerInvert(void) const
		{
			return follower_invert_;
		}

		bool changedFollower(ExternalFollower &follower_type,
				int &follower_id,
				bool &follower_invert)
		{
			follower_type = follower_type_;
			follower_id = follower_id_;
			follower_invert = follower_invert_;
			if (follower_changed_)
			{
				follower_changed_ = false;
				return true;
			}
			return false;
		}
		void resetFollower(void)
		{
			follower_changed_ = true;
		}

		void setForwardSoftlimitEnable(bool enable)
		{
			if (forward_softlimit_enable_ != enable)
			{
				forward_softlimit_enable_ = enable;
				forward_softlimit_changed_ = true;
			}
		}
		bool getForwardSoftlimitEnable(void) const
		{
			return forward_softlimit_enable_;
		}
		void setForwardSoftlimit(double limit)
		{
			if (forward_softlimit_ != limit)
			{
				forward_softlimit_ = limit;
				forward_softlimit_changed_ = true;
			}
		}
		double getForwardSoftlimit(void) const
		{
			return forward_softlimit_;
		}
		bool changedForwardSoftlimit(bool &enable, double &limit)
		{
			enable = forward_softlimit_enable_;
			limit  = forward_softlimit_;
			if (forward_softlimit_changed_)
			{
				forward_softlimit_changed_ = false;
				return true;
			}
			return false;
		}
		void resetForwardSoftlimit(void)
		{
			forward_softlimit_changed_ = true;
		}

		void setReverseSoftlimitEnable(bool enable)
		{
			if (reverse_softlimit_enable_ != enable)
			{
				reverse_softlimit_enable_ = enable;
				reverse_softlimit_changed_ = true;
			}
		}
		bool getReverseSoftlimitEnable(void) const
		{
			return reverse_softlimit_enable_;
		}
		void setReverseSoftlimit(double limit)
		{
			if (reverse_softlimit_ != limit)
			{
				reverse_softlimit_ = limit;
				reverse_softlimit_changed_ = true;
			}
		}
		double getReverseSoftlimit(void) const
		{
			return reverse_softlimit_;
		}
		bool changedReverseSoftlimit(bool &enable, double &limit)
		{
			enable = reverse_softlimit_enable_;
			limit  = reverse_softlimit_;
			if (reverse_softlimit_changed_)
			{
				reverse_softlimit_changed_ = false;
				return true;
			}
			return false;
		}
		void resetReverseSoftlimit(void)
		{
			reverse_softlimit_changed_ = true;
		}
		unsigned int getEncoderTicksPerRotation(void) const
		{
			return encoder_ticks_per_rotation_;
		}

		void setEncoderTicksPerRotation(unsigned int encoder_ticks_per_rotation)
		{
			encoder_ticks_per_rotation_ = encoder_ticks_per_rotation;
		}

		void setEncoderType(SensorType encoder_type)
		{
			if (encoder_type_ != encoder_type)
			{
				encoder_type_ = encoder_type;
				encoder_type_changed_ = true;
			}
		}
		SensorType getEncoderType(void) const
		{
			return encoder_type_;
		}

		bool changedEncoderType(SensorType &encoder_type)
		{
			encoder_type = encoder_type_;
			if (encoder_type_changed_)
			{
				encoder_type_changed_ = false;
				return true;
			}
			return false;
		}
		void resetEncoderType(void)
		{
			encoder_type_changed_ = true;
		}

	private:
		double              set_point_;
		bool                set_point_changed_;
		bool                inverted_;
		bool                inverted_changed_;

		// PID Controller
		std::array<double, SPARK_MAX_PID_SLOTS>      p_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      i_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      d_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      f_gain_;
		std::array<double, SPARK_MAX_PID_SLOTS>      i_zone_;
		std::array<double, SPARK_MAX_PID_SLOTS>      d_filter_;
		std::array<bool  , SPARK_MAX_PID_SLOTS>      pidf_constants_changed_;

		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_output_min_;
		std::array<double, SPARK_MAX_PID_SLOTS>      pidf_output_max_;
		std::array<bool  , SPARK_MAX_PID_SLOTS>      pidf_output_range_changed_;

		std::array<double     , SPARK_MAX_PID_SLOTS> pidf_reference_value_;
		std::array<ControlType, SPARK_MAX_PID_SLOTS> pidf_reference_ctrl_;
		std::array<double     , SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_;
		std::array<ArbFFUnits , SPARK_MAX_PID_SLOTS> pidf_arb_feed_forward_units_;
		std::array<bool       , SPARK_MAX_PID_SLOTS> pidf_reference_changed_;

		size_t              pidf_reference_slot_;
		bool                pidf_reference_slot_changed_;

		// Forward and Reverse Limit switches
		LimitSwitchPolarity forward_limit_switch_polarity_;
		bool                forward_limit_switch_enabled_;
		bool                forward_limit_switch_changed_;
		LimitSwitchPolarity reverse_limit_switch_polarity_;
		bool                reverse_limit_switch_enabled_;
		bool                reverse_limit_switch_changed_;

		// Something for current limit mode?
		unsigned int        current_limit_;
		bool                current_limit_one_changed_;
		unsigned int        current_limit_stall_;
		unsigned int        current_limit_free_;
		unsigned int        current_limit_rpm_;
		bool                current_limit_changed_;
		double              secondary_current_limit_;
		unsigned int        secondary_current_limit_cycles_;
		bool                secondary_current_limit_changed_;

		IdleMode            idle_mode_;
		bool                idle_mode_changed_;

		bool                voltage_compensation_enable_;
		double              voltage_compensation_nominal_voltage_;
		bool                voltage_compensation_changed_;

		double              open_loop_ramp_rate_;
		bool                open_loop_ramp_rate_changed_;
		double              closed_loop_ramp_rate_;
		bool                closed_loop_ramp_rate_changed_;

		ExternalFollower    follower_type_;
		int                 follower_id_;
		bool                follower_invert_;
		bool                follower_changed_;

		bool                forward_softlimit_enable_;
		double              forward_softlimit_;
		bool                forward_softlimit_changed_;

		bool                reverse_softlimit_enable_;
		double              reverse_softlimit_;
		bool                reverse_softlimit_changed_;

		unsigned int        encoder_ticks_per_rotation_;
		SensorType          encoder_type_;
		bool                encoder_type_changed_;
};

// Handle - used by each controller to get, by name of the
// corresponding joint, an interface with which to send commands
// to a SparkMax
class SparkMaxCommandHandle: public SparkMaxStateHandle
{
	public:
		SparkMaxCommandHandle(void) :
			SparkMaxStateHandle(),
			cmd_(0)
		{
		}

		SparkMaxCommandHandle(const SparkMaxStateHandle &js, SparkMaxHWCommand *cmd) :
			SparkMaxStateHandle(js),
			cmd_(cmd)
		{
			if (!cmd_)
				throw HardwareInterfaceException("Cannot create SparkMax handle '" + js.getName() + "'. command pointer is null.");
		}

		// Operator which allows access to methods from
		// the SparkMaxHWCommand member var associated with this
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
		SparkMaxHWCommand *operator->()
		{
			assert(cmd_);
			return cmd_;
		}

		// Get a pointer to the HW state associated with
		// this SparkMax.  Since CommandHandle is derived
		// from StateHandle, there's a state embedded
		// in each instance of a CommandHandle. Use
		// this method to access it.
		//
		// handle->state()->getCANID();
		//
		const SparkMaxHWState *state(void) const
		{
			return SparkMaxStateHandle::operator->();
		}

	private:
		SparkMaxHWCommand *cmd_;
};

// Use ClaimResources here since we only want 1 controller
// to be able to access a given SparkMax at any particular time
class SparkMaxCommandInterface : public HardwareResourceManager<SparkMaxCommandHandle, ClaimResources> {};

}
