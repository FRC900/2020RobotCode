#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <state_handle/state_handle.h>

namespace hardware_interface
{

//holds data from the PDP
class PDPHWState
{
	public:
		PDPHWState(void) :
			voltage_(0.0),
			temperature_(0.0),
			total_current_(0.0),
			total_power_(0.0),
			total_energy_(0.0),
			current_ {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 
			things_plugged_in_{"", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""}
		{}

		//access and set
		double getVoltage(void) const		{return voltage_;}
		double getTemperature(void) const	{return temperature_;}
		double getTotalCurrent(void) const	{return total_current_;}
		double getTotalPower(void) const	{return total_power_;}
		double getTotalEnergy(void) const	{return total_energy_;}
		double getCurrent(int channel) const {
			if(channel >= 0 && channel <= 15)
				return current_[channel];
			else
			{
				ROS_WARN_STREAM("Invalid channel. Cannot read current.");
				return 0.0;
			}
		}
		std::string getThingsPluggedIn(int channel) const{
			if(channel >= 0 && channel <= 15)
				return things_plugged_in_[channel];
			else
			{
				ROS_WARN_STREAM("Invalid channel. Cannot read thing_plugged_in.");
				return "";
			}
		}

		void setVoltage(double voltage)				{voltage_ = voltage;}
		void setTemperature(double temperature)		{temperature_ = temperature;}
		void setTotalCurrent(double total_current)	{total_current_ = total_current;}
		void setTotalPower(double total_power)		{total_power_ = total_power;}
		void setTotalEnergy(double total_energy)	{total_energy_ = total_energy;}
		void setCurrent(double current, int channel) {
			if(channel >= 0 && channel <= 15)
				current_[channel] = current;
			else
				ROS_WARN_STREAM("Invalid channel. Cannot set current.");
		}
		
		void setThingsPluggedIn(std::string thing, int channel){
			if(channel >= 0 && channel <=15)
				things_plugged_in_[channel] = thing;
			else
				ROS_WARN_STREAM("Invalid channel. Cannot set thing_plugged_in.");
		}

	private:
		double voltage_;
		double temperature_;
		double total_current_;
		double total_power_;
		double total_energy_;
		double current_[16];
		std::string things_plugged_in_[16];
};

typedef StateHandle<const PDPHWState> PDPStateHandle;
typedef StateHandle<PDPHWState> PDPWritableStateHandle;

class PDPStateInterface: public HardwareResourceManager<PDPStateHandle> {};
}
