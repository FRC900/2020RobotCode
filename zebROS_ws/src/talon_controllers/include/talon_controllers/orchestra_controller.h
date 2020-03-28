#ifndef ORCHESTRA_CONTROLLER
#define ORCHESTRA_CONTROLLER

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <talon_interface/orchestra_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <talon_controller_msgs/LoadInstrumentsSrv.h>
#include <talon_controller_msgs/LoadMusicSrv.h>
#include <talon_controller_msgs/SetOrchestraStateSrv.h>

namespace orchestra_controller
{
	class OrchestraController: public controller_interface::Controller<hardware_interface::OrchestraCommandInterface>
	{
		public:
			OrchestraController() {}

			virtual bool init(hardware_interface::OrchestraCommandInterface *hw,
					ros::NodeHandle						&root_nh,
					ros::NodeHandle						&controller_nh);
			virtual void starting(const ros::Time &time);
			virtual void update(const ros::Time &time, const ros::Duration & );
			virtual void stopping(const ros::Time &time);

			bool loadMusicService(talon_controller_msgs::LoadMusicSrv::Request &req,
					talon_controller_msgs::LoadMusicSrv::Response &res);

			bool setStateService(talon_controller_msgs::SetOrchestraStateSrv::Request &req,
					talon_controller_msgs::SetOrchestraStateSrv::Response &res);

			bool reloadInstrumentsService(talon_controller_msgs::LoadInstrumentsSrv::Request &req,
					talon_controller_msgs::LoadInstrumentsSrv::Response &res);

		private:
			hardware_interface::OrchestraCommandHandle orchestra_command_handle_;
			realtime_tools::RealtimeBuffer<std::string> music_file_path_;
			realtime_tools::RealtimeBuffer<bool> music_file_path_changed_;
			realtime_tools::RealtimeBuffer<std::vector<std::string>> instruments_;
			realtime_tools::RealtimeBuffer<bool> instruments_changed_;
			realtime_tools::RealtimeBuffer<int> state_;
			realtime_tools::RealtimeBuffer<bool> state_changed_;

			ros::ServiceServer load_music_server_;
			ros::ServiceServer set_state_server_;
			ros::ServiceServer load_instruments_server_;
	}; //class

} //namespace

#endif
