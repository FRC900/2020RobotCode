#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/EjectAction.h"

//include controller service files and other service files
#include "controllers_2020_msgs/IntakeRollerSrv.h"
#include "controllers_2020_msgs/IntakeArmSrv.h"
#include "controllers_2020_msgs/IndexerSrv.h"
#include "sensor_msgs/JointState.h"
#include "std_srvs/Empty.h" //used for the lost a ball client


#include "behaviors/linebreak.h" //contains the Linebreak class, which has logic for processing linebreak signals

#include "behaviors/linebreak.h" //contains the Linebreak class, which has logic for processing linebreak signals

//create the class for the actionlib server
class EjectAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::EjectAction> as_; //create the actionlib server
		std::string action_name_;

		//clients to call controllers
		ros::ServiceClient intake_roller_controller_client_; //create a ros client to send requests to the controller
		ros::ServiceClient intake_arm_controller_client_;
		ros::ServiceClient indexer_controller_client_;

		//client to tell indexer server when we lose a ball
		ros::ServiceClient lost_a_ball_client_;

		//subscribers
		ros::Subscriber joint_states_sub_;

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::EjectGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			double start_time_ = ros::Time::now().toSec();
			bool preempted_ = false;
			bool timed_out_ = false;


			//wait for all controller servers we need
			if(! intake_roller_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find intake_controller's roller ROS service");
				as_.setPreempted();
				return;
			}
			if(! intake_arm_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find intake_controller's arm ROS service");
				as_.setPreempted();
				return;
			}
			if(! indexer_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find indexer_controller");
				as_.setPreempted();
				return;
			}



			//Send intake backwards if we want it to
			if(goal->run_intake_backwards && !preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("Eject server - running intake backwards");

				//define requests to send to intake controller
				controllers_2020_msgs::IntakeRollerSrv roller_srv;
				roller_srv.request.percent_out = intake_percent_out_;

				controllers_2020_msgs::IntakeArmSrv arm_srv;
				arm_srv.request.intake_arm_extend = true;

				//call controller
				if(!intake_roller_controller_client_.call(roller_srv))
				{
					ROS_ERROR("Eject server - controller call to run intake backwards failed");
					preempted_ = true;
				}
				if(!intake_arm_controller_client_.call(arm_srv))
				{
					ROS_ERROR("Eject server - controller call to move intake down failed");
					preempted_ = true;
				}
			}

			//Send indexer backwards if we want it to
			if(goal->run_indexer_backwards && !preempted_ && !timed_out_ && ros::ok())
			{
				ROS_INFO("Eject server - running indexer backwards");

				controllers_2020_msgs::IndexerSrv srv;
				srv.request.indexer_velocity = indexer_velocity_;
				if(!indexer_controller_client_.call(srv))
				{
					ROS_ERROR("Eject server - controller call to move indexer backwards failed");
					preempted_ = true;
				}
			}


			//run a loop to wait for the controller to finish
			ros::Rate r(10);
			indexer_linebreak_.resetPulseDetection();
			while(!preempted_ && !timed_out_ && ros::ok())
			{
				//check if we got a falling edge on the indexer linebreak - indicates we lost a ball
				//note: balls are only counted as "had" if they previously fully passed the indexer linebreak.
				//		It is possible that a ball could have made it up to the indexer linebreak, triggering it,
				//		and then gotten stuck there when the indexer server preempted, before it registered as "had".
				//		Thus, this check could detect losing a ball we never had. To fix this, doing a check for negative
				//		balls in the lost a ball callback (in indexer server), and then just assuming that it's zero.
				//		Also this actionlib server shouldn't really be used, so logic doesn't have to be impeccable for every case.
				if(indexer_linebreak_.falling_edge_happened_)
				{
					ROS_INFO("Eject server - lost a ball");

					//tell the indexer server we lost it
					std_srvs::Empty srv;
					if(!lost_a_ball_client_.call(srv))
					{
						ROS_ERROR("Eject server - call to indexer server's lost a ball service failed");
						preempted_ = true;
					}
					indexer_linebreak_.resetPulseDetection(); //so we don't process this falling edge multiple times
				}

				//check preempted_ - the intended way for this actionlib server to finish is for the driver to preempt it
				if(as_.isPreemptRequested() || !ros::ok()) {
					ROS_ERROR_STREAM(action_name_ << ": preempt while running stuff backwards");
					preempted_ = true;
				}
				//check timed out - TODO might want to use a timeout for this specific controller call rather than the whole server's timeout?
				else if (ros::Time::now().toSec() - start_time_ > server_timeout_) {
					ROS_ERROR_STREAM(action_name_ << ": timed out while running stuff backwards");
					timed_out_ = true;
				}
				//otherwise, pause then loop again
				else {
					r.sleep();
				}
			}


			//Finish -----------------------------------------------

			//set final state using client calls
			//stop the intake rollers. Don't move the intake back up b/c we were trying to clear a jam, that probably won't help
			controllers_2020_msgs::IntakeRollerSrv roller_srv;
			roller_srv.request.percent_out = 0.0;
			if(!intake_roller_controller_client_.call(roller_srv))
			{
				ROS_ERROR("Eject server - controller call to stop intake failed");
			}
			//stop the indexer
			controllers_2020_msgs::IndexerSrv srv;
			srv.request.indexer_velocity = 0.0;
			if(!indexer_controller_client_.call(srv))
			{
				ROS_ERROR("Eject server - controller call to stop indexer failed");
			}


			//log result and set actionlib server state appropriately
			behavior_actions::EjectResult result;

			if(preempted_) {
				ROS_WARN("%s: Finished - Preempted", action_name_.c_str());
				result.timed_out = false;
				result.success = false;
				as_.setPreempted(result);
			}
			else if(timed_out_) {
				ROS_WARN("%s: Finished - Timed Out", action_name_.c_str());
				result.timed_out = true;
				result.success = false;
				as_.setSucceeded(result); //timed out is encoded as succeeded b/c actionlib doesn't have a timed out state
			}
			else { //implies succeeded
				ROS_INFO("%s: Finished - Succeeded", action_name_.c_str());
				result.timed_out = false;
				result.success = true;
				as_.setSucceeded(result);
			}

			return;

		}



	public:
		//Constructor - create actionlib server; the executeCB function will run every time the actionlib server is called
		EjectAction(const std::string &name, ros::NodeHandle eject_params_nh) :
			as_(nh_, name, boost::bind(&EjectAction::executeCB, this, _1), false),
			action_name_(name)
	{
		as_.start(); //start the actionlib server

		//do networking stuff
		std::map<std::string, std::string> service_connection_header;
		service_connection_header["tcp_nodelay"] = "1";

		//initialize clients used to call controllers
		intake_roller_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeRollerSrv>("/frcrobot_jetson/intake_controller/intake_roller_command", false, service_connection_header);
		intake_arm_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeArmSrv>("/frcrobot_jetson/intake_controller/intake_arm_command", false, service_connection_header);
		indexer_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IndexerSrv>("/frcrobot_jetson/indexer_controller/indexer_command", false, service_connection_header);
		if (!eject_params_nh.getParam("server_timeout", server_timeout_)) {
			ROS_ERROR("Could not read server_timeout in eject_server");
			server_timeout_ = 10;
		}
		if (!eject_params_nh.getParam("wait_for_server_timeout", wait_for_server_timeout_)) {
			ROS_ERROR("Could not read wait_for_server_timeout in eject_sever");
			wait_for_server_timeout_ = 10;
		}
		if (!eject_params_nh.getParam("intake_percent_out", intake_percent_out_)) {
			ROS_ERROR("Could not read intake_percent_out in eject_sever");
			intake_percent_out_ = -0.5; //TODO better default
		}
		if (!eject_params_nh.getParam("indexer_velocity", indexer_velocity_)) {
			ROS_ERROR("Could not read indexer_velocity in eject_sever");
			indexer_velocity_ = -4; //TODO better default
		}

	
	}

		~EjectAction(void)
		{
		}

	private:
		//config values
		double server_timeout_;
		double wait_for_server_timeout_;
		double intake_percent_out_;
		double indexer_velocity_;
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "eject_server");
	ros::NodeHandle nh;
	ros::NodeHandle eject_params_nh(nh, "eject_actionlib_params");

	//create the actionlib server
	EjectAction eject_action("eject_server", eject_params_nh);
	
	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
