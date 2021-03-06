#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "actionlib/client/simple_action_client.h"
#include <atomic>
#include <ros/console.h>

//include action files - for this actionlib server and any it sends requests to
#include "behavior_actions/IndexerAction.h"
#include "behavior_actions/IntakeAction.h"

#include "behavior_actions/enumerated_indexer_actions.h"

//include controller service files and other service files
#include "controllers_2020_msgs/IndexerSrv.h"
#include "controllers_2020_msgs/IntakeRollerSrv.h"
#include "std_msgs/UInt8.h" //for subscribing to num balls

#include "behaviors/linebreak.h" //contains a class used for linebreak logic

//create the class for the actionlib server
class IndexerAction {
	protected:
		ros::NodeHandle nh_;

		actionlib::SimpleActionServer<behavior_actions::IndexerAction> as_; //create the actionlib server
		std::string action_name_;

		//clients to call controllers
		ros::ServiceClient indexer_controller_client_; //create a ros client to send requests to the controller
		ros::ServiceClient intake_controller_client_;

		//clients for actionlib servers
		actionlib::SimpleActionClient<behavior_actions::IntakeAction> ac_intake_; //only used for preempting the intake actionlib server

		//subscribers
		ros::Subscriber n_indexer_balls_sub_;

		//linebreak sensors
		Linebreak intake_linebreak_{"intake_linebreak"};
		Linebreak indexer_linebreak_{"indexer_linebreak"}; //just inside the entrance to the indexer
		Linebreak shooter_linebreak_{"shooter_linebreak"}; //just before the shooter

		//variables to store if server was preempted_ or timed out. If either true, skip everything (if statements). If both false, we assume success.
		std::atomic<bool> preempted_;
		bool timed_out_;
		double start_time_;

		//other variables
		std::atomic<int> n_indexer_balls_{3}; //balls anywhere in the indexer, default of 3


		void checkPreemptedAndTimedOut(const std::string &activity)
		{
			if(as_.isPreemptRequested() || !ros::ok())
			{
				preempted_ = true;
				ROS_ERROR_STREAM("indexer_server: preempt during - " << activity);
			}
			else if ((ros::Time::now().toSec() - start_time_) >= server_timeout_)
			{
				timed_out_ = true;
				ROS_ERROR_STREAM("indexer_server: timeout during - " << activity);
			}
		}

		void numIndexerBallsCallback(const std_msgs::UInt8 &msg)
		{
			n_indexer_balls_ = msg.data;
		}


		//function to send balls towards intake until linebreak right inside indexer is triggered - default state if less than 5 balls
		bool goToPositionIntake()
		{
			ROS_INFO("Indexer server - going to position intake");

			if (!indexer_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok()){
				//set velocity to reverse
				controllers_2020_msgs::IndexerSrv srv;
				srv.request.indexer_velocity = -indexer_speed_;
				if ( !indexer_controller_client_.call(srv) )
				{
					ROS_ERROR("Indexer controller failed in indexer server, in goToPositionIntake()");
					preempted_ = true;
				}

				//keep going backwards until indexer linebreak is triggered (until ball right in front of intake)
				ros::Rate r(10); //TODO config?
				const double position_intake_start_time = ros::Time::now().toSec();
				while (!indexer_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok())
				{
					if(ros::Time::now().toSec() - position_intake_start_time > position_intake_timeout_) {
						ROS_ERROR("Indexer server - position intake timed out!");
						timed_out_ = true;
						break;
					}
					checkPreemptedAndTimedOut("going to position intake");
					if(!preempted_ && !timed_out_){
						r.sleep(); //sleep if we didn't preempt or timeout
					}
				}

			}
			//no matter what happened, stop the indexer now
			controllers_2020_msgs::IndexerSrv srv;
			srv.request.indexer_velocity = 0;
			if(!indexer_controller_client_.call(srv)) {
				ROS_ERROR("Indexer server - controller call to stop the indexer failed in goToPositionIntake()");
				preempted_ = true;
			}

			if(preempted_ || timed_out_){
				return false;
			}
			return true;
		}

		//function to send balls towards shooter until linebreak right before shooter is triggered - default state if have 5 balls
		bool goToPositionShoot()
		{
			ROS_INFO("Indexer server - going to position shoot.");

			if (!shooter_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok()){
				//set velocity to forwards
				controllers_2020_msgs::IndexerSrv srv;
				srv.request.indexer_velocity = indexer_speed_; //TODO make sure positive means forward
				if ( !indexer_controller_client_.call(srv) )
				{
					ROS_ERROR("Indexer controller failed in indexer server, in goToPositionShoot()");
					preempted_ = true;
				}

				//keep going forwards until shooter linebreak is triggered (until ball right in front of shooter)
				ros::Rate r(10); //TODO config?
				const double position_shoot_start_time = ros::Time::now().toSec();
				while (!shooter_linebreak_.triggered_ && !preempted_ && !timed_out_ && ros::ok())
				{
					if(ros::Time::now().toSec() - position_shoot_start_time > position_shoot_timeout_) {
						ROS_ERROR("Indexer server - position shoot timed out!");
						timed_out_ = true;
						break;
					}
					checkPreemptedAndTimedOut("going to position shoot");
					if(!preempted_ && !timed_out_){
						r.sleep(); //sleep if we didn't preempt or timeout
					}
				}

				//no matter what happened, stop the indexer now
				srv.request.indexer_velocity = 0;
				if(!indexer_controller_client_.call(srv)) {
					ROS_ERROR("Indexer server - controller call to stop the indexer failed in goToPositionIntake()");
					preempted_ = true;
				}
			}
			if(preempted_ || timed_out_){
				return false;
			}
			return true;
		}


		//define the function to be executed when the actionlib server is called
		void executeCB(const behavior_actions::IndexerGoalConstPtr &goal)
		{
			ROS_INFO("%s: Running callback", action_name_.c_str());

			start_time_ = ros::Time::now().toSec();
			preempted_ = false;
			timed_out_ = false;

			//wait for all controller servers we need
			if(! indexer_controller_client_.waitForExistence(ros::Duration(wait_for_server_timeout_)))
			{
				ROS_ERROR_STREAM(action_name_ << " can't find indexer_controller");
				as_.setPreempted();
				return;
			}


			//Three possible actions we can do:
			//0: go to position intake (back up until trigger linebreak sensor just inside the indexer)
			//1: intake a ball
			//2: feed one ball to the shooter, then stop at position shoot (go forward until trigger linebreak sensor just before shooter)

			switch(goal->action)
			{
				case POSITION_INTAKE: //go to position intake
                {
					ROS_INFO_STREAM("Going to position intake in indexer actionlib server");
					if(!preempted_ && !timed_out_ && ros::ok())
					{
						goToPositionIntake();
					}
				}
					break;
				case INTAKE_ONE_BALL:
                { //braces here fix a cross-initialization error

					ROS_INFO_STREAM("Intaking a ball in indexer actionlib server");


					if(!preempted_ && !timed_out_ && ros::ok())
					{
						//set velocity forward if we're allowed to (e.g. if that won't force a ball into the shooter)
						if(!shooter_linebreak_.triggered_)
						{
							controllers_2020_msgs::IndexerSrv srv;
							srv.request.indexer_velocity = indexer_speed_; //TODO check that positive means forward
							if(!indexer_controller_client_.call(srv))
							{
								preempted_ = true;
								ROS_ERROR("Indexer server - indexer controller call failed when trying to set velocity forward to intake.");
							}
						}
						else {
							//error out because we weren't able to move the indexer forward
							preempted_ = true;
							ROS_ERROR("Indexer server - couldn't start intaking a ball because there was a ball at the shooter");
							//stop the intake
							ROS_WARN("Indexer server - stopping the intake");

							ac_intake_.cancelGoalsAtAndBeforeTime(ros::Time::now());

							controllers_2020_msgs::IntakeRollerSrv srv;
							srv.request.percent_out = 0;
							//srv.request.intake_arm_extend = false;
							if(!intake_controller_client_.call(srv)){
								ROS_ERROR("Indexer server - controller call to stop intake failed");
							}
						}


					}

					//wait for ball to be intaked - ball is intaked if the indexer linebreak gets a rising edge
					indexer_linebreak_.resetPulseDetection(); //clear previous checks for rising/falling edges
					ros::Rate r(10); //TODO config?
					while(!preempted_ && !timed_out_ && ros::ok())
					{
						ROS_INFO("Indexer server - Waiting for ball to be fully intaked");

						if(shooter_linebreak_.triggered_) //then stop intaking
						{
							preempted_ = true; //will break out of the loop
							ROS_ERROR("Indexer server - couldn't finish intaking a ball because there was a ball at the shooter");

							//stop the intake
							ROS_WARN("Indexer server - stopping the intake");

							ac_intake_.cancelGoalsAtAndBeforeTime(ros::Time::now());

							controllers_2020_msgs::IntakeRollerSrv srv;
							srv.request.percent_out = 0;
							if(!intake_controller_client_.call(srv)){
								ROS_ERROR("Indexer server - controller call to stop intake failed");
							}
						}

						if(indexer_linebreak_.rising_edge_happened_)
						{
							ROS_INFO("Indexer server - successfully intaked the ball!");
							break;
						}

						checkPreemptedAndTimedOut("waiting for ball to be fully intaked");
						if(!preempted_ && !timed_out_){
							r.sleep();
						}
					}


				}
					break;
				case SHOOT_ONE_BALL: //feed a ball to the shooter
                {
					ROS_INFO_STREAM("Feeding a ball to the shooter in indexer actionlib server");

					//shoot if you've got the balls - or if not I guess, we removed the num balls check for reliability
					if(!preempted_ && !timed_out_ && ros::ok())
					{
						//set indexer velocity forwards
						controllers_2020_msgs::IndexerSrv srv;
						srv.request.indexer_velocity = indexer_speed_; //TODO check that positive means forwards
						if(!indexer_controller_client_.call(srv)){
							ROS_ERROR("Indexer server - controller call for setting indexer velocity forward for shooting failed");
							preempted_ = true;
						}

						//wait until get a falling edge on the shooter linebreak - means a ball has been shot
						shooter_linebreak_.resetPulseDetection(); //so we can detect a NEW falling edge
						ros::Rate r(10); //TODO config?
						while(!preempted_ && !timed_out_ && ros::ok())
						{

							//check for successful shoot
							if(shooter_linebreak_.falling_edge_happened_)
							{
								ROS_INFO("Indexer server - successfully shot a ball!");
								break;
							}

							checkPreemptedAndTimedOut("moving forwards, waiting for ball to be shot");
							if(!preempted_ && !timed_out_)
							{
								r.sleep();
							}
						}

						//go to position shoot to get ready for the next ball shoot
						if(!preempted_ && !timed_out_ && ros::ok())
						{
							goToPositionShoot(); //won't do anything if we don't have any balls
						}
					}
				}
					break;
				default:
				{
					ROS_ERROR_STREAM("Indexer server: invalid goal. " << (int) goal->action << " is not a valid action (valid ones are 0,1,2)");
					preempted_ = true;
					break;
				}
			}


			//no matter what happened, set the velocity of the indexer to 0 at the end
			controllers_2020_msgs::IndexerSrv srv;
			srv.request.indexer_velocity = 0;
			if(!indexer_controller_client_.call(srv))
			{
				ROS_ERROR("Indexer server - indexer controller call failed when setting final state (for the intake a ball action)");
			}


			//Finish -----------------------------------------------

			//set final state using client calls - if you did preempt handling before, put a check here so don't override that

			//log result and set actionlib server state appropriately
			behavior_actions::IndexerResult result;

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
		IndexerAction(const std::string &name) :
			as_(nh_, name, boost::bind(&IndexerAction::executeCB, this, _1), false),
			action_name_(name),
			ac_intake_("/intake/intake_server", true)
		{
			as_.start(); //start the actionlib server

			//do networking stuff
			std::map<std::string, std::string> service_connection_header;
			service_connection_header["tcp_nodelay"] = "1";

			//initialize clients used to call controllers
			indexer_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IndexerSrv>("/frcrobot_jetson/indexer_controller/indexer_command", false, service_connection_header);
			intake_controller_client_ = nh_.serviceClient<controllers_2020_msgs::IntakeRollerSrv>("/frcrobot_jetson/intake_controller/intake_command", false, service_connection_header);

			//initialize subscribers
			n_indexer_balls_sub_ = nh_.subscribe("/num_indexer_powercells", 1, &IndexerAction::numIndexerBallsCallback, this);
		}

		~IndexerAction(void)
		{
		}

		//config variables
		double server_timeout_; //overall timeout for your server
		double wait_for_server_timeout_; //timeout for waiting for other actionlib servers to become available before exiting this one
		double position_intake_timeout_;
		double position_shoot_timeout_;
		double indexer_speed_;
};

int main(int argc, char** argv) {
	//create node
	ros::init(argc, argv, "indexer_server");
	ros::NodeHandle nh;

	//create the actionlib server
	IndexerAction indexer_action("indexer_server");

	//get config values
	ros::NodeHandle nh_indexer(nh, "actionlib_indexer_params");

	if (! nh_indexer.getParam("server_timeout", indexer_action.server_timeout_) ){
		ROS_ERROR("Couldn't read server_timeout in indexer server");
		indexer_action.server_timeout_ = 10;
	}
	if (! nh_indexer.getParam("wait_for_server_timeout", indexer_action.wait_for_server_timeout_) ){
		ROS_ERROR("Couldn't read wait_for_server_timeout in indexer server");
		indexer_action.wait_for_server_timeout_ = 10;
	}
	if (! nh_indexer.getParam("position_intake_timeout", indexer_action.position_intake_timeout_) ){
		ROS_ERROR("Couldn't read position_intake_timeout in indexer server");
		indexer_action.position_intake_timeout_ = 2;
	}
	if (! nh_indexer.getParam("position_shoot_timeout", indexer_action.position_shoot_timeout_) ){
		ROS_ERROR("Couldn't read position_shoot_timeout in indexer server");
		indexer_action.position_shoot_timeout_ = 2;
	}
	if (! nh_indexer.getParam("indexer_speed", indexer_action.indexer_speed_) ){
		ROS_ERROR("Couldn't read indexer_speed in indexer server");
		indexer_action.wait_for_server_timeout_ = 4; //TODO fix
	}

	ros::AsyncSpinner Spinner(2);
	Spinner.start();
	ros::waitForShutdown();
	return 0;
}
