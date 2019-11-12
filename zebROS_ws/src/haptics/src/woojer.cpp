#include "ros/ros.h"
#include "std_msgs/String.h"
#include "frc_msgs/MatchSpecificData.h"
#include "sensor_msgs/JointState.h"
double match_time;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void chatterCallback(const frc_msgs::MatchSpecificData::ConstPtr& msg)
{
match_time = msg->matchTimeRemaining;
}

void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
{

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("match_time", 1000, chatterCallback);
  ros::Subscriber joint_states = n.subscribe("joint_states", 1000, jointStatesCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  while(ros::ok())
  {
	  ros::spinOnce();
	if (match_time > 30 && match_time < 31)
	{
	}
  }

  return 0;
}
