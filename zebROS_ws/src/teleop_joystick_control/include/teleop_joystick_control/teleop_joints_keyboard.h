#include <ros/ros.h>
#include <senson_msgs/Joy.h>

class TeleopJointsKeyboard
{
	public:
		TeleopJointsKeyboard(ros::NodeHandle &nh);
		~TeleopJointsKeyboard();
		void keyboardLoop();
		int pollKeyboard(int kfd, char &c) const;

	private:
		ros::Publisher joints_pub_;
		sensor_msgs::Joy cmd_;
		//bool has_recieved_joints_;
};
