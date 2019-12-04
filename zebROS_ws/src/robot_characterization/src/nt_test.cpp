#include <ros/ros.h>
#include <networktables/NetworkTableInstance.h>


int main(void)
{
	// Set up a network tables client
	auto ntInst = nt::NetworkTableInstance::GetDefault();
	ntInst.StartClientTeam(900);
	ntInst.SetUpdateRate(0.01); // 100hz
	// Get the entries for the input and output tables
	auto autoSpeedEntry = ntInst.GetEntry("/robot/autospeed");
	auto telemetryEntry = ntInst.GetEntry("/robot/telemetry");

	std::vector<double> telemetry(9);

	ros::Rate r(1);

	unsigned int j = 0;
	while (ros::ok())
	{
		// Example of setting a vector of doubles
		for (size_t i = 0; i < telemetry.size(); i++)
			telemetry[i] = j + .1 * i;
		telemetryEntry.SetDoubleArray(telemetry);

		// Example of reading a double
		ROS_INFO_STREAM("/robot/autospeed is " << autoSpeedEntry.GetDouble(0));

		// Loop once per second
		j += 1;
		r.sleep();
	}

	return 0;
}
