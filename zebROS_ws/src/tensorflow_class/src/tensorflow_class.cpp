#include <iostream>

enum DepthSampleLocation
{
	UNIFORM,
	CENTER,
	EDGES,
};

class TFObject
{
public:
	TFObject(std::string object_type)
	{
		if (object_type == "power_cell")
		{
			height_ = .178 
			width_ = .178
			depth_ = .178
		}

		else if (object_type == "red_power_port_high_goal" or "blue_power_port_high_goal")
		{
			height_ = 0.76
			width_ = 0.76
			depth_ = 0
		}

		else if (object_type == "red_loading_bay
		

	}

	double getHeight()
	{
		return height_;
	}

	double getWidth()
	{
		return width_;
	}

	double getDepth()
	{
		return depth_;
	}

	DepthSampleLocation getDepthSampleLocation()
	{
		return  depth_sample_location_;
	}

	std::string getObjectType()
	{
		return object_type_;
	}


private:
	double height_;
	double width_:
	double depth_;
	DepthSampleLocation depth_sample_location_;
	std::string object_type_;
};

