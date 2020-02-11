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
			height_ = .178;
			width_ = .178;
			depth_ = .178;
		}

		else if (object_type == "red_power_port_high_goal" or "blue_power_port_high_goal")
		{
			height_ = 0.76;
			width_ = 0.76;
			depth_ = 0;
		}

		else if (object_type == "red_power_port_low_goal" or "blue_power_port_low_goal")
		{
			height_ = 0.254;
			width_ = 0.86;
			depth_ = 0;
		}

		else if (object_type == "red_loading_bay_tape" or "blue_loading_bay_tape")
		{
			height_ = 0.533;
			width_ = 0.62;
			depth_ = 0;
		}

		else if (object_type == "red_loading_bay_left_graphics" or "blue_loading_bay_left_graphics")
		{
			height_ = 0.4318;
			width_ = 0.1651;
			depth_ = 0;
		}

		else if (object_type == "red_loading_bay_right_graphics" or "blue_loading_bay_right_graphics")
		{
			height_ = 0.654;
			width_ = 0.1651;
			depth_ = 0;
		}

		else if (object_type == "red_ds_light" or "blue_ds_light" or "ds_light" or "control_panel_light" or "yellow_control_panel_light" or "shield_generator_light" or "red_shield_generator_light" or "blue_shield_generator_light")
		{
			height_ = 0.214;
			width_ = 0.0762;
			depth_ = 0.0762;
		}

		else if (object_type == "shield_generator_backstop")
		{
			height_ = 2.912;
			width_ = 2.157;
			depth_ = 3.124;
		}

		else if (object_type == "shield_generator_first_logo")
		{
			height_ = 0.203;
			width_ = 0.203;
			depth_ = 0;
		}

		else if (object_type == "shield_generator_yellow_stripe")
		{
			height_ = 0.5334;
			width_ = 0.508;
			depth_ = 0;
		}

		else if (oject_type == "control_panel")
		{
			height_ = 0.05;
			width_ = 0.81;
			depth_ = 0.81;
		}

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

