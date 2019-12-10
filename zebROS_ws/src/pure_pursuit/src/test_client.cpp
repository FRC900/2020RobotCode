#include <ros/ros.h>
#include <pure_pursuit/PurePursuitAction.h>
#include <pure_pursuit/PurePursuitGoal.h>
#include <pure_pursuit/PurePursuitResult.h>
#include <actionlib/client/simple_action_client.h>
#include "pure_pursuit/spline.h"
#include <std_srvs/Trigger.h>
#include "base_trajectory/GenerateSpline.h"
#include <swerve_point_generator/FullGenCoefs.h>
#include <pure_pursuit/Point.h>
#include "tf2/LinearMath/Quaternion.h"

std::shared_ptr<actionlib::SimpleActionClient<pure_pursuit::PurePursuitAction>> ac;
ros::ServiceClient spline_gen;
double defined_dt = 0.02;
double iteration_length = 0.3;

struct spline_coefs
{
	double a;
	double b;
	double c;
	double d;
	double e;
	double f;
	spline_coefs(void):
		a(0),
		b(0),
		c(0),
		d(0),
		e(0),
		f(0)
	{
	}
	spline_coefs(const double &aa, const double &bb, const double &cc, const double &dd, const double &ee, const double &ff) :
		a(aa),
		b(bb),
		c(cc),
		d(dd),
		e(ee),
		f(ff)
	{
	}

	const spline_coefs first_derivative(void) const
	{
		return spline_coefs(0, 5 * a, 4 * b, 3 * c, 2 * d, 1 * e);
	}

	void print(std::ostream &os) const
	{
		os << "a:" << a << " b:" << b << " c:" << c << " d:" << d << " e:" << e << " f:" << f;
	}
};

void calc_point(const spline_coefs &spline, double t, double &returner)
{
	const double t_squared = t * t;
	const double t_cubed   = t_squared * t;
	const double t_fourth  = t_squared * t_squared;
	const double t_fifth   = t_cubed * t_squared;
	returner = spline.a * t_fifth + spline.b * t_fourth + spline.c * t_cubed + spline.d * t_squared + spline.e * t + spline.f;
	//if (t)
	//ROS_INFO_STREAM("calc_point spline:" << spline << " t:" << t << " t_squared:" << t_squared << " t_cubed:" << t_cubed << " t_fourth:" << t_fourth << " t_fifth:" << t_fifth << " f(t):" << returner);
}

tk::spline parametrize_spline(const std::vector<spline_coefs> &x_splines_first_deriv,
		const std::vector<spline_coefs> &y_splines_first_deriv, 
		const std::vector<spline_coefs> &x_splines, const std::vector<spline_coefs> &y_splines, const std::vector<spline_coefs> &orient_splines,
		const std::vector<double> &end_points, double &total_arc_length,
		std::vector<double> &dtds_by_spline,
		std::vector<double> &arc_length_by_spline,
		nav_msgs::Path &path)
{
	total_arc_length = 0;
	double spline_points = 1000.;
	double period_t = (end_points[0] - 0.0) / spline_points;
	double start = 0;
	double arc_before = 0;
	double a_val = 0;
	double b_val = 0;
	int seq = 0;
	std::vector<double> t_vals;
	std::vector<double> s_vals;
	t_vals.reserve(x_splines_first_deriv.size() * (static_cast<size_t>(spline_points) + 1));
	s_vals.reserve(x_splines_first_deriv.size() * (static_cast<size_t>(spline_points) + 1));

	double last_waypoint_position = 0;

	for (size_t i = 0; i < x_splines_first_deriv.size(); i++)
	{
		if (i != 0)
		{
			ROS_INFO_STREAM("parametrizing spline " << i+1);
			period_t = (end_points[i] - end_points[i - 1]) / spline_points;
			//start = end_points[i - 1];
		}
		else
		{
			ROS_INFO_STREAM("parametrizing first spline");
		}
		if (i > 1)
		{
			dtds_by_spline.push_back((end_points[i - 1] - end_points[i - 2]) /  (total_arc_length
									 - arc_before));
	ROS_INFO_STREAM("dtds by spline:" << dtds_by_spline[i]);
		}
		else if (i == 1)
		{
			dtds_by_spline.push_back((end_points[0] - 0) /  (total_arc_length - arc_before));
	ROS_INFO_STREAM("dtds by spline:" << dtds_by_spline[i]);
		}

		arc_before = total_arc_length;
		ROS_INFO_STREAM("arc_before: " << arc_before);
		for (size_t k = 0; k < static_cast<size_t>(spline_points); k++)
		{
			a_val = k * period_t + start;
			b_val = (k + 1) * period_t + start;
			//ROS_INFO_STREAM("a_val = " << a_val << " b_val = " << b_val << " total arc = " << total_arc_length);
			t_vals.push_back(a_val);
			s_vals.push_back(total_arc_length);
			//TODO: improve efficiency here
			double x_at_a;
			double x_at_b;
			double y_at_a;
			double y_at_b;
			double x_at_avg;
			double y_at_avg;
			calc_point(x_splines_first_deriv[i], a_val, x_at_a);
			calc_point(x_splines_first_deriv[i], b_val, x_at_b);
			calc_point(y_splines_first_deriv[i], a_val, y_at_a);
			calc_point(y_splines_first_deriv[i], b_val, y_at_b);
			calc_point(x_splines_first_deriv[i], (a_val + b_val) / 2, x_at_avg);
			calc_point(y_splines_first_deriv[i], (a_val + b_val) / 2, y_at_avg);

			//delete me, debug only

			//Simpsons rule
			total_arc_length += period_t / 6. * (hypot(x_at_a, y_at_a) + 4. * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));

			if(total_arc_length - last_waypoint_position > iteration_length)
			{
				last_waypoint_position = total_arc_length;
				geometry_msgs::PoseStamped pose;
				double current_time = a_val; //meaningless

				double x_position;
				double y_position;
				double yaw;
				calc_point(x_splines[i], current_time, x_position);
				calc_point(y_splines[i], current_time, y_position);
				calc_point(orient_splines[i], current_time, yaw);

				geometry_msgs::Quaternion orientation;
				tf2::Quaternion tf_orientation;
				tf_orientation.setRPY(0, 0, yaw);
				orientation.x = tf_orientation.getX();
				orientation.y = tf_orientation.getY();
				orientation.z = tf_orientation.getZ();
				orientation.w = tf_orientation.getW();

				pose.header.seq = seq;
				pose.header.stamp = ros::Time(current_time);
				pose.header.frame_id = "initial_pose";
				pose.pose.position.x = x_position;
				pose.pose.position.y = y_position;
				pose.pose.orientation = orientation;

				path.poses.push_back(pose);

				seq++;
			}
		}
		arc_length_by_spline.push_back(total_arc_length);
	}
	if (x_splines_first_deriv.size() == 1)
	{
		dtds_by_spline.push_back((end_points[x_splines_first_deriv.size() - 1] - 0)
								 /  (total_arc_length - arc_before));
		ROS_INFO_STREAM("dtds by spline:" << dtds_by_spline.back());
	}
	else
	{
		dtds_by_spline.push_back((end_points[x_splines_first_deriv.size() - 1]
								  - end_points[x_splines_first_deriv.size() - 2]) /  (total_arc_length - arc_before));
		ROS_INFO_STREAM("dtds by spline:" << dtds_by_spline.back());
	}

	//Put in the last values
	t_vals.push_back(b_val);
	s_vals.push_back(total_arc_length);

	//Spline fit of t in terms of s (we input a t -> s)
	tk::spline s;

	s.set_points(s_vals, t_vals);
	for (size_t i = 0; i < t_vals.size(); i++)
	{
		ROS_INFO_STREAM("t_val = " << t_vals[i] << " s vals = " << s_vals[i]);
	}
	ROS_INFO_STREAM("successful parametrize spline");
	return s;
}

bool trigger_pathing_cb(pure_pursuit::Point::Request &req, pure_pursuit::Point::Response &res)
{
	/*** GET SPLINE COEFFICIENTS**/
	ROS_ERROR_STREAM("Getting spline coefficients in pure pursuit test client");
	base_trajectory::GenerateSpline srvBaseTrajectory;
	int point_num = req.points.size();
	srvBaseTrajectory.request.points.resize(point_num);
	ROS_INFO_STREAM(req.points[0].x << " " << req.points[0].y << " " << req.points[0].z);

	for(int i = 0; i<point_num; i++)
	{
		//x-movement
		srvBaseTrajectory.request.points[i].positions.push_back(req.points[i].x);
		//y-movement
		srvBaseTrajectory.request.points[i].positions.push_back(req.points[i].y);
		//z-rotation
		srvBaseTrajectory.request.points[i].positions.push_back(req.points[i].z);
		//time for profile to run
		srvBaseTrajectory.request.points[i].time_from_start = ros::Duration(10*(i+1));
	}

	if(!spline_gen.call(srvBaseTrajectory))
	{
		ROS_ERROR_STREAM("spline_gen died");
	}

	swerve_point_generator::FullGenCoefs traj;
	traj.request.orient_coefs.resize(point_num);
	traj.request.x_coefs.resize(point_num);
	traj.request.y_coefs.resize(point_num);
	traj.request.end_points.resize(point_num);
	traj.request.spline_groups.push_back(point_num);

	ROS_INFO_STREAM("srv base trajectory size = " << srvBaseTrajectory.response.orient_coefs.size());

	for(int p = 0; p < point_num; p++)
	{
		for(size_t i = 0; i < srvBaseTrajectory.response.orient_coefs[p].spline.size(); i++)
		{
			ROS_INFO_STREAM("p = " << p << " i = " << i);
			traj.request.orient_coefs[p].spline.push_back(srvBaseTrajectory.response.orient_coefs[p].spline[i]);
			ROS_INFO_STREAM("p = " << p << " i = " << i);
			traj.request.x_coefs[p].spline.push_back(srvBaseTrajectory.response.x_coefs[p].spline[i]);
			ROS_INFO_STREAM("p = " << p << " i = " << i);
			traj.request.y_coefs[p].spline.push_back(srvBaseTrajectory.response.y_coefs[p].spline[i]);
			ROS_INFO_STREAM("p = " << p << " i = " << i);
		}

		traj.request.wait_before_group.push_back(p*0.16);
		traj.request.t_shift.push_back(0);
		traj.request.flip.push_back(false);
		traj.request.end_points[p] = srvBaseTrajectory.response.end_points[p+1];
		traj.request.initial_v = 0;
		traj.request.final_v = 0;
		traj.request.x_invert.push_back(0);
	}

	nav_msgs::Path path;

		std::vector<spline_coefs> x_splines;
		std::vector<spline_coefs> y_splines;
		std::vector<spline_coefs> orient_splines;
		std::vector<spline_coefs> x_splines_first_deriv;
		std::vector<spline_coefs> y_splines_first_deriv;
		std::vector<spline_coefs> orient_splines_first_deriv;
		std::vector<spline_coefs> x_splines_second_deriv;
		std::vector<spline_coefs> y_splines_second_deriv;
		std::vector<spline_coefs> orient_splines_second_deriv;

		const int neg_x = traj.request.x_invert[0] ? -1 : 1;
		std::vector<double> end_points_holder;

		// Make the splines from traj into spline_coefs format. TODO remove traj middleman
	/*** ITERATE THROUGH SPLINES ***/
	ROS_ERROR_STREAM("Iterating through splines in pure pursuit test client");
		for (int i = 0; i < traj.request.spline_groups[0]; i++)
		{
			ROS_INFO_STREAM("orient_coefs[" << i << "].spline=" << traj.request.orient_coefs[i].spline[0] << " " <<
							traj.request.orient_coefs[i].spline[1] << " " <<
							traj.request.orient_coefs[i].spline[2] << " " <<
							traj.request.orient_coefs[i].spline[3] << " " <<
							traj.request.orient_coefs[i].spline[4] << " " <<
							traj.request.orient_coefs[i].spline[5]);

			orient_splines.push_back(spline_coefs(
										 traj.request.orient_coefs[i].spline[0] * neg_x,
										 traj.request.orient_coefs[i].spline[1] * neg_x,
										 traj.request.orient_coefs[i].spline[2] * neg_x,
										 traj.request.orient_coefs[i].spline[3] * neg_x,
										 traj.request.orient_coefs[i].spline[4] * neg_x,
										 traj.request.orient_coefs[i].spline[5] * neg_x));
			//ROS_INFO_STREAM("orient_coefs[" << i << "].spline=" << orient_splines.back());

			ROS_INFO_STREAM("x_coefs[" << i << "].spline=" << traj.request.x_coefs[i].spline[0] << " " <<
							traj.request.x_coefs[i].spline[1] << " " <<
							traj.request.x_coefs[i].spline[2] << " " <<
							traj.request.x_coefs[i].spline[3] << " " <<
							traj.request.x_coefs[i].spline[4] << " " <<
							traj.request.x_coefs[i].spline[5]);

			x_splines.push_back(spline_coefs(
									traj.request.x_coefs[i].spline[0] * neg_x,
									traj.request.x_coefs[i].spline[1] * neg_x,
									traj.request.x_coefs[i].spline[2] * neg_x,
									traj.request.x_coefs[i].spline[3] * neg_x,
									traj.request.x_coefs[i].spline[4] * neg_x,
									traj.request.x_coefs[i].spline[5] * neg_x));
			//ROS_INFO_STREAM("x_coefs[" << i << "].spline=" << x_splines.back());

			ROS_INFO_STREAM("y_coefs[" << i << "].spline=" << traj.request.y_coefs[i].spline[0] << " " <<
							traj.request.y_coefs[i].spline[1] << " " <<
							traj.request.y_coefs[i].spline[2] << " " <<
							traj.request.y_coefs[i].spline[3] << " " <<
							traj.request.y_coefs[i].spline[4] << " " <<
							traj.request.y_coefs[i].spline[5]);

			y_splines.push_back(spline_coefs(
									traj.request.y_coefs[i].spline[0],
									traj.request.y_coefs[i].spline[1],
									traj.request.y_coefs[i].spline[2],
									traj.request.y_coefs[i].spline[3],
									traj.request.y_coefs[i].spline[4],
									traj.request.y_coefs[i].spline[5]));
			//ROS_INFO_STREAM("y_coefs[" << i << "].spline=" << y_splines.back());

			//Take derivatives of splines
			//ROS_INFO_STREAM("x splines[" << i << "] " << x_splines[i]);
			x_splines_first_deriv.push_back(x_splines[i].first_derivative());
			//ROS_INFO_STREAM("x splines[" << i << "] first deriv " << x_splines_first_deriv[i]);

			x_splines_second_deriv.push_back(x_splines_first_deriv[i].first_derivative());
			//ROS_INFO_STREAM("x splines[" << i << "] second deriv " << x_splines_second_deriv[i]);

			//ROS_INFO_STREAM("y splines[" << i << "] : " << y_splines[i]);
			y_splines_first_deriv.push_back(y_splines[i].first_derivative());
			//ROS_INFO_STREAM("y splines[" << i << "] first deriv " << y_splines_first_deriv[i]);

			y_splines_second_deriv.push_back(y_splines_first_deriv[i].first_derivative());
			//ROS_INFO_STREAM("y splines[" << i << "] second deriv " << y_splines_second_deriv[i]);

			//ROS_INFO_STREAM("orient splines[" << i << "] : " << orient_splines[i]);
			orient_splines_first_deriv.push_back(orient_splines[i].first_derivative());
			//ROS_INFO_STREAM("orient splines[" << i << "] first deriv " << orient_splines_first_deriv[i]);

			orient_splines_second_deriv.push_back(orient_splines_first_deriv[i].first_derivative());
			//ROS_INFO_STREAM("orient splines[" << i << "] second deriv " << orient_splines_second_deriv[i]);

			ROS_INFO_STREAM("hrer: " << traj.request.end_points[i] << " r_s: " <<  traj.request.spline_groups[0] );
			end_points_holder.push_back(traj.request.end_points[i] );
		}

		const double t_shift = traj.request.t_shift[0];
		const bool flip_dirc = traj.request.flip[0];

		/*** PARAMETRIZE SPLINE ***/
		ROS_ERROR_STREAM("parametrizing spline in pure pursuit test client");
		std::vector<double> dtds_for_spline;
		std::vector<double> arc_length_for_spline;
		double total_arc;
		//Run spline parametrizing code - also gets dtds and arc lengths
		//This turns the x,y linear positions into a single linear-path-position vs. time curve
		tk::spline spline = parametrize_spline(x_splines_first_deriv, y_splines_first_deriv, x_splines, y_splines, orient_splines, end_points_holder,
				total_arc, dtds_for_spline, arc_length_for_spline, path);
		for(int i = 0; i < arc_length_for_spline.size(); i++)
		{
			ROS_INFO_STREAM("arc length for spline " << i << " is " << arc_length_for_spline[i]);
		}

		/*** TRANSFER INTO nav_msgs::Path FORMAT***/
		/*ROS_ERROR_STREAM("transfering into path format in pure pursuit test client");
		double seq = 0;
		int spline_num = 0;
		for(double current_spline_position = 0; current_spline_position < total_arc; current_spline_position += iteration_length)
		{
			if(current_spline_position == 0)
			{
				spline_num = 0;
			}
			else
			{
				for(int i = 0; i < x_splines_first_deriv.size(); i++)
				{
					if(current_spline_position > arc_length_for_spline[i])
					{
						spline_num = i;
						break;
					}
				}
			}
			ROS_INFO_STREAM("spline_num");
			geometry_msgs::PoseStamped pose;
			double current_time = spline(current_spline_position);

			double x_position;
			double y_position;
			double yaw;
			calc_point(x_splines[spline_num], current_time, x_position);
			calc_point(y_splines[spline_num], current_time, y_position);
			calc_point(orient_splines[spline_num], current_time, yaw);

			geometry_msgs::Quaternion orientation;
			tf2::Quaternion tf_orientation;
			tf_orientation.setRPY(0, 0, yaw);
			orientation.x = tf_orientation.getX();
			orientation.y = tf_orientation.getY();
			orientation.z = tf_orientation.getZ();
			orientation.w = tf_orientation.getW();

			pose.header.seq = seq;
			pose.header.stamp = ros::Time(current_time);
			pose.header.frame_id = "initial_pose";
			pose.pose.position.x = x_position;
			pose.pose.position.y = y_position;
			pose.pose.orientation = orientation;

			path.poses.push_back(pose);

			seq++;
		}*/
		ROS_INFO_STREAM("number of points in path = " << path.poses.size());

	/*** SEND GOAL TO ACTION SERVER ***/
	ROS_ERROR_STREAM("Sending goal to action server in pure pursuit test client");
    pure_pursuit::PurePursuitGoal goal;
    goal.path = path;
    ac->sendGoal(goal);

    bool finished_before_timeout = ac->waitForResult(ros::Duration(15));

    if(finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac->getState();
        ROS_INFO_STREAM("state = " << state.toString());
    }
    else
        ROS_INFO_STREAM("timed out in test_client");

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_client");
    ros::NodeHandle n;

	std::map<std::string, std::string> service_connection_header;
	service_connection_header["tcp_nodelay"] = 1;
    ac = std::make_shared<actionlib::SimpleActionClient<pure_pursuit::PurePursuitAction>>("pure_pursuit_server", true);
    ros::ServiceServer trigger_pathing = n.advertiseService("trigger_pathing", &trigger_pathing_cb);
	spline_gen = n.serviceClient<base_trajectory::GenerateSpline>("/path_to_goal/base_trajectory/spline_gen", false, service_connection_header);

    ac->waitForServer();

    ros::spin();

    return 0;
}
