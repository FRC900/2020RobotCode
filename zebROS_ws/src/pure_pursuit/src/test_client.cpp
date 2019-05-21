#include <ros/ros.h>
#include <pure_pursuit/PurePursuitAction.h>
#include <pure_pursuit/PurePursuitGoal.h>
#include <pure_pursuit/PurePursuitResult.h>
#include <actionlib/client/simple_action_client.h>
#include "behaviors/enumerated_elevator_indices.h"
#include "pure_pursuit/spline.h"
#include <std_srvs/Trigger.h>
#include "base_trajectory/GenerateSpline.h"
#include <swerve_point_generator/FullGenCoefs.h>
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
		const std::vector<double> &end_points, double &total_arc_length,
		std::vector<double> &dtds_by_spline,
		std::vector<double> &arc_length_by_spline)
{
	ROS_INFO_STREAM("line = " << __LINE__);
	ROS_INFO_STREAM("endpoints size = " << end_points.size());
	total_arc_length = 0;
	double spline_points = 1000.;
	double period_t = (end_points[0] - 0.0) / spline_points;
	double start = 0;
	double arc_before = 0;
	double b_val = 0;
	std::vector<double> t_vals;
	std::vector<double> s_vals;
	t_vals.reserve(x_splines_first_deriv.size() * (static_cast<size_t>(spline_points) + 1));
	s_vals.reserve(x_splines_first_deriv.size() * (static_cast<size_t>(spline_points) + 1));
	ROS_INFO_STREAM("line = " << __LINE__);

	for (size_t i = 0; i < x_splines_first_deriv.size(); i++)
	{
		ROS_INFO_STREAM("line = " << __LINE__);
		if (i != 0)
		{
			ROS_INFO_STREAM("line = " << __LINE__);
			period_t = (end_points[i] - end_points[i - 1]) / spline_points;
			start = end_points[i - 1];
		}
		if (i > 1)
		{
			ROS_INFO_STREAM("line = " << __LINE__);
			dtds_by_spline.push_back((end_points[i - 1] - end_points[i - 2]) /  (total_arc_length
									 - arc_before));
		ROS_INFO_STREAM("dtds by spline:" << dtds_by_spline[i]);
		}
		else if (i == 1)
		{
			dtds_by_spline.push_back((end_points[0] - 0) /  (total_arc_length - arc_before));
		ROS_INFO_STREAM("dtds by spline:" << dtds_by_spline[i]);
		}
		ROS_INFO_STREAM("line = " << __LINE__);
		arc_before = total_arc_length;
		ROS_INFO_STREAM("arc_before: " << arc_before);
		for (size_t k = 0; k < static_cast<size_t>(spline_points); k++)
		{
			const double a_val = k * period_t + start;
			b_val = (k + 1) * period_t + start;
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

			//f(t) = sqrt((dx/dt)^2 + (dy/dt)^2)

			//ROS_INFO_STREAM("period_t: " << period_t);
			//ROS_INFO_STREAM("idek: " << hypot(x_at_a, y_at_a) + 4 * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));
			total_arc_length += period_t / 6. * (hypot(x_at_a, y_at_a) + 4. * hypot(x_at_avg, y_at_avg) + hypot(x_at_b, y_at_b));
			//ROS_INFO_STREAM("arc_now: " << total_arc_length);
			//Simpsons rule
			//ROS_INFO_STREAM("Spline: " << i << " t_val: " << a_val <<"  arc_length: " << total_arc_length);
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
		//ROS_INFO_STREAM("s_vale = " << s_vals[i] << " s vals = " << s(s_vals[i]));
	}
	ROS_INFO_STREAM("successful parametrize spline");
	return s;
}

bool trigger_pathing_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
	/*** GET SPLINE COEFFICIENTS**/
	ROS_ERROR_STREAM("Getting spline coefficients in pure pursuit test client");
	base_trajectory::GenerateSpline srvBaseTrajectory;
	srvBaseTrajectory.request.points.resize(2);

	//x-movement
	srvBaseTrajectory.request.points[0].positions.push_back(1);
	srvBaseTrajectory.request.points[0].velocities.push_back(0);
	srvBaseTrajectory.request.points[0].accelerations.push_back(0);
	//y-movement
	srvBaseTrajectory.request.points[0].positions.push_back(2);
	srvBaseTrajectory.request.points[0].velocities.push_back(0);
	srvBaseTrajectory.request.points[0].accelerations.push_back(0);
	//z-rotation
	double rotation = 0;
	srvBaseTrajectory.request.points[0].positions.push_back(rotation);
	srvBaseTrajectory.request.points[0].velocities.push_back(0); //velocity at the end point
	srvBaseTrajectory.request.points[0].accelerations.push_back(0); //acceleration at the end point
	//time for profile to run
	srvBaseTrajectory.request.points[0].time_from_start = ros::Duration(10);

	//x-movement
	srvBaseTrajectory.request.points[1].positions.push_back(0);
	srvBaseTrajectory.request.points[1].velocities.push_back(0);
	srvBaseTrajectory.request.points[1].accelerations.push_back(0);
	//y-movement
	srvBaseTrajectory.request.points[1].positions.push_back(4);
	srvBaseTrajectory.request.points[1].velocities.push_back(0);
	srvBaseTrajectory.request.points[1].accelerations.push_back(0);
	//z-rotation
	srvBaseTrajectory.request.points[1].positions.push_back(rotation);
	srvBaseTrajectory.request.points[1].velocities.push_back(0); //velocity at the end point
	srvBaseTrajectory.request.points[1].accelerations.push_back(0); //acceleration at the end point
	//time for profile to run
	srvBaseTrajectory.request.points[1].time_from_start = ros::Duration(20);

	if(!spline_gen.call(srvBaseTrajectory))
	{
		ROS_ERROR_STREAM("spline_gen died");
	}

	swerve_point_generator::FullGenCoefs traj;
	traj.request.orient_coefs.resize(2);
	traj.request.x_coefs.resize(2);
	traj.request.y_coefs.resize(2);
	traj.request.end_points.resize(2);

	for(size_t i = 0; i < srvBaseTrajectory.response.orient_coefs[0].spline.size(); i++)
	{
		traj.request.orient_coefs[0].spline.push_back(srvBaseTrajectory.response.orient_coefs[1].spline[i]);
		traj.request.x_coefs[0].spline.push_back(srvBaseTrajectory.response.x_coefs[1].spline[i]);
		traj.request.y_coefs[0].spline.push_back(srvBaseTrajectory.response.y_coefs[1].spline[i]);
	}
	for(size_t i = 0; i < srvBaseTrajectory.response.orient_coefs[1].spline.size(); i++)
	{
		traj.request.orient_coefs[0].spline.push_back(srvBaseTrajectory.response.orient_coefs[2].spline[i]);
		traj.request.x_coefs[0].spline.push_back(srvBaseTrajectory.response.x_coefs[2].spline[i]);
		traj.request.y_coefs[0].spline.push_back(srvBaseTrajectory.response.y_coefs[2].spline[i]);
	}

	traj.request.spline_groups.push_back(1);
	traj.request.wait_before_group.push_back(.16);
	traj.request.t_shift.push_back(0);
	traj.request.flip.push_back(false);
	traj.request.end_points.push_back(1);
	traj.request.end_points[0] = srvBaseTrajectory.response.end_points[1];
	traj.request.initial_v = 0;
	traj.request.final_v = 0;
	traj.request.x_invert.push_back(0);

	traj.request.spline_groups.push_back(1);
	traj.request.wait_before_group.push_back(.16);
	traj.request.t_shift.push_back(0);
	traj.request.flip.push_back(false);
	traj.request.end_points.push_back(1);
	traj.request.end_points[1] = srvBaseTrajectory.response.end_points[2];
	traj.request.initial_v = 0;
	traj.request.final_v = 0;
	traj.request.x_invert.push_back(0);

	nav_msgs::Path path;
	double last_time = 0;
	/*** ITERATE THROUGH SPLINES ***/
	ROS_ERROR_STREAM("Iterating through splines in pure pursuit test client");
	for (size_t s = 0; s < traj.request.spline_groups.size(); s++)
	{
		//I don't know what priv_num is
		int priv_num = 0;
		if (s > 0)
		{
			priv_num = traj.request.spline_groups[s - 1];
		}

		const int n = round(traj.request.wait_before_group[s] / defined_dt);
		std::vector<spline_coefs> x_splines;
		std::vector<spline_coefs> y_splines;
		std::vector<spline_coefs> orient_splines;

		const int neg_x = traj.request.x_invert[s] ? -1 : 1;
		std::vector<double> end_points_holder;

		//I also don't know what shift_by does
		double shift_by = 0;
		if (s != 0)
		{
			shift_by = traj.request.end_points[priv_num - 1];
		}

		// Make the splines from traj into spline_coefs format. TODO remove traj middleman
		for (int i = priv_num; i < traj.request.spline_groups[s]; i++)
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

			ROS_INFO_STREAM("hrer: " << traj.request.end_points[i] - shift_by << " r_s: " <<  traj.request.spline_groups[s] <<  " s: " << s);
			end_points_holder.push_back(traj.request.end_points[i] - shift_by);
		}

		const double t_shift = traj.request.t_shift[s];
		const bool flip_dirc = traj.request.flip[s];

		//Take derivatives of splines
		std::vector<spline_coefs> x_splines_first_deriv;
		std::vector<spline_coefs> y_splines_first_deriv;
		std::vector<spline_coefs> orient_splines_first_deriv;
		std::vector<spline_coefs> x_splines_second_deriv;
		std::vector<spline_coefs> y_splines_second_deriv;
		std::vector<spline_coefs> orient_splines_second_deriv;
		for (size_t i = 0; i < x_splines.size(); i++)
		{
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
		}

		/*** PARAMETRIZE SPLINE ***/
		ROS_ERROR_STREAM("parametrizing spline in pure pursuit test client");
		std::vector<double> dtds_for_spline;
		std::vector<double> arc_length_for_spline;
		double total_arc;
		//Run spline parametrizing code - also gets dtds and arc lengths
		//This turns the x,y linear positions into a single linear-path-position vs. time curve
		tk::spline spline = parametrize_spline(x_splines_first_deriv, y_splines_first_deriv, end_points_holder,
				total_arc, dtds_for_spline, arc_length_for_spline);

		/*** TRANSFER INTO nav_msgs::Path FORMAT***/
		ROS_ERROR_STREAM("transfering into path format in pure pursuit test client");
		int num_spline_points = 1000;
		double seq = 0;
		for(double current_spline_position = 0; current_spline_position < total_arc; current_spline_position += iteration_length)
		{
			geometry_msgs::PoseStamped pose;
			double current_time = current_spline_position * dtds_for_spline[s] + last_time;

			double x_position;
			double y_position;
			double yaw;
			calc_point(x_splines[s], current_time, x_position);
			calc_point(y_splines[s], current_time, y_position);
			calc_point(orient_splines[s], current_time, yaw);

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
			if(total_arc - current_spline_position < iteration_length)
				last_time = current_time;
		}
		ROS_INFO_STREAM("number of points in path = " << path.poses.size());

	}

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
