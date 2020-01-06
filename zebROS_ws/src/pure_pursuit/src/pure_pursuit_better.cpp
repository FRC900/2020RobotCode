/* 1. information you have: current position and velcocity of the robot, waypoints already limited for velocity and acceleration
 * 2. find path point closest to the position of the robot
 * 3. find the goal point -- that is, point which the robot should drive to (based on lookahead distance)
 */
#include "pure_pursuit/pure_pursuit_better.h"

void PurePursuit::loadPath(const T &path)
{
	path_ = path;
	num_waypoints_ = path_.poses.size();
        path_length_ = 0;
        for(size_t i; i < num_waypoints_; ++i)
        {
            start_x = path_.poses[i].pose.position.x;
            start_y = path_.poses[i].pose.position.y;
            end_x = path_.poses[i+1].pose.position.x;
            end_y = path_.poses[i+1].pose.position.y;
            
            path_length_ += hypot(end_x - start_x, end_y - start_y);
            vec_path_length_.append(path_length);
        }
}

// The idea would be to have other code be responsible for getting current
// position and passing it in to run. run would then return a pose (x_pos, y_pos,
// orientation) and whoever called run would be responsible for sending that
// where it needs to go.
geometry_msgs::Pose PurePursuit::run(nav_msgs::Odometry odom)
{
	ROS_INFO_STREAM("----------------------------------------------");
	ROS_INFO_STREAM("current_position = " << odom.pose.pose.position.x
			<< " " << odom.pose.pose.position.y);

        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;
        double normal_found = false;
        double current_x_path, current_y_path;
        size_t current_waypoint_index;
	double minimum_distance = std::numeric_limits<double>::max();

        double start_x;
        double start_y;
        double end_x;
        double end_y;

        double magnitude_projection;

	// Find point in path closest to odometry reading
	// TODO (KCJ) - another possibility here is looking to see which segment between two wayponints
	// the current position is normal to.  That is, if it is off track, assume it is off track to the left
	// or right of the desired path, and check to see which segment it is on if the point were projected
	// perpendicular back to the correct track.
	// See e.g. https://stackoverflow.com/questions/17581738/check-if-a-point-projected-on-a-line-segment-is-not-outside-it
	// It could hit multiple segments, though, so maybe the minimum distance of segments it is normal to, using
	// the right angle distance to the closest point along each segment
	// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
	// This would also potentially give a location between two segments as the current
	// location, which makes the next lookahead point also somewhere
	// between two waypoints.
        for(size_t i = 0; i < num_waypoints_; i++)
        {
            // The line segment between this waypoint and the next waypoint
            start_x = path_.poses[i].pose.position.x;
            start_y = path_.poses[i].pose.position.y;
            end_x = path_.poses[i+1].pose.position.x;
            end_y = path_.poses[i+1].pose.position.y;

            double dx = end_x - start_x;
            double dy = end_y - start_y;
            double innerProduct = (current_x - start_x)*dx + (current_y - start_y)*dy;
            // If the current position is normal to this segment
            if(0 <= innerProduct && innerProduct <= dx*dx + dy*dy)
            {
                normal_found = true;
                // Find location of projection onto path
                // Distance from waypoint to projection onto path
                magnitude_projection = innerProduct / hypot(dx, dy);
                // Find path location
                current_x_path = start_x + magnitude_projection*(dx/hypot(dx, dy));
                current_y_path = start_y + magnitude_projection*(dy/hypot(dx, dy));

                // Update minimum distance from path
                double distance_from_path = hypot(current_x - current_x_path, current_y - current_y_path); 
                if(distance_from_path < minimum_distance)
                {
                    minimum_distance = distance_from_path;
                    current_waypoint_index = i;
                }
            }
        }

	ROS_INFO_STREAM("minimum_distance = " << minimum_distance);

        double distance_to_travel = hypot(dx, dy) - magnitude_projection; // Current value is distance that the robot will travel to reach the next waypoint
        // Determine next waypoint
        size_t end_i = current_waypoint_index;
        for(; end_i < num_waypoints_; ++end_i)
        {
            start_x = path_.poses[end_i].pose.position.x;
            start_y = path_.poses[end_i].pose.position.y;
            end_x = path_.poses[end_i+1].pose.position.x;
            end_y = path_.poses[end_i+1].pose.position.y;

            if(distance_to_travel + hypot(end_x - start_x, end_y - start_y) < lookahead_distance)
                distance_to_travel += hypot(end_x - start_x, end_y - start_y);
            else
                break;
        }

        // Add fraction of distance between waypoints to find final x and y
        double dx = end_x - start_x;
        double dy = end_y - start_y;
        double final_x = path_.poses[end_i].pose.position.x + (lookahead_distance - distance_to_travel) * (dx / hypot(dx, dy)); 
        double final_y = path_.poses[end_i].pose.position.y + (lookahead_distance - distance_to_travel) * (dy / hypot(dx, dy)); 

        // Determine target orientation
        double roll, pitch, start_yaw, end_yaw;
        // Convert to Euler
	tf::Quaternion waypoint_q_start(
			path_.poses[end_i].pose.orientation.w,
			path_.poses[end_i].pose.orientation.x,
			path_.poses[end_i].pose.orientation.y,
			path_.poses[end_i].pose.orientation.z);
	tf::Matrix3x3(waypoint_q_end).getRPY(roll, pitch, start_yaw);
	tf::Quaternion waypoint_q_end(
			path_.poses[end_i + 1].pose.orientation.w,
			path_.poses[end_i + 1].pose.orientation.x,
			path_.poses[end_i + 1].pose.orientation.y,
			path_.poses[end_i + 1].pose.orientation.z);
	tf::Matrix3x3(waypoint_q_end).getRPY(roll, pitch, end_yaw);
        // Find orientation between the waypoints
        double final_orientation = start_yaw + (end_yaw - start_yaw) * ((lookahead_distance - distance_to_travel) / hypot(dx, dy));
        // Convert back to quaternion
        tf2::Quaternion q_final;
        q_final.setRPY(0, 0, final_orientation);

        // Return Pose of target position
        geometry_msgs::Pose target_pos;
        target_pos.position.x = final_x;
        target_pos.position.y = final_y;
        target_pos.position.z = 0;
        target_pos.orientation.x = q_final.x;
        target_pos.orientation.y = q_final.y;
        target_pos.orientation.z = q_final.z;
        target_pos.orientation.w = q_final.w;

        double total_distance_travelled = vec_path_length_[current_waypoint_index] + magnitude_projection; 

        return target_pos, total_distance_travelled;
}
