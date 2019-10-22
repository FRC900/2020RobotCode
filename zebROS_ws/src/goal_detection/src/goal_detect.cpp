#include <iostream>
#include <mutex>
#include <sstream>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include "teraranger_array/RangeArray.h"

#include "field_obj/Detection.h"
#include "field_obj/Object.h"
#include "objtype.hpp"

#include "GoalDetector.hpp"
#include "goal_detection/GoalDetectionConfig.h"
#include "dynamic_reconfigure_wrapper/dynamic_reconfigure_wrapper.h"

namespace goal_detection
{
	class GoalDetect : public nodelet::Nodelet
	{
		public:

			GoalDetect(void)
			{
			}

			~GoalDetect()
			{
			}

		protected:
			void onInit() override
			{
				nh_ = getMTPrivateNodeHandle();
				image_transport::ImageTransport it(nh_);
				int sub_rate = 1;
				int pub_rate = 1;
				nh_.getParam("sub_rate", sub_rate);
				nh_.getParam("pub_rate", pub_rate);

				nh_.param("hFov", config_.hFov, 105.);
				nh_.param("camera_angle", config_.camera_angle, -25.0);
				nh_.param("blue_scale", config_.blue_scale, 0.90);
				nh_.param("red_scale", config_.red_scale, 0.80);
				nh_.param("otsu_threshold", config_.otsu_threshold, 5);
				nh_.param("min_confidence", config_.min_confidence, 0.30);
				drw_.init(nh_, config_);

				bool no_depth = false;
				nh_.getParam("no_depth", no_depth);

				distance_from_terabee_ = -1;

				if (!no_depth)
				{
					ROS_INFO("starting goal detection using ZED");
					frame_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/left/image_rect_color", sub_rate);
					depth_sub_ = std::make_unique<image_transport::SubscriberFilter>(it, "/zed_goal/depth/depth_registered", sub_rate);
					// ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(xxx)
					rgbd_sync_ = std::make_unique<message_filters::Synchronizer<RGBDSyncPolicy>>(RGBDSyncPolicy(10), *frame_sub_, *depth_sub_);
					rgbd_sync_->registerCallback(boost::bind(&GoalDetect::callback, this, _1, _2));

					camera_info_sub_ = nh_.subscribe("/zed_goal/left/camera_info", sub_rate, &GoalDetect::camera_info_callback, this);
				}
				else
				{
					ROS_INFO("starting goal detection using webcam");
					rgb_sub_ = std::make_unique<image_transport::Subscriber>(it.subscribe("/c920_camera/image_raw", sub_rate, &GoalDetect::callback_no_depth, this));
					terabee_sub_ = nh_.subscribe("/multiflex_1/ranges_raw", 1, &GoalDetect::multiflexCB, this);
				}

				// Set up publisher
				pub_ = nh_.advertise<field_obj::Detection>("goal_detect_msg", pub_rate);

				pub_debug_image_ = it.advertise("debug_image", 2);
			}
			// TODO : move this into GoalDetect / object code to replace the old
			// screen to world / world to screen calcs
			// Use the camera model to convert the pixel coords at the center of the bounding
			// box into real-world coords.
			// pos is the x,y,z coordinates computed using the other method - this is a hack
			// for getting the reported ZED depth of the targer.  Moving this function into the goal detect class will
			// make the depth directly readable by this code
			cv::Point3f get_world_coord_scaled(const image_geometry::PinholeCameraModel &model,
											   const cv::Rect &bounding_rect,
											   const cv::Point3f &pos,
											   const std::string &debug_name) const
			{

				// Center point of left and right bounding rect
				const cv::Point2f uv(
						bounding_rect.tl().x + bounding_rect.width  / 2.0,
						bounding_rect.tl().y + bounding_rect.height / 2.0
						);
				const cv::Point3f world_coord_unit = model.projectPixelTo3dRay(uv);
				const float distance = sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
				const cv::Point3f world_coord_scaled = world_coord_unit * distance;

				const cv::Point3f adj_world_coord_scaled(world_coord_scaled.x, world_coord_scaled.z, -world_coord_scaled.y);
#if 0
				ROS_INFO_STREAM("bounding_rect:" << bounding_rect);
				ROS_INFO_STREAM("uv:" << uv);
				ROS_INFO_STREAM("world_coord_unit:" << world_coord_unit);
				ROS_INFO_STREAM("distance:" << distance);
				ROS_INFO_STREAM("world_coord_scaled:" << world_coord_scaled);
				ROS_INFO_STREAM("pos:" << pos);
				ROS_INFO_STREAM("adj_world_coord_scaled:" << adj_world_coord_scaled);
				ROS_INFO_STREAM("adj_distance:" << adj_world_coord_scaled - pos);
				ROS_INFO_STREAM(debug_name << " : uv : " << uv << " gd_pos : " << pos << " model_pos : " << adj_world_coord_scaled << " difference " << adj_world_coord_scaled - pos);
#endif
				return adj_world_coord_scaled;
			}

			void callback(const sensor_msgs::ImageConstPtr &frameMsg, const sensor_msgs::ImageConstPtr &depthMsg)
			{
				//std::lock_guard<std::mutex> l(camera_mutex_);
				if (!camera_mutex_.try_lock())  // If the previous message is still being
					return;              // processed, drop this one
				cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
				cv_bridge::CvImageConstPtr cvDepth = cv_bridge::toCvShare(depthMsg, sensor_msgs::image_encodings::TYPE_32FC1);

				// Initialize goal detector object the first time
				// through here. Use the size of the frame
				// grabbed from the ZED messages
				if (gd_ == NULL)
				{
					const cv::Point2f fov(config_.hFov * (M_PI / 180.),
										  config_.hFov * (M_PI / 180.) * ((double)cvFrame->image.rows / cvFrame->image.cols));
					gd_ = std::make_unique<GoalDetector>(fov, cvFrame->image.size(), false);
				}
				gd_->setCameraAngle(config_.camera_angle);
				gd_->setBlueScale(config_.blue_scale);
				gd_->setRedScale(config_.red_scale);
				gd_->setOtsuThreshold(config_.otsu_threshold);
				gd_->setMinConfidence(config_.min_confidence);

				//Send current color and depth image to the actual GoalDetector
				gd_->setTargetNum(POWER_PORT_2020);
				gd_->findTargets(cvFrame->image, cvDepth->image);
				std::vector< GoalFound > gfd_power_port = gd_->return_found();

				gd_->setTargetNum(LOADING_BAY_2020);
				gd_->findTargets(cvFrame->image, cvDepth->image);
				std::vector< GoalFound > gfd_loading_bay = gd_->return_found();

				std::vector< GoalFound > gfd;
				gfd.reserve( gfd_power_port.size() + gfd_loading_bay.size() );
				gfd.insert( gfd.end(), gfd_power_port.begin(), gfd_power_port.end() );
				gfd.insert( gfd.end(), gfd_loading_bay.begin(), gfd_loading_bay.end() );

				field_obj::Detection gd_msg;

				gd_msg.header.seq = frameMsg->header.seq;
				gd_msg.header.stamp = frameMsg->header.stamp;
				std::string frame_id = frameMsg->header.frame_id;
				// Remove _optical_frame from the camera frame ID if present
				const size_t idx = frame_id.rfind("_optical_frame");
				if (idx != std::string::npos)
				{
					frame_id.erase(idx);
					frame_id += "_frame";
				}
				gd_msg.header.frame_id = frame_id;

				image_geometry::PinholeCameraModel model;
				model.fromCameraInfo(camera_info_);
				//ROS_INFO_STREAM("Camera_info " << camera_info_);

				for(size_t i = 0; i < gfd.size(); i++)
				{
					field_obj::Object dummy;
					dummy.location.x = gfd[i].pos.y;
					dummy.location.y = gfd[i].pos.x;
					dummy.location.z = gfd[i].pos.z;
					dummy.id = gfd[i].id;
					dummy.confidence = gfd[i].confidence;

					// Center point of left and right bounding rect in world coords
					const cv::Point3f left_world_coord_scaled = get_world_coord_scaled(model, gfd[i].left_rect, gfd[i].left_pos, "left");
					const cv::Point3f right_world_coord_scaled = get_world_coord_scaled(model, gfd[i].right_rect, gfd[i].right_pos, "right");

					// Midpoint of those two should be the goal coords
					const cv::Point3f center_world_coord_scaled = (left_world_coord_scaled + right_world_coord_scaled) / 2.0;

#if 0
					ROS_INFO_STREAM("center gd_pos : " << gfd[i].pos <<
									" model_pos : " << center_world_coord_scaled <<
									" difference " << center_world_coord_scaled - gfd[i].pos);
#endif
					// TODO : pick a coord frame and stick with it?
					dummy.x = center_world_coord_scaled.y;
					dummy.y = center_world_coord_scaled.x;
					dummy.z = center_world_coord_scaled.z;
					gd_msg.objects.push_back(dummy);
#if 0
					ROS_INFO_STREAM("center_world_coord_scaled:" << center_world_coord_scaled);
					ROS_INFO_STREAM("gfd[i].pos:" << gfd[i].pos);
					ROS_INFO_STREAM("difference:" << center_world_coord_scaled - gfd[i].pos);
					// Midpoint along the segment connecting those
					// two center points
					const cv::Point2f center_uv(
							left_uv.x + (right_uv.x - left_uv.x) / 2.0,
							left_uv.y + (right_uv.y - left_uv.y) / 2.0
					);
					const cv::Point3f world_coord = model.projectPixelTo3dRay(center_uv);
					const float distance = sqrt(gfd[i].pos.x * gfd[i].pos.x +
					                            gfd[i].pos.y * gfd[i].pos.y +
					                            gfd[i].pos.z * gfd[i].pos.z);
					ROS_INFO_STREAM("model.fullResolution: " << model.fullResolution());

					ROS_INFO_STREAM("left_rect" << gfd[i].left_rect);
					ROS_INFO_STREAM("right_rect" << gfd[i].right_rect);
					ROS_INFO_STREAM("left_uv" << left_uv);
					ROS_INFO_STREAM("right_uv" << right_uv);
					ROS_INFO_STREAM("center_uv" << center_uv);

					const cv::Point3f gd_pos(gfd[i].pos.y, gfd[i].pos.x, gfd[i].pos.z);

					ROS_INFO_STREAM("gd_pos:" << gd_pos);
					ROS_INFO_STREAM("distance:" << distance);
					ROS_INFO_STREAM("world_coord (unscaled):" << world_coord);
					ROS_INFO_STREAM("projectPixelTo3dRay(unscaled):" << model.projectPixelTo3dRay(center_uv));
					ROS_INFO_STREAM("world_coord (scaled):" << world_coord * distance);
					ROS_INFO_STREAM("difference:" << world_coord * distance - gd_pos);

					const cv::Point3f adj_world_coord(world_coord.z * distance, world_coord.x * distance, -world_coord.y * distance);
					ROS_INFO_STREAM("adj_world_coord:" << adj_world_coord);
					ROS_INFO_STREAM("adj_distance:" << adj_world_coord - gd_pos);
#endif
				}

				pub_.publish(gd_msg);

				if (pub_debug_image_.getNumSubscribers() > 0)
				{
					cv::Mat thisFrame(cvFrame->image.clone());
					gd_->drawOnFrame(thisFrame, gd_->getContours(thisFrame), gfd);
					pub_debug_image_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", thisFrame).toImageMsg());
				}

				if (gd_msg.objects.size() > 0)
				{
					//Transform between goal frame and odometry/map.
					static tf2_ros::TransformBroadcaster br;
					for(size_t i = 0; i < gfd.size(); i++)
					{
						geometry_msgs::TransformStamped transformStamped;

						transformStamped.header.stamp = gd_msg.header.stamp;
						transformStamped.header.frame_id = frame_id;
						std::stringstream child_frame;
						child_frame << "goal_";
						child_frame << i;
						transformStamped.child_frame_id = child_frame.str();

						transformStamped.transform.translation.x = gd_msg.objects[i].location.x;
						transformStamped.transform.translation.y = gd_msg.objects[i].location.y;
						transformStamped.transform.translation.z = gd_msg.objects[i].location.z;

						// Can't detect rotation yet, so publish 0 instead
						tf2::Quaternion q;
						q.setRPY(0, 0, 0);

						transformStamped.transform.rotation.x = q.x();
						transformStamped.transform.rotation.y = q.y();
						transformStamped.transform.rotation.z = q.z();
						transformStamped.transform.rotation.w = q.w();

						br.sendTransform(transformStamped);
					}

					/*
					//Transform between a fixed frame and the goal.
					tf2_ros::Buffer tfBuffer;
					tf2_ros::TransformListener tfListener(tfBuffer);

					geometry_msgs::TransformStamped transformStampedOdomCamera;
					try
					{
					transformStampedOdomCamera = tfBuffer.lookupTransform("odom", cvFrame->header.frame_id,
					ros::Time(0));
					}
					catch (tf2::TransformException &ex)
					{
					ROS_WARN("%s", ex.what());
					ros::Duration(1.0).sleep();
					return;
					}

					geometry_msgs::TransformStamped transformStampedOdomGoal;

					tf2::doTransform(transformStamped, transformStampedOdomGoal, transformStampedOdomCamera);

					br.sendTransform(transformStampedOdomGoal);
					*/
				}
				camera_mutex_.unlock();
			}
			void camera_info_callback(const sensor_msgs::CameraInfoConstPtr &info)
			{
				if (!camera_mutex_.try_lock())  // If the previous message is still being
					return;              // processed, drop this one

				camera_info_ = *info;
				camera_mutex_.unlock();
			}

			void multiflexCB(const teraranger_array::RangeArray& msg)
			{
				// If previous message is still being processed, drop
				// this one.
				if (!multiflex_mutex_.try_lock())
					return;
				double min_dist = std::numeric_limits<double>::max();
				distance_from_terabee_ = -1;
				for(int i = 0; i < 2; i++)
				{
					const double range = static_cast<double>(msg.ranges[i].range);
					if(!std::isnan(range))
						min_dist = std::min(min_dist, range);
				}
				if (min_dist != std::numeric_limits<double>::max())
					distance_from_terabee_ = min_dist;
				multiflex_mutex_.unlock();
			}

			void callback_no_depth(const sensor_msgs::ImageConstPtr &frameMsg)
			{
				cv_bridge::CvImageConstPtr cvFrame = cv_bridge::toCvShare(frameMsg, sensor_msgs::image_encodings::BGR8);
				cv::Mat depthMat(cvFrame->image.size(), CV_32FC1, cv::Scalar(distance_from_terabee_));
				callback(frameMsg, cv_bridge::CvImage(std_msgs::Header(), "32FC1", depthMat).toImageMsg());
			}
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> RGBDSyncPolicy;

			ros::NodeHandle                                                nh_;
			std::unique_ptr<image_transport::SubscriberFilter>             frame_sub_;
			std::unique_ptr<image_transport::SubscriberFilter>             depth_sub_;
			std::unique_ptr<image_transport::Subscriber>                   rgb_sub_;
			std::unique_ptr<message_filters::Synchronizer<RGBDSyncPolicy>> rgbd_sync_;
			ros::Subscriber                                                terabee_sub_;
			ros::Publisher                                                 pub_;
			image_transport::Publisher                                     pub_debug_image_;
			std::unique_ptr<GoalDetector>                                  gd_;
			double                                                         distance_from_terabee_;
			goal_detection::GoalDetectionConfig                            config_;
			DynamicReconfigureWrapper<goal_detection::GoalDetectionConfig> drw_;
			std::mutex                                                     camera_mutex_;
			std::mutex                                                     multiflex_mutex_;

			ros::Subscriber                                                camera_info_sub_;
			sensor_msgs::CameraInfo                                        camera_info_;
	};
} // namspace

PLUGINLIB_EXPORT_CLASS(goal_detection::GoalDetect, nodelet::Nodelet)
