#pragma once

#include "see_localization/Algorithm.hpp"
#include "see_localization/SLAM_cone_msg.h"
#include "see_localization/world_map_msg.h"
#include "see_localization/vehicle_pose_msg.h"
#include "see_localization/DataGenerator.hpp"
#include "see_localization/SLAM.hpp"
#include "see_lidar_processing/ConeContainer.h"
#include "see_camera_processing/ConeContainer.h"
#include "see_ego_motion/see_ego_motion_interface.h"
#include "tf/transform_datatypes.h"
#include <Eigen/Geometry> 
#include <vector>
#include <ros/ros.h>
#include <see_localization/Sensor.hpp>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SeeLocalization {
public:

	/*!
	 * Constructor.
	 * @param nodeHandle the ROS node handle.
	 */
	SeeLocalization(ros::NodeHandle& nodeHandle);

	/*!
	 * Destructor.
	 */
	virtual ~SeeLocalization();

	/*!
	 * Method for running the ROS node algorithm.
	 */
	void run();

private:
	
	int index = 0;//used for visualizing fake data

	//creating particles vector
	std::vector<Particle> saved_particles;
	
	GlobalPose gp;	

	/*!
	 * ROS topic callback method.
	 * @param message the received message.
	 */
	void lidarMsgReceive(const see_lidar_processing::ConeContainer& cone_container);

	/*!
	 * ROS topic callback method.
	 * @param message the received message.
	 */
	void cameraMsgReceive(const see_camera_processing::ConeContainer& cone_container);

	/*!
	 * ROS topic callback method.
	 * @param message the received message.
	 */
	void ego_motion_callback(const see_ego_motion::see_ego_motion_interface& ego_interface);
	/*
	 * method to create and populate message for egomotion
	 */
	see_ego_motion::see_ego_motion_interface create_pose_msg(Particle Best);
	
	/*
	 * method to create and populate message for trajectory planning.
	 */
	see_localization::world_map_msg create_map_msg(Particle Best);

	/*
	 * method that returns the particle with the highest weight from a set.
	 */
	Particle getBestParticle(std::vector<Particle> allParticles);

	visualization_msgs::MarkerArray vis_msg_lm(std::vector<Particle> v_particles);

	geometry_msgs::PoseStamped vis_msg_pose(std::vector<Particle> v_particles);

	geometry_msgs::PoseStamped real_pose(int index);

	geometry_msgs::PoseArray vis_pose_array(std::vector<Particle> v_particles);

	/*!
	 * ROS service server callback.
	 * @param request the request of the service.
	 * @param response the provided response.
	 * @return true if successful, false otherwise.
	 */
	bool serviceCallback(std_srvs::Trigger::Request& request,
			std_srvs::Trigger::Response& response);

	//! ROS node handle.
	ros::NodeHandle& nodeHandle;

	//! ROS topic subscriber.
	ros::Subscriber subscriber;
	ros::Subscriber subscriber_ego;
	ros::Subscriber subscriber_lidar;
	ros::Subscriber subscriber_camera;

	//! ROS topic publisher.
	ros::Publisher publisher_map;
	ros::Publisher publisher_pose;
	ros::Publisher publisher_rviz_lm;
	ros::Publisher publisher_rviz_pose;
	ros::Publisher publisher_rviz_real_pose;
	ros::Publisher publisher_rviz_pose_cluster;

	//! ROS service server.
	ros::ServiceServer serviceServer;

	//! Algorithm computation object.
	Algorithm algorithm;
};
