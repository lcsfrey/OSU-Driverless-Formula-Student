#pragma once

#include <ros/ros.h>
#include <see_ego_motion/see_ego_motion_interface.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

using see_ego_motion::see_ego_motion_interface;

class SeeEgoMotionTest
{
public:
    SeeEgoMotionTest(ros::NodeHandle& nodeHandle);
    void run();
private:
    void odomTopicCallback(const see_ego_motion_interface::ConstPtr& message);
    void simulationTopicCallback(const nav_msgs::OdometryConstPtr& message);
    ros::NodeHandle & nodeHandle;
    ros::Subscriber sub_odom;
    ros::Subscriber sub_simulation;

    // publish shapes to rviz
    ros::Publisher pub_rviz_odom;
    ros::Publisher pub_rviz_sim;
    ros::Publisher pub_rviz_path;

    // publish error in pose estimation
    ros::Publisher pub_normError;
    geometry_msgs::Pose lastOdomEstimate;

    // odometry path
    nav_msgs::Path odomPath;

    // for centering simulation start position
    geometry_msgs::Point startLocation;
    geometry_msgs::Quaternion startOrientation;
    bool haveStartLocation;

    // helper for RVIZ
    void showArrow(geometry_msgs::Pose state, int marker_id);

    // helper for other stuff
    geometry_msgs::Pose odomEstimateToPose(const see_ego_motion_interface::ConstPtr& odom);
};
