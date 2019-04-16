#include "see_ego_motion/testing/SeeEgoMotionTest.hpp"
#include <see_ego_motion/see_ego_motion_interface.h>
#include <see_ego_motion/Error.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64.h>

#define ODOM_MARKER_ID 1
#define SIM_MARKER_ID 0

using see_ego_motion::see_ego_motion_interface;
using see_ego_motion::Error;

SeeEgoMotionTest::SeeEgoMotionTest(ros::NodeHandle & nodeHandle) : nodeHandle(nodeHandle)
{
    this->pub_rviz_odom = nodeHandle.advertise<visualization_msgs::Marker>("odometry_visualization/odom_marker", 1);
    this->pub_rviz_sim = nodeHandle.advertise<visualization_msgs::Marker>("odometry_visualization/sim_marker", 1);
    this->pub_rviz_path = nodeHandle.advertise<nav_msgs::Path>("odometry_visualization/odom_path", 1);

    this->pub_normError = nodeHandle.advertise<see_ego_motion::Error>("odometry_error", 1);
    this->sub_odom = nodeHandle.subscribe("/see_ego_motion/odom_filtered", 1, &SeeEgoMotionTest::odomTopicCallback, this);
    this->sub_simulation = nodeHandle.subscribe("/ground_truth/state", 1, &SeeEgoMotionTest::simulationTopicCallback, this);

    ROS_INFO("started node see_ego_motion_test");
}

void SeeEgoMotionTest::run() { /* nothing to do here */ } 

void SeeEgoMotionTest::odomTopicCallback(const see_ego_motion_interface::ConstPtr & msg) 
{
    lastOdomEstimate = odomEstimateToPose(msg);
    geometry_msgs::PoseStamped lastOdomStamped;
    lastOdomStamped.pose = lastOdomEstimate;
    lastOdomStamped.header = msg->header;
    odomPath.poses.push_back(lastOdomStamped);
    showArrow(lastOdomEstimate, ODOM_MARKER_ID);
    this->pub_rviz_path.publish(odomPath);
}

void SeeEgoMotionTest::simulationTopicCallback(const nav_msgs::OdometryConstPtr& msg)
{
    if(!haveStartLocation) //save first message
    {
        startLocation = msg->pose.pose.position;
        startOrientation = msg->pose.pose.orientation;
        haveStartLocation = true;
    }

    geometry_msgs::Pose msgPose = msg->pose.pose; //because const

    //move vehicle location in rviz to start at (0, 0, 0), with heading 0;
    msgPose.position.x -= startLocation.x;
    msgPose.position.y -= startLocation.y;

    //It looks weird, but don't question it
    msgPose.position.z = -msgPose.position.x;
    msgPose.position.x = msgPose.position.y;
    msgPose.position.y = msgPose.position.z;
    msgPose.position.z = 0;

    tf::Quaternion startOrientationTf, currentOrientationTf;
    tf::quaternionMsgToTF(startOrientation, startOrientationTf);
    tf::quaternionMsgToTF(msgPose.orientation, currentOrientationTf);
    tf::Quaternion correctedOrientation = startOrientationTf.inverse() * currentOrientationTf;
    tf::quaternionTFToMsg(correctedOrientation, msgPose.orientation);

    showArrow(msgPose, SIM_MARKER_ID);

    //calculate and publish error
    //Euclidean error, lateral error, linear error, we want everything
    float errorX = msgPose.position.x - lastOdomEstimate.position.x;
    float errorY = msgPose.position.y - lastOdomEstimate.position.y;

    see_ego_motion::Error e;
    float x = errorX;
    float y = errorY;
    float angle = msgPose.orientation.z - atan2f(errorX, errorY);

    e.euclidean = sqrt(x * x + y * y);
    e.linear = e.euclidean * cos(angle);
    e.lateral = e.euclidean * sin(angle);

    pub_normError.publish(e);
}

void SeeEgoMotionTest::showArrow(geometry_msgs::Pose state, int marker_id)
{
    uint32_t shape = visualization_msgs::Marker::ARROW;

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/base_link";
    marker.header.stamp = ros::Time::now();

    marker.ns = "odometry_visualizer";
    marker.id = marker_id;

    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = state;

    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    marker.lifetime = ros::Duration();

    if(marker_id == ODOM_MARKER_ID)
    {
        //orange
        marker.color.r = 1.0f;
        marker.color.g = 0.5f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        pub_rviz_odom.publish(marker);
    }
    else if(marker_id == SIM_MARKER_ID)
    {
        //green
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0f;
        pub_rviz_sim.publish(marker);
    }
}

geometry_msgs::Pose SeeEgoMotionTest::odomEstimateToPose(const see_ego_motion_interface::ConstPtr& odom)
{
    geometry_msgs::Pose pose;

    pose.position.x = odom->x_global;
    pose.position.y = odom->y_global;
    pose.position.z = 0;

    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(odom->theta_global / 2);
    pose.orientation.w = cos(odom->theta_global / 2);

    return pose;
}