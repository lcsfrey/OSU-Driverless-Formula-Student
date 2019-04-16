#include "see_ego_motion/SeeEgoMotion.hpp"
#include <see_ego_motion/see_ego_motion_interface.h>
#include <string>

using see_ego_motion::see_ego_motion_interface;

SeeEgoMotion::SeeEgoMotion(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "can_driver_publication";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeEgoMotion::canTopicCallback, this);

  std::string published_topic = "odom_filtered";
  publisher_odom = nodeHandle.advertise<see_ego_motion_interface>(published_topic, 1, true);

  std::string advertised_service = "see_ego_motion_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeEgoMotion::serviceCallback, this);

  ROS_INFO("started node see_ego_motion");
}

SeeEgoMotion::~SeeEgoMotion()
{
}

void SeeEgoMotion::run()
{
  see_ego_motion_interface message;

  message.x_global = 0;
  message.y_global = 0;
  message.theta_global = 0;

  message.velocity_longtitudonal = 0;
  message.velocity_lateral = 0;
  message.yaw_rate = 0;

  message.pos_covariance = {0, 0, 0, 0};
  message.theta_variance = 0;
  
  message.delta_time = ros::Duration(0);

  publisher_odom.publish(message);
}


void SeeEgoMotion::canTopicCallback(const std_msgs::String::ConstPtr& message)
{
}

bool SeeEgoMotion::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_ego_motion service response";
  return true;
}