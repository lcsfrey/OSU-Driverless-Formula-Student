#include "see_camera_processing/SeeCameraProcessing.hpp"

#include <string>

SeeCameraProcessing::SeeCameraProcessing(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "see_camera_processing_subscription";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeCameraProcessing::topicCallback, this);

  std::string published_topic = "see_camera_processing_publication";
  publisher = nodeHandle.advertise<std_msgs::String>(published_topic, 1);

  std::string advertised_service = "see_camera_processing_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeCameraProcessing::serviceCallback, this);

  ROS_INFO("started node see_camera_processing");
}

SeeCameraProcessing::~SeeCameraProcessing()
{
}

void SeeCameraProcessing::run()
{
  std_msgs::String message;
  message.data = "see_camera_processing publication";
  publisher.publish(message);
}


void SeeCameraProcessing::topicCallback(const std_msgs::String::ConstPtr& message)
{
}

bool SeeCameraProcessing::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_camera_processing service response";
  return true;
}