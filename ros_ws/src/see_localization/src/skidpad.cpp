//#include "see_localization/SeeLocalization.hpp"

/*#include <string>

SeeLocalization::SeeLocalization(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "see_localization_subscription";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLocalization::topicCallback, this);

  std::string published_topic = "see_localization_publication";
  publisher = nodeHandle.advertise<std_msgs::String>(published_topic, 1);

  std::string advertised_service = "see_localization_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLocalization::serviceCallback, this);

  ROS_INFO("started node see_localization");
}

SeeLocalization::~SeeLocalization()
{
}

void SeeLocalization::run()
{
  std_msgs::String message;
  message.data = "see_localization publication";
  publisher.publish(message);
}


void SeeLocalization::topicCallback(const std_msgs::String::ConstPtr& message)
{
}

bool SeeLocalization::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_localization service response";
  return true;
}