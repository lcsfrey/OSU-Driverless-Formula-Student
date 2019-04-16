#include <ros/ros.h>
#include "see_camera_processing/ConeDetection.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_camera_processing_cone_detection");
  ros::NodeHandle nodeHandle("/see_camera_processing");

  ConeDetection rosPackageTemplate(nodeHandle);

  ros::spin();

  return 0;
}

ConeDetection::ConeDetection(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle), it(nodeHandle)
{
  std::string subscribed_topic = "enhanced_image";
  subscriber = it.subscribe(subscribed_topic, 1, &ConeDetection::topicCallback, this);

  std::string published_topic = "cone_data";
  publisher = nodeHandle.advertise<see_camera_processing::ImageCones>(published_topic, 1);

  ROS_INFO("started node see_camera_processing_cone_detection");
}

ConeDetection::~ConeDetection()
{
}

void ConeDetection::topicCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //TODO Detect Position, Type and Size of Cones in the received Image
  
  see_camera_processing::ImageCones detectedCones;
  publisher.publish(detectedCones);
}