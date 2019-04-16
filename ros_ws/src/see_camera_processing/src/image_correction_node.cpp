#include <ros/ros.h>
#include "see_camera_processing/ImageCorrection.hpp"
#include <image_transport/image_transport.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_camera_processing_image_correction");
  ros::NodeHandle nodeHandle("/see_camera_processing");

  ImageCorrection rosPackageTemplate(nodeHandle);

  ros::spin();

  return 0;
}

ImageCorrection::ImageCorrection(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle), it(nodeHandle)
{
  std::string subscribed_topic = "see_camera_processing_subscription";
  subscriber = it.subscribe(subscribed_topic, 1, &ImageCorrection::topicCallback, this);

  std::string published_topic = "enhanced_image";
  publisher = it.advertise(published_topic, 1);

  ROS_INFO("started node see_camera_processing_image_correction");
}

ImageCorrection::~ImageCorrection()
{
}

void ImageCorrection::topicCallback(const sensor_msgs::ImageConstPtr& msg)
{
  //TODO: Apply corrections to the image

  publisher.publish(msg);
}