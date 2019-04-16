#pragma once

#include "see_camera_processing/Algorithm.hpp"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ImageCorrection
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ImageCorrection(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ImageCorrection();

private:
 /*!
  * ROS topic callback method.
  * @param message the received message.
  */
  void topicCallback(const sensor_msgs::ImageConstPtr& msg);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle;

  //! Image_transport handle.
  image_transport::ImageTransport it;

  //! Image_transport subscriber.
  image_transport::Subscriber subscriber;

  //! Image_transport publisher.
  image_transport::Publisher publisher;
};