#pragma once

#include "see_camera_processing/Algorithm.hpp"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include <image_transport/image_transport.h>
#include "see_camera_processing/ImageCones.h"
#include "see_camera_processing/ImageCone.h"
#include "see_camera_processing/ImageCoordinates.h"

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ConeDetection
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ConeDetection(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ConeDetection();

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

  //! ROS topic publisher.
  ros::Publisher publisher;
};