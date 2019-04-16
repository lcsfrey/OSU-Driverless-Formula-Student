#pragma once

#include "see_ego_motion/Algorithm.hpp"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SeeEgoMotion
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SeeEgoMotion(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SeeEgoMotion();

  /*!
   * Method for running the ROS node algorithm.
   */
  void run();

private:
 /*!
  * ROS topic callback method.
  * @param message the received message.
  */
  void canTopicCallback(const std_msgs::String::ConstPtr& message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle;

  //! ROS topic subscriber.
  ros::Subscriber subscriber;

  //! ROS topic publisher.
  ros::Publisher publisher_odom;

  //! ROS service server.
  ros::ServiceServer serviceServer;

  //! Algorithm computation object.
  Algorithm algorithm;
};