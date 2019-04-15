#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ouster_ros/os1_ros.h>
#include <ouster_ros/point_os1.h>
#include <ctime>
#include <iostream>
#include <string>
#include <cstdint>
#include <cmath>

using CloudOS1 = pcl::PointCloud<ouster_ros::OS1::PointOS1>;
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SeeLidarPointcloudPrefilter
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SeeLidarPointcloudPrefilter(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SeeLidarPointcloudPrefilter();

  /*!
   * Method for running the ROS node algorithm.
   */
  void run(uint64_t);

  /*!
   * get the parameter, if multithreading is enabled
   */
  bool isMultithreadingEnabled();

private:
  /*!
   * configuration parameters
   */
  double ground_threshold;
  double area_size;
  bool multithreading;
  /*!
  * ROS topic callback method.
  * @param message the received message.
  */
  void topicCallback(const sensor_msgs::PointCloud2ConstPtr& message);

  /*!
   * ROS service server callback.
   * @param request the request of the service.
   * @param response the provided response.
   * @return true if successful, false otherwise.
   */
  bool serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

  /*!
   * loadConfiguration gets the parameters from the ros parameter server and puts the important values to the object attributes
   * return: true, if all parameters could be loaded succesfully, false otherwise (kept some parameters at its default values)
   */
  bool loadConfiguration();

  std::vector<int> get_front_points(const CloudOS1& cloud);

  void get_indices_for_ground_plane(int cloud_size);
  std::vector<double> get_values_from_index(std::vector<double> point_vector,CloudOS1 editable_cloud, int rand_idx);

  std::vector<double> calculate_cross_product(std::vector<double> vector1, std::vector<double> vector2);

  std::vector<std::vector<int>> split_point_cloud_in_areas(double width, double height, CloudOS1 editable_cloud);

  CloudOS1 ransac(CloudOS1 editable_cloud, int iterations, unsigned int min_points, int loops_per_iteration, double threshhold, CloudOS1 *ground_points, unsigned int p);
  //rand index to calc ground plane
  int rand_idx1;
  int rand_idx2;
  int rand_idx3;

  double field_of_interest_width = 20.0;
  double field_of_interest_height = 20.0;

  //! ROS node handle.
  ros::NodeHandle& nodeHandle;

  //! ROS topic subscriber.
  ros::Subscriber subscriber;

  //! ROS topic publisher.
  ros::Publisher publisher;

  ros::Publisher publisher2;

  //! ROS service server.
  ros::ServiceServer serviceServer;
};

