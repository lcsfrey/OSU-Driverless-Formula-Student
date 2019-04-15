#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ouster_ros/os1_ros.h>
#include <ouster_ros/point_os1.h>
#include <see_lidar_processing/Coordinates.h>
#include <see_lidar_processing/ConeColor.h>


using CloudOS1 = pcl::PointCloud<ouster_ros::OS1::PointOS1>;


/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SeeLidarConeDetector
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SeeLidarConeDetector(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SeeLidarConeDetector();

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
   * lidar_x_offset             x offset of the lidar mounting position
   * lidar_y_offset             y offset of the lidar mounting position
   * lidar_pitch                pitch angle (degree) of the lidar mounting position
   * lidar_roll                 roll angle (degree) of the lidar mounting position
   * lidar_yaw                  yaw angle (degree) of the lidar mounting position
   * small_cone_height_by_rules cone height (left cone, roght cone) defined by rules
   * small_cone_width_by_rules  cone width (left cone, roght cone) defined by rules
   * small_cone_depth_by_rules  cone depth (left cone, roght cone) defined by rules
   * big_cone_height_by_rules   cone height (start cone, finish cone) defined by rules
   * big_cone_width_by_rules    cone width (start cone, finish cone) defined by rules
   * big_cone_depth_by_rules    cone depth (start cone, finish cone) defined by rules
   * cone_size_threshold        
   * use_pcl                    defines, if the detector should use pcl algorithms (e.g. matrix transformation) or not
   */
  // default parameter values
  double lidar_x_offset = 0;
  double lidar_y_offset = 0;
  double lidar_pitch = 0;
  double lidar_roll = 0;
  double lidar_yaw = 0;
  double small_cone_height_by_rules = 0.325;
  double small_cone_width_by_rules = 0.228;
  double small_cone_depth_by_rules = 0.228;
  double big_cone_height_by_rules = 0.505;
  double big_cone_width_by_rules = 0.285;
  double big_cone_depth_by_rules = 0.285;
  double cone_size_threshold = 0;
  bool multithreading = true;
  bool use_pcl = true;

  /*!
   * loadConfiguration gets the parameters from the ros parameter server and puts the important values to the object attributes
   * this has to be called in the constructor (or anywhere else, but before using the calculation methods) to get the values from the config file
   * return: true, if all parameters could be loaded succesfully, false otherwise (kept some parameters at its default values)
   */
  bool loadConfiguration();
  /*!
   * detect the cones contained in the clusters
   */
  void detectCones(const sensor_msgs::PointCloud2ConstPtr& message);
  /*!
   * create the clusters from the whole pointcloud
   */
  std::vector<CloudOS1> getClusters(const CloudOS1& pointCloud);
  /*!
   * returns the 2D-distance of the point from the origin (the Lidar)
   */
  double getDistance2D(const ouster_ros::OS1::PointOS1& point);
  /*!
   * calculates the dimensions of the cluster
   */
  void getClusterDimensions(
    const CloudOS1& cluster,
    double& witdh,
    double& depth,
    double& height,
    double& nearest_distance
  );
  /*!
   * calculates the probability that the cluster is a cone
   */
  double getConeProbability(CloudOS1 &cluster);

  /*!
   * calculates the center of the cluster
   */
  see_lidar_processing::Coordinates getClusterCenter(const CloudOS1& cluster);

  /*!
   * creates the height probability factor of the cone 
   */
  double getHeightProbabilityFactor(double height);

  /*!
   * creates the width probability factor of the cone 
   */
  double getWidthProbabilityFactor(double width);

  /*!
   * creates the depth probability factor of the cone 
   */
  double getDepthProbabilityFactor(double depth);

  /*!
   * transform the cloud back to the "global coordinate system" (origin is Back axle)
   * calculation is done with an "own algorithm" and a maual matrix rotation
   */
  CloudOS1 transformCloud(const CloudOS1& cloud);

  /*!
   * transform the cloud back to the "global coordinate system" (origin is Back axle)
   * calculation is done using algorithms from the pcl (PointCloud Library)
   */
  CloudOS1 transformCloudUsingPcl(const CloudOS1& cloud);

  /*!
   * calculating the cone color
   * return a vector with the possible colors and its probability
   */
  std::vector<see_lidar_processing::ConeColor> getConeColor(const CloudOS1& cone);

  /*!
   * creates for each cluster a visualization (Pointcloud) to show the center of the given clusters
   * This will draw a vertical line (height=2m)
   * The intensity is used to adapt the color
   */
  CloudOS1 createPositionVisualization(
    const see_lidar_processing::Coordinates& position,
    uint16_t intensity
  );

  /*!
   * creates a legend for a visualization (a horizontal line with color gradient)
   */
  CloudOS1 createLegendForVisualization(double width);

  /*
   * creates a message from the given pointcloud and publishes it with the specified publisher
   */
  void publishPointcloud(
    const CloudOS1& cloud,
    const ros::Publisher& publisher,
    const std::string& frame_id,
    const ros::Time& stamp
  );

 /*!
  * ROS topic callback method.
  * @param message the received message.
  */
  void topicCallback(const std_msgs::String::ConstPtr& message);

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
  ros::Publisher publisher;
  ros::Publisher publisher2;
  ros::Publisher cones_publisher;
  ros::Publisher transform_publisher;

  //! ROS service server.
  ros::ServiceServer serviceServer;
};

