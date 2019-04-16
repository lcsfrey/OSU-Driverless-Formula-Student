#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ouster_ros/os1_ros.h>
#include <ouster_ros/point_os1.h>
#include <string>

using CloudOS1 = pcl::PointCloud<ouster_ros::OS1::PointOS1>;
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SeeLidarPointcloudClusterer
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SeeLidarPointcloudClusterer(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SeeLidarPointcloudClusterer();

  /*!
   * Method for running the ROS node algorithm.
   */
  void run(uint64_t);

  enum Point_Type
  {
    CORE,
    BORDER,
    NOISE,
    UNDEFINED
  };

  struct DBSCAN_Point
  {
    //TODO: make os1_point to a pointer --> object doesn't get copied --> faster
    ouster_ros::OS1::PointOS1* os1_point;
    bool is_visited;
    Point_Type type;
    bool is_in_cluster;

    // required for std::find()
    const bool operator==(const DBSCAN_Point& rhs) const;
    
  };

  /*!
   * get the parameter, if multithreading is enabled
   */
  bool isMultithreadingEnabled();

private:
  // Epsilon: max. Distanz (m) between points to belong to the same cluster
  double eps;
  // minPts: min. count of Points to "create" a new cluster
  unsigned int minPts;

  // multithreading: specifies, if multithreading should be used
  bool multithreading;

  /*!
   * loadConfiguration gets the parameters from the ros parameter server and puts the important values to the object attributes
   * return: true, if all parameters could be loaded succesfully, false otherwise (kept some parameters at its default values)
   */
  bool loadConfiguration();

  double getDistance(
    const ouster_ros::OS1::PointOS1& point_1, 
    const ouster_ros::OS1::PointOS1& point_2
  );

  bool isClusterMember(
    const int& point_idx, 
    const std::vector<std::vector<ouster_ros::OS1::PointOS1*>>& clusters, 
    const CloudOS1& complete_point_cloud
  );

  std::vector<ouster_ros::OS1::PointOS1*> expandCluster_WithIndexReference(
    ouster_ros::OS1::PointOS1& point, 
    std::vector<int>& neighborhood_points, 
    std::vector<ouster_ros::OS1::PointOS1*>& cluster, 
    const std::vector<std::vector<ouster_ros::OS1::PointOS1*>>& clusters, 
    CloudOS1& all_points, 
    std::vector<bool>& is_point_visited
  );

  std::vector<SeeLidarPointcloudClusterer::DBSCAN_Point*> expandCluster_copybackCluster(
    DBSCAN_Point& point, 
    std::vector<DBSCAN_Point*>& neighborhood_points,
    std::vector<DBSCAN_Point>& all_points
  );

  void expandCluster_Reference(
    const DBSCAN_Point& point, 
    std::vector<DBSCAN_Point*>& neighborhood_points,
    std::vector<DBSCAN_Point*>& cluster,
    std::vector<DBSCAN_Point>& all_points
  );

  bool isSamePoint(
    const SeeLidarPointcloudClusterer::DBSCAN_Point& point_1, 
    const SeeLidarPointcloudClusterer::DBSCAN_Point& point_2
  );
 /*!
  * ROS topic callback method.
  * @param message the received message.
  */
  void dbscanClustering(const sensor_msgs::PointCloud2ConstPtr& message);
  void dbscanClustering_WithIndexReferences(const sensor_msgs::PointCloud2ConstPtr& message);

  void updateClusterBounds(
    const ouster_ros::OS1::PointOS1& point,
    double& left_edge,
    double& right_edge,
    double& top_edge,
    double& bottom_edge,
    double& front_edge,
    double& back_edge
  );

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

  //! ROS service server.
  ros::ServiceServer serviceServer;
};

