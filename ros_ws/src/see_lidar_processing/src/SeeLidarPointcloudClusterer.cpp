#include "see_lidar_processing/SeeLidarPointcloudClusterer.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <libgen.h>
#include <unistd.h>
#include <XmlRpcValue.h>
#include <boost/thread.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_lidar_pointcloud_clusterer");
  ros::NodeHandle nodeHandle("see_lidar");

  std::string current_working_dir = dirname(argv[0]);
  std::cout << "Path: " << current_working_dir << std::endl;

  SeeLidarPointcloudClusterer pointcloudClusterer(nodeHandle);

  // check for multithreading
  if(pointcloudClusterer.isMultithreadingEnabled()) {
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown();
  } else {
    ros::spin();    
  }

  return 0;
}

SeeLidarPointcloudClusterer::SeeLidarPointcloudClusterer(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "see_lidar_points_of_interest";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLidarPointcloudClusterer::dbscanClustering, this);

  std::string published_topic = "see_lidar_clustered_point_clouds";
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

  /*std::string advertised_service = "see_lidar_pointcloud_prefilter_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLidarPointcloudClusterer::serviceCallback, this);*/

  ROS_INFO("started node see_lidar_pointcloud_clusterer");

  auto all_successfull = loadConfiguration();
  if(!all_successfull) {
    ROS_INFO_STREAM("Could not read all parameters.");
  }
}

SeeLidarPointcloudClusterer::~SeeLidarPointcloudClusterer()
{
}

void SeeLidarPointcloudClusterer::run(uint64_t run_count)
{
  ROS_INFO("run SeeLidarPointcloudClusterer");
}

void SeeLidarPointcloudClusterer::dbscanClustering(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ROS_INFO("Pointcloud-Clusterer received a pointcloud-message");
  sensor_msgs::PointCloud2 output = *message;

  auto frame = output.header.frame_id;
  auto timestamp = output.header.stamp;

  CloudOS1 pointCloud;
  pcl::fromROSMsg(*message, pointCloud);

  std::vector<DBSCAN_Point> dbscan_points;
  std::vector<DBSCAN_Point*> neighborhood_points;
  std::vector<std::vector<DBSCAN_Point*>> clusters;

  DBSCAN_Point dbscan_point;
  for(unsigned int i = 0; i < pointCloud.size(); i++) {
    dbscan_point.os1_point = &pointCloud.at(i);
    dbscan_point.is_visited = false;
    dbscan_point.type = UNDEFINED;
    dbscan_point.is_in_cluster = false;

    dbscan_points.push_back(dbscan_point);
    //ROS_INFO_STREAM("Address DBSCAN point: " << &dbscan_point << ". Address in vector: " << &dbscan_points.at(i));
    //ROS_INFO_STREAM("Address OS1 point: " << &pointCloud.at(i) << ". Address in vector: " << &dbscan_points.at(i).os1_point);
  }

  //ROS_INFO_STREAM("Test 1");

  for(unsigned int i = 0; i < dbscan_points.size(); i++) {
    //ROS_INFO_STREAM("work on point");
    neighborhood_points.clear();
    DBSCAN_Point &point_a = dbscan_points.at(i);
    if(point_a.type != UNDEFINED) {
      //ROS_INFO_STREAM("already worked on point");
      continue;
    }

    //dbscan_points.at(i).is_visited = true;
    //ROS_INFO_STREAM("not worked on point, yet.");
    for(unsigned int j = 0; j < dbscan_points.size(); j++) {
      DBSCAN_Point &point_b = dbscan_points.at(j);
      if(getDistance(*point_a.os1_point, *point_b.os1_point) <= eps) {
        neighborhood_points.push_back(&point_b);
      }
    }

    if(neighborhood_points.size() < minPts) {
      point_a.type = NOISE;
    } else {
      /*
      old version: create cluster in function and return it

      auto cluster = expandCluster_copybackCluster(
        point_a,
        neighborhood_points,
        dbscan_points
      );
      */
      std::vector<SeeLidarPointcloudClusterer::DBSCAN_Point*> cluster;
      cluster.push_back(&point_a);
      point_a.type = CORE;

      expandCluster_Reference(
        point_a,
        neighborhood_points,
        cluster,
        dbscan_points
      );
      clusters.push_back(cluster);
    }
  }
  std::vector<sensor_msgs::PointCloud2> clusterMsg;
  ROS_INFO_STREAM("Anzahl Cluster: " << clusters.size());

  sensor_msgs::PointCloud2 msg;
  CloudOS1 pointCloudCluster;

  for(unsigned int i = 0; i < clusters.size(); i++) {
    auto &cluster = clusters.at(i);
    ROS_INFO_STREAM(i << ". Cluster: " << cluster.size() << " member.");
    //CloudOS1 pointCloudCluster;
    for(unsigned int j = 0; j < cluster.size(); j++) {
      pointCloudCluster.push_back(*cluster.at(j)->os1_point);
    }
    ouster_ros::OS1::PointOS1 dummy_point;
    dummy_point.x = 0;
    dummy_point.y = 0;
    dummy_point.z = 0;
    pointCloudCluster.push_back(dummy_point);
    //sensor_msgs::PointCloud2 msg;

    //pcl::toROSMsg(pointCloudCluster, msg);
    //msg.header.frame_id = frame;
    //msg.header.stamp = timestamp;

    //publisher.publish(msg);


    // to publish all clusters at the same time, a separate "msg" file is necessary which defines a vector
    // containing multiple PointCloud2 as a msg.
    //clusterMsg.push_back(msg);
  }
  pcl::toROSMsg(pointCloudCluster, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = timestamp;
  publisher.publish(msg);
}

void SeeLidarPointcloudClusterer::dbscanClustering_WithIndexReferences(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ROS_INFO("Pointcloud-Clusterer received a pointcloud-message");
  sensor_msgs::PointCloud2 output = *message;

  //auto frame = output.header.frame_id;
  //auto timestamp = output.header.stamp;

  CloudOS1 pointCloud;
  pcl::fromROSMsg(*message, pointCloud);

  auto core_points = std::vector<int>();
  auto near_points = std::vector<int>();
  auto noise_points = std::vector<int>();
  auto neighborhood_points = std::vector<int>();
  auto is_point_visited = std::vector<bool>(pointCloud.size(), false);
  auto cluster = std::vector<ouster_ros::OS1::PointOS1*>();
  auto clusters = std::vector<std::vector<ouster_ros::OS1::PointOS1*>>();

  core_points.clear();
  near_points.clear();
  noise_points.clear();
  neighborhood_points.clear();

  clusters.clear();

  ROS_INFO("Start");
  for(unsigned int i = 0; i < pointCloud.size(); i++) {
    //ROS_INFO("work on new point.");
    if(is_point_visited.at(i)) {
      continue;
    }
    is_point_visited.at(i) = true;

    neighborhood_points.clear();
    for(unsigned int j = 0; j < pointCloud.size(); j++) {
      if(getDistance(pointCloud.at(i), pointCloud.at(j)) <= eps) {
        neighborhood_points.push_back(j);

        //if(count_neighbors >= minPts) {
        //  core_points.push_back(i);

        //  break;
        //}

      }
    }
    if(neighborhood_points.size() < minPts) {
      // mark all points, which are not core points, as NOISE
      noise_points.push_back(i);
    } else {
      //ROS_INFO("create new cluster.");
      cluster.clear();
      cluster = expandCluster_WithIndexReference(
        pointCloud.at(i),
        neighborhood_points,
        cluster,
        clusters,
        pointCloud,
        is_point_visited
      );
      clusters.push_back(cluster);
      //ROS_INFO_STREAM("new cluster created: No. " << clusters.size());
    }
  }
  ROS_INFO("Pointcloud-Clusterer has finished.");
}

std::vector<ouster_ros::OS1::PointOS1*> SeeLidarPointcloudClusterer::expandCluster_WithIndexReference(
  ouster_ros::OS1::PointOS1& point,
  std::vector<int>& neighborhood_points,
  std::vector<ouster_ros::OS1::PointOS1*>& cluster,
  const std::vector<std::vector<ouster_ros::OS1::PointOS1*>>& clusters,
  CloudOS1& all_points,
  std::vector<bool>& is_point_visited
)
{
  cluster.push_back(&point);
  auto new_neighborhood_points = std::vector<int>();

  for(unsigned int i = 0; i < neighborhood_points.size(); i++) {
    new_neighborhood_points.clear();
    //ROS_INFO("create new neighborhood");

    if(!is_point_visited.at(neighborhood_points.at(i))) {
      is_point_visited.at(neighborhood_points.at(i)) = true;
      for(unsigned int j = 0; j < all_points.size(); j++) {
        if(getDistance(all_points.at(neighborhood_points.at(i)), all_points.at(j)) < eps) {
          new_neighborhood_points.push_back(j);
        }
      }
      //ROS_INFO("new neighborhood created");
      if(new_neighborhood_points.size() >= minPts) {
        for(unsigned int j = 0; j < new_neighborhood_points.size(); j++) {
          if(std::find(neighborhood_points.begin(), neighborhood_points.end(), new_neighborhood_points.at(j)) == neighborhood_points.end()) {
            // Punkt ist noch nicht in der Nachbarschaft vorhanden
            neighborhood_points.push_back(new_neighborhood_points.at(j));
            //ROS_INFO("neighborhood extended by 1 point.");
          }
        }

      }
    }
    if(!isClusterMember(neighborhood_points.at(i), clusters, all_points)) {
      cluster.push_back(&all_points.at(neighborhood_points.at(i)));
      //ROS_INFO("added point to cluster.");
      // unmark point as noise if necessary...
    }
  }
  ROS_INFO_STREAM("Clustersize: " << cluster.size());
  return cluster;
}

std::vector<SeeLidarPointcloudClusterer::DBSCAN_Point*> SeeLidarPointcloudClusterer::expandCluster_copybackCluster(
  DBSCAN_Point& point,
  std::vector<DBSCAN_Point*>& neighborhood_points,
  std::vector<DBSCAN_Point>& all_points
)
{
  std::vector<DBSCAN_Point*> cluster;
  point.type = CORE;
  cluster.push_back(&point);
  for(unsigned int i = 0; i < neighborhood_points.size(); i++) {
    //ROS_INFO_STREAM("Size neighborhood: " << neighborhood_points.size());

    if(*neighborhood_points.at(i) == point) {
      continue;
    }
    if(neighborhood_points.at(i)->type == NOISE) {
      neighborhood_points.at(i)->type = BORDER;
      //ROS_INFO_STREAM("Point is BORDER");
      cluster.push_back(neighborhood_points.at(i));
    }
    if(neighborhood_points.at(i)->type != UNDEFINED) {
      //ROS_INFO_STREAM("point is already categorized");
      continue;
    }

    //ROS_INFO_STREAM("Point " << i << " is set to CORE. Type before: " << (*neighborhood_points.at(i)).type);
    //ROS_INFO_STREAM("amaount in whole pointcloud: " << all_points.size());
    neighborhood_points.at(i)->type = CORE;

    cluster.push_back(neighborhood_points.at(i));
    /*
    auto p1 = neighborhood_points.at(i);
    auto p2 = cluster.at(cluster.size() - 1);
    ROS_INFO_STREAM("Adresses: " << &p1 << ", " << &p2);

    auto t1 = p1.type;
    auto t2 = p2.type;

    ROS_INFO_STREAM("Types: " << p1.type << ", " << p2.type);
    p1.type = UNDEFINED;
    ROS_INFO_STREAM("Types: " << p1.type << ", " << p2.type);

    p1.type = t1;
    p2.type = t2;
    */

    std::vector<DBSCAN_Point*> new_neighborhood_points;
    for(unsigned int j = 0; j < all_points.size(); j++) {
      if(getDistance(*all_points.at(j).os1_point, *neighborhood_points.at(i)->os1_point) <= eps) {
        new_neighborhood_points.push_back(&all_points.at(j));
      }
    }
    //ROS_INFO_STREAM("new neighborhood: " << new_neighborhood_points.size());
    if(new_neighborhood_points.size() >= minPts) {
      // alternative: use a "set" for all_points --> point can only be added once
      for(unsigned int k = 0; k < new_neighborhood_points.size(); k++) {

        /*
         * instead of using a own search-function, use the "native" search function from C++
        bool alreadyThere = false;
        for(unsigned int l = 0; l < neighborhood_points.size(); l++) {
          if(*neighborhood_points.at(l) == *new_neighborhood_points.at(k)) {
            alreadyThere = true;
            break;
          }
        }
        if(!alreadyThere) {
          //ROS_INFO_STREAM("Point added to neighborhood.");
          neighborhood_points.push_back(new_neighborhood_points.at(k));
        }
        */

        if(std::find(neighborhood_points.begin(),
                     neighborhood_points.end(),
                     new_neighborhood_points.at(k)) == neighborhood_points.end()) {
          // point is not in neighborhood, yet
          //ROS_INFO_STREAM("Point added to neighborhood.");
          neighborhood_points.push_back(new_neighborhood_points.at(k));
        }

      }
    }
  }
  return cluster;
}

void SeeLidarPointcloudClusterer::expandCluster_Reference(
  const DBSCAN_Point& point,
  std::vector<DBSCAN_Point*>& neighborhood_points,
  std::vector<DBSCAN_Point*>& cluster,
  std::vector<DBSCAN_Point>& all_points
)
{
  double left_edge = point.os1_point->y;
  double right_edge = point.os1_point->y;
  double top_edge = point.os1_point->z;
  double bottom_edge = point.os1_point->z;
  double front_edge = point.os1_point->x;
  double back_edge = point.os1_point->x;

  for(unsigned int i = 0; i < neighborhood_points.size(); i++) {
    //ROS_INFO_STREAM("Size neighborhood: " << neighborhood_points.size());
    if(*neighborhood_points.at(i) == point) {
      continue;
    }

    if(neighborhood_points.at(i)->type == NOISE) {
      neighborhood_points.at(i)->type = BORDER;
      //ROS_INFO_STREAM("Point is BORDER");
      cluster.push_back(neighborhood_points.at(i));

      // check bounds
      updateClusterBounds(
        *neighborhood_points.at(i)->os1_point,
        left_edge,
        right_edge,
        top_edge,
        bottom_edge,
        front_edge,
        back_edge
      );
    }
    if(neighborhood_points.at(i)->type != UNDEFINED) {
      //ROS_INFO_STREAM("Point is already categorized");
      continue;
    }

    //ROS_INFO_STREAM("Point " << i << " is set to CORE. Typ davor: " << (*neighborhood_points.at(i)).type);
    //ROS_INFO_STREAM("Amount in whole pointcloud: " << all_points.size());
    neighborhood_points.at(i)->type = CORE;

    cluster.push_back(neighborhood_points.at(i));

    // check bounds
    updateClusterBounds(
      *neighborhood_points.at(i)->os1_point,
      left_edge,
      right_edge,
      top_edge,
      bottom_edge,
      front_edge,
      back_edge
    );


    std::vector<DBSCAN_Point*> new_neighborhood_points;
    for(unsigned int j = 0; j < all_points.size(); j++) {
      if(getDistance(*all_points.at(j).os1_point, *neighborhood_points.at(i)->os1_point) <= eps) {
        new_neighborhood_points.push_back(&all_points.at(j));
      }
    }
    //ROS_INFO_STREAM("new neighborhood: " << new_neighborhood_points.size());
    if(new_neighborhood_points.size() >= minPts) {
      // use a "set" for all_points --> point can only be added once
      for(unsigned int k = 0; k < new_neighborhood_points.size(); k++) {

        if(std::find(neighborhood_points.begin(),
                     neighborhood_points.end(),
                     new_neighborhood_points.at(k)) == neighborhood_points.end()) {
          // point is not in neighborhood, yet
          //ROS_INFO_STREAM("Point is added to neighborhood.");
          neighborhood_points.push_back(new_neighborhood_points.at(k));
        }

      }
    }
  }

  // print the size of the cluster
  double width = right_edge - left_edge;
  double height = top_edge - bottom_edge;
  double depth = back_edge - front_edge;
  ROS_INFO_STREAM("Width: " << width << ", Height: " << height << ", Depth: " << depth);
}


bool SeeLidarPointcloudClusterer::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_lidar_processing service response";
  return true;
}

double SeeLidarPointcloudClusterer::getDistance(
  const ouster_ros::OS1::PointOS1& point_1,
  const ouster_ros::OS1::PointOS1& point_2)
{
  return sqrt((point_1.x - point_2.x) * (point_1.x - point_2.x) +
              (point_1.y - point_2.y) * (point_1.y - point_2.y) +
              (point_1.z - point_2.z) * (point_1.z - point_2.z)
  );
}

bool SeeLidarPointcloudClusterer::isClusterMember(
  const int& point_idx,
  const std::vector<std::vector<ouster_ros::OS1::PointOS1*>>& clusters,
  const CloudOS1& complete_point_cloud
)
{
  //TODO: "is_member" could be stored in a vector (similiar to "is_visited") -> faster access
  bool is_member = false;
  for(unsigned int i = 0; i < clusters.size(); i++) {
    auto cluster = clusters.at(i);
    for(unsigned int j = 0; j < cluster.size(); j++) {
      auto point = cluster.at(j);
      if(point->x == complete_point_cloud.at(point_idx).x &&
         point->y == complete_point_cloud.at(point_idx).y &&
         point->z == complete_point_cloud.at(point_idx).z) {
        is_member = true;
        break;
      }
    }
    if(is_member) {
      break;
    }
  }
  return is_member;
}

bool SeeLidarPointcloudClusterer::isSamePoint(
  const SeeLidarPointcloudClusterer::DBSCAN_Point& point_1,
  const SeeLidarPointcloudClusterer::DBSCAN_Point& point_2
)
{
  //ROS_INFO_STREAM("check if points are the same");
  if (point_1.os1_point->x == point_2.os1_point->x &&
      point_1.os1_point->y == point_2.os1_point->y &&
      point_1.os1_point->z == point_2.os1_point->z) {
    return true;
  } else {
    return false;
  }
}

void SeeLidarPointcloudClusterer::updateClusterBounds(
  const ouster_ros::OS1::PointOS1& point,
  double& left_edge,
  double& right_edge,
  double& top_edge,
  double& bottom_edge,
  double& front_edge,
  double& back_edge
)
{
  if(point.y < left_edge) {
    left_edge = point.y;
  } else if(point.y > right_edge) {
     right_edge = point.y;
  }

  if(point.z < bottom_edge) {
    bottom_edge = point.z;
  } else if(point.z > top_edge) {
    top_edge = point.z;
  }

   if(point.x < front_edge) {
    front_edge = point.x;
  } else if(point.x > back_edge) {
    back_edge = point.x;
  }
}

bool SeeLidarPointcloudClusterer::loadConfiguration()
{
  // default values
  minPts = 3;
  eps = 0.2;
  multithreading = true;

  // keys to get the configuration parameters
  std::string clusterer_key = "clusterer";
  std::string eps_key = "eps";
  std::string minPts_key = "minPts";
  std::string multithreading_enabled_key = "multithreading";

  // variable to indicate if all parameters could be loaded.
  bool all_successfull = true;

  XmlRpc::XmlRpcValue clusterer_config;

  // load configuration from parameter server
  // to edit the parameters in the configuration file, have a look at the launch-file.
  // There is the file path to the configuration file specified
  if(!nodeHandle.getParam(clusterer_key, clusterer_config)) {
    ROS_INFO_STREAM("Could not read config for Clusterer");
    ROS_INFO_STREAM("Kept all default values");
    all_successfull = false;
  } else {
    ROS_ASSERT(clusterer_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Read parameter \"" << clusterer_key << "\".");
    if(clusterer_config.hasMember(minPts_key)) {
      minPts = (int) clusterer_config[minPts_key];
    } else {
      ROS_INFO_STREAM(minPts_key << " not defined. Kept the default value.");
      all_successfull = false;
    }
    if(clusterer_config.hasMember(eps_key)) {
      eps = (double) clusterer_config[eps_key];
    } else {
      ROS_INFO_STREAM(eps_key << " not defined. Kept the default value.");
      all_successfull = false;
    }
    if(clusterer_config.hasMember(multithreading_enabled_key)) {
      multithreading = clusterer_config[multithreading_enabled_key];
    } else {
      ROS_INFO_STREAM(multithreading_enabled_key << " not defined. Kept the default value");
      all_successfull = false;
    }
  }
  ROS_INFO_STREAM("Configured Clusterer...\n\t" <<
    eps_key << ": " << eps << "\n\t" << 
    minPts_key << ": " << minPts << "\n\t" << 
    multithreading_enabled_key << ": " << multithreading
  );

  // Test, how to get all parameters without manually accessing it.
  // But because of the difference of some parameters, this is not good.
  //std::vector<std::string> clusterer_keys;
  //clusterer_keys.push_back("minPts");
  //clusterer_keys.push_back("eps");
  
  // attention: not all values are doubles
  /*
  std::map<std::string, double*> clusterer_config_values;
  // fill map
  clusterer_config_values["minPts"] = &minPts;
  clusterer_config_values["eps"] = &eps;
  // iterate through clusterer_keys and select for each value in clusterer_config_values the variable from the parameter server
  std::map<std::string, double*>::iterator it;
  for(it = clusterer_config_values.begin(); it != clusterer_config_values.end(); it++) {
    if(clusterer_config.hasMember(it->first)) {
      *(it->second) = (double) clusterer_config[it->first];
    } else {
      ROS_INFO_STREAM(it->first << " not defined. Kept the default value.");
    }
  }
  */

  return all_successfull;
}

bool SeeLidarPointcloudClusterer::isMultithreadingEnabled() {
  return multithreading;
}


const bool SeeLidarPointcloudClusterer::DBSCAN_Point::operator==(const DBSCAN_Point& rhs) const
{
  //ROS_INFO_STREAM("check if points are the same");
  if(this->os1_point->x == rhs.os1_point->x &&
     this->os1_point->y == rhs.os1_point->y &&
     this->os1_point->z == rhs.os1_point->z &&
     this->is_visited == rhs.is_visited &&
     this->type == rhs.type &&
     this->is_in_cluster == rhs.is_in_cluster) {
    //ROS_INFO_STREAM("points are same");
    /*ROS_INFO_STREAM(this->os1_point.x << ", " <<
      this->os1_point.y << ", " <<
      this->os1_point.z << ", " <<
      this->is_visited << ", " <<
      this->type << ", " <<
      this->is_in_cluster);
    ROS_INFO_STREAM(rhs.os1_point.x << ", " <<
      rhs.os1_point.y << ", " <<
      rhs.os1_point.z << ", " <<
      rhs.is_visited << ", " <<
      rhs.type << ", " <<
      rhs.is_in_cluster);*/
    return true;
  } else {
    //ROS_INFO_STREAM("points are not same");
    /*ROS_INFO_STREAM(this->os1_point.x << ", " <<
      this->os1_point.y << ", " <<
      this->os1_point.z << ", " <<
      this->is_visited << ", " <<
      this->type << ", " <<
      this->is_in_cluster);
    ROS_INFO_STREAM(rhs.os1_point.x << ", " <<
      rhs.os1_point.y << ", " <<
      rhs.os1_point.z << ", " <<
      rhs.is_visited << ", " <<
      rhs.type << ", " <<
      rhs.is_in_cluster);*/
    return false;
  }
}
