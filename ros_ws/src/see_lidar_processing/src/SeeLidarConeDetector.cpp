#define _USE_MATH_DEFINES

#include "see_lidar_processing/SeeLidarConeDetector.hpp"

#include <ouster_ros/point_os1.h>
#include "see_lidar_processing/ConeContainer.h"
#include "see_lidar_processing/Cone.h"
#include "see_lidar_processing/Coordinates.h"
#include "see_lidar_processing/CovarianceMatrix.h"
#include "see_lidar_processing/ConeColor.h"
#include <string>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <cmath>
#include <boost/thread.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_lidar_cone_detector");
  ros::NodeHandle nodeHandle("see_lidar");

  SeeLidarConeDetector coneDetector(nodeHandle);

  // check for multithreading
  if(coneDetector.isMultithreadingEnabled()) {
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown();
  } else {
    ros::spin();    
  }
  
  return 0;
}

SeeLidarConeDetector::SeeLidarConeDetector(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "see_lidar_clustered_point_clouds";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLidarConeDetector::detectCones, this);

  std::string published_topic = "see_lidar_cone_point_clouds";
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

  std::string published_topic2 = "see_lidar_cones_center";
  publisher2 = nodeHandle.advertise<sensor_msgs::PointCloud2>(published_topic2, 1);
  
  std::string published_topic3 = "see_lidar_cone_container_test";
  cones_publisher = nodeHandle.advertise<see_lidar_processing::ConeContainer>(published_topic3, 1);

  std::string transform_topic = "transformed_cloud";
  transform_publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(transform_topic, 1);

  /*std::string advertised_service = "see_lidar_pointcloud_prefilter_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLidarConeDetector::serviceCallback, this);*/

  ROS_INFO("started node see_lidar_cone_detector");

  auto all_successfull = loadConfiguration();
  if(!all_successfull) {
    ROS_INFO_STREAM("Could not read all parameters.");
  }
}

SeeLidarConeDetector::~SeeLidarConeDetector()
{
}

void SeeLidarConeDetector::detectCones(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ROS_INFO_STREAM("ConeDetector received a message");

  auto frame = message->header.frame_id;
  auto timestamp = message->header.stamp;

  CloudOS1 pointCloud;
  pcl::fromROSMsg(*message, pointCloud);
  //ROS_INFO_STREAM(frame << ", " << timestamp);

  if(pointCloud.size() == 0) {
    // error: something went wrong/ missing Points
    // discard this message
    return;
  }

  auto clusters = getClusters(pointCloud);
  std::vector<CloudOS1> cones;
  std::vector<CloudOS1> conesCenterVisualization;
  
  see_lidar_processing::ConeContainer lidar_cones_message;

  std::vector<CloudOS1> transformed_cones;
  
  for(uint64_t i = 0; i < clusters.size(); i++) {
    auto& cluster = clusters.at(i);
    double cone_probability = getConeProbability(cluster);
    //ROS_INFO_STREAM("Cone probability: " << cone_probability * 100 << "%");
    if(cone_probability > 0) {
      cones.push_back(cluster);
      CloudOS1 transformed_cone;
      if(use_pcl) {
        transformed_cone = transformCloudUsingPcl(cluster);
      } else {
        transformed_cone = transformCloud(cluster);      }
      transformed_cones.push_back(transformed_cone);
      auto cone_center = getClusterCenter(transformed_cone);

      see_lidar_processing::Cone cone;
      std::vector<see_lidar_processing::ConeColor> cone_color = getConeColor(cluster);

      cone.coordinates = cone_center;
      cone.cone_colors = cone_color;
      cone.cone_probability = cone_probability;
      
      lidar_cones_message.cones.push_back(cone);
      
      //ROS_INFO_STREAM("Center: x: " << cone_center.x << ", y: " << cone_center.y);
      CloudOS1 coneCenter = createPositionVisualization(
        cone_center, 
        cone_probability * 100 * 650
        // set the range to the full intensity
      );
      
      conesCenterVisualization.push_back(coneCenter);
    }
  }
  
  lidar_cones_message.timestamp = timestamp;
  lidar_cones_message.count = lidar_cones_message.cones.size();
  
  CloudOS1 centerPointCloud;
  for(uint64_t i = 0; i < conesCenterVisualization.size(); i++) {
    centerPointCloud += conesCenterVisualization.at(i);
  }
  CloudOS1 conesPointCloud;
  for(uint64_t i = 0; i < cones.size(); i++) {
    conesPointCloud += cones.at(i);
  }
  CloudOS1 transformed_cones_pointcloud;
  for(uint64_t i = 0; i < transformed_cones.size(); i++) {
    transformed_cones_pointcloud += transformed_cones.at(i);
  }

  publishPointcloud(
    conesPointCloud,
    publisher,
    frame,
    timestamp
  );

  publishPointcloud(
    transformed_cones_pointcloud,
    transform_publisher,
    frame,
    timestamp
  );

  // create legend for probability visualization
  CloudOS1 legend = createLegendForVisualization(10);

  centerPointCloud += conesPointCloud;
  centerPointCloud += legend;
  publishPointcloud(
    centerPointCloud,
    publisher2,
    frame,
    timestamp
  );
  
  cones_publisher.publish(lidar_cones_message);
}

std::vector<CloudOS1> SeeLidarConeDetector::getClusters(const CloudOS1& pointCloud)
{
  std::vector<CloudOS1> clusters;
  CloudOS1 cluster;
  clusters.push_back(cluster);
  //ROS_INFO_STREAM("Address cluster: " << &cluster << ", " << &clusters.at(0));
  for(uint64_t i = 0; i < pointCloud.size() - 1; i++) {
    // pointCloud.size() "- 1" is necessary, because the last point is currently a dummy point (x,y,z = 0)
    const auto& current_point = pointCloud.at(i);
    if(current_point.x == 0 &&
       current_point.y == 0 &&
       current_point.z == 0) {
      CloudOS1 new_cluster;
      clusters.push_back(new_cluster);
      //ROS_INFO_STREAM("Address new_cluster: " << &new_cluster << ", " << &clusters.at(clusters.size() - 1));
    } else {
      auto& last_cluster = clusters.back();
      last_cluster.push_back(current_point);
    }
  }

  //ROS_INFO_STREAM("Amount of clusters: " << clusters.size());
  return clusters;
}

double SeeLidarConeDetector::getDistance2D(const ouster_ros::OS1::PointOS1& point)
{
  return sqrt(point.x * point.x + point.y * point.y);
}

void SeeLidarConeDetector::getClusterDimensions(
  const CloudOS1& cluster,
  double& width,
  double& depth,
  double& height,
  double& nearest_distance
)
{
  // initialize bounds
  double front_bound = cluster.at(0).x;
  double back_bound = cluster.at(0).x;
  double left_bound = cluster.at(0).y;
  double right_bound = cluster.at(0).y;
  double top_bound = cluster.at(0).z;
  double bottom_bound = cluster.at(0).z;
  nearest_distance = getDistance2D(cluster.at(0));

  // get real bounds
  for(uint64_t i = 1; i < cluster.size(); i++) {
    auto& point = cluster.at(i);
    if(point.x < front_bound) front_bound = point.x;
    if(point.x > back_bound) back_bound = point.x;
    // is this "direction" correct?
    if(point.y > left_bound) left_bound = point.y;
    if(point.y < right_bound) right_bound = point.y;
    if(point.z > top_bound) top_bound = point.z;
    if(point.z < bottom_bound) bottom_bound = point.z;
    auto current_distance = getDistance2D(point);
    if(current_distance < nearest_distance) {
      nearest_distance = current_distance;
    }
  }

  width = fabs(right_bound - left_bound);
  depth = fabs(front_bound - back_bound);
  height = fabs(top_bound - bottom_bound);
}

double SeeLidarConeDetector::getConeProbability(CloudOS1& cluster)
{
  // get cluster dimensions
  double distance;
  double width, height, depth;
  getClusterDimensions(cluster, width, depth, height, distance);
  //ROS_INFO_STREAM("Width: " << width << ", Depth: " << depth << ", Height: " << height);

  double vertical_distance_per_meter = 9.075;
  double horizontal_distance_per_meter = 0.314;
  double reduction_per_mm = 0.036;
  // max width of cone without base
  double cone_width_bottom = 14.6;
  // max width of cone on top
  double cone_width_top = 4.6;

  double max_height = height * 1000 + (vertical_distance_per_meter * distance * 0.95);

  // This is unnecessary?
  cone_width_bottom = cone_width_top + max_height * reduction_per_mm;

  // What does this?
  double current_height = 0;
  double height_counter = 0;
  double amount_points = 0;
  while(current_height < height) {
    current_height = height_counter * (vertical_distance_per_meter * distance);
    amount_points += (cone_width_bottom - (current_height * reduction_per_mm)) / (horizontal_distance_per_meter * distance);
    height_counter++;
  }

  // why are the variables called like this?
  double height_percentage_up = getHeightProbabilityFactor(height);

  // why is xx_percentage_up == 0.5 in the last case?
  double width_percentage_up = getWidthProbabilityFactor(width);

  // why is xx_percentage_up == 0.5 in the last case?
  double depth_percentage_up = getDepthProbabilityFactor(depth);

  //ROS_INFO_STREAM("Height Percentage up: " << height_percentage_up <<
  //  ", Width Percentage up: " << width_percentage_up <<
  //  ", Depth Percentage up: " << depth_percentage_up);

  double cone_probability = 0;
  if(height >= 0.03 &&
     height <= 0.6 &&
     width <= 0.3 &&
     depth <= 0.3 &&
     amount_points * 2 >= cluster.size()) {
    // amount_points * 2 >= cluster.size() --> test, if there are not too much points in this cluster (if there are too much, this can't be a cone)
    //ROS_INFO_STREAM("Setting cone probability");
    // xx_precentage_down is ignored
    cone_probability = (height_percentage_up + width_percentage_up + depth_percentage_up) / 30.0;
  }
  //ROS_INFO_STREAM("Cluster is cone: " << cone_probability * 100 << "%");
  

  return cone_probability;
}

see_lidar_processing::Coordinates SeeLidarConeDetector::getClusterCenter(const CloudOS1& cluster)
{
  double width = 0;
  double depth = 0;

  for(uint64_t i = 0; i < cluster.size(); i++) {
    const auto& current_point = cluster.at(i);
    width += current_point.y;
    depth += current_point.x;
  }

  see_lidar_processing::Coordinates center;
  center.x = depth / cluster.size();
  center.y = width / cluster.size();

  return center;
}

double SeeLidarConeDetector::getHeightProbabilityFactor(double height)
{
  double max_points = 10;
  double height_factor = 0;
  if(height <= small_cone_height_by_rules) {
    height_factor = (max_points / small_cone_height_by_rules) * height;
  } else if (height <= big_cone_height_by_rules) {
    height_factor = (max_points / big_cone_height_by_rules) * height;
  } else {
    height_factor = 0;
  }
  return height_factor;
}

double SeeLidarConeDetector::getWidthProbabilityFactor(double width)
{
  double max_points = 10;
  double width_factor = 0;
  if(width < small_cone_width_by_rules) {
    width_factor = (max_points / small_cone_width_by_rules) * width;
  } else if(width <= big_cone_width_by_rules) {
    width_factor = (max_points / big_cone_width_by_rules) * width;
  } else {
    width_factor = 0;
  }
  return width_factor;
}

double SeeLidarConeDetector::getDepthProbabilityFactor(double depth)
{
  double max_points = 10;
  double depth_factor = 0;
  if(depth < small_cone_depth_by_rules) {
    depth_factor = (max_points / small_cone_depth_by_rules) * depth;
  } else if(depth <= big_cone_depth_by_rules) {
    depth_factor = (max_points / big_cone_depth_by_rules) * depth;
  } else {
    depth_factor = 0;
  }
  return depth_factor;
}

CloudOS1 SeeLidarConeDetector::transformCloud(const CloudOS1& cloud)
{
  CloudOS1 transformed_cloud;
  // get the angles in radian (defined in the config)
  // change the sign (- --> +, + --> -), because we want to "turn back" the
  // back the coordinate system of the lidar to the "global" coordinate system (of the car)
  auto roll_radian_angle = -(lidar_roll / 360) * (2 * M_PI);
  auto pitch_radian_angle = -(lidar_pitch / 360) * (2 * M_PI);
  auto yaw_radian_angle = -(lidar_yaw / 360) * (2 * M_PI);

  for(uint64_t i = 0; i < cloud.size(); i++) {
    auto point = cloud.at(i);
    // do rotation
    point.x = (cos(pitch_radian_angle) * cos(yaw_radian_angle)) * point.x +
              (cos(pitch_radian_angle) * sin(yaw_radian_angle)) * point.y +
              (-sin(pitch_radian_angle)) * point.z;
    point.y = (sin(roll_radian_angle) * sin(pitch_radian_angle) * cos(yaw_radian_angle) - cos(roll_radian_angle) * sin(yaw_radian_angle)) * point.x +
              (sin(roll_radian_angle) * sin(pitch_radian_angle) * sin(yaw_radian_angle) + cos(roll_radian_angle) * cos(yaw_radian_angle)) * point.y +
              (sin(roll_radian_angle) * cos(pitch_radian_angle)) * point.z;
    point.z = (cos(roll_radian_angle) * sin(pitch_radian_angle) * cos(yaw_radian_angle) + sin(yaw_radian_angle) * sin(roll_radian_angle)) * point.x +
              (cos(roll_radian_angle) * sin(pitch_radian_angle) * sin(yaw_radian_angle) - sin(roll_radian_angle) * cos(yaw_radian_angle)) * point.y +
              (cos(roll_radian_angle) * cos(pitch_radian_angle)) * point.z;
    // do translation
    // here is no sign change necessary
    point.x += lidar_x_offset;
    point.y += lidar_y_offset;
    transformed_cloud.push_back(point);
  }

  return transformed_cloud;
}

CloudOS1 SeeLidarConeDetector::transformCloudUsingPcl(const CloudOS1& cloud)
{
  CloudOS1 transformed_cloud;
  // create a special rotation matrix
  // a "normal" rotation matrix has 3 ros & 3 columns
  // this matrix has 4 rows & 4 columns
  // the first 3 rows & the first 3 columns are used as the "normal" rotation matrix
  // the 4th column is used for the translation
  // the 4th row is not used
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  // get the angles in radian (defined in the config)
  // change the sign (- --> +, + --> -), because we want to "turn back" the
  // back the coordinate system of the lidar to the "global" coordinate system (of the car)
  auto roll_radian_angle = -(lidar_roll / 360) * (2 * M_PI);
  auto pitch_radian_angle = -(lidar_pitch / 360) * (2 * M_PI);
  auto yaw_radian_angle = -(lidar_yaw / 360) * (2 * M_PI);

  // define the rotation of the coordinates
  transform(0,0) = cos(pitch_radian_angle) * cos(yaw_radian_angle);
  transform(0,1) = cos(pitch_radian_angle) * sin(yaw_radian_angle);
  transform(0,2) = -sin(pitch_radian_angle);
  transform(1,0) = sin(roll_radian_angle) * sin(pitch_radian_angle) * cos(yaw_radian_angle) - cos(roll_radian_angle) * sin(yaw_radian_angle);
  transform(1,1) = sin(roll_radian_angle) * sin(pitch_radian_angle) * sin(yaw_radian_angle) + cos(roll_radian_angle) * cos(yaw_radian_angle);
  transform(1,2) = sin(roll_radian_angle) * cos(pitch_radian_angle);
  transform(2,0) = cos(roll_radian_angle) * sin(pitch_radian_angle) * cos(yaw_radian_angle) + sin(yaw_radian_angle) * sin(roll_radian_angle);
  transform(2,1) = cos(roll_radian_angle) * sin(pitch_radian_angle) * sin(yaw_radian_angle) - sin(roll_radian_angle) * cos(yaw_radian_angle);
  transform(2,2) = cos(roll_radian_angle) * cos(pitch_radian_angle);

  // define the translation of the coordinates
  // here is no sign change necessary
  transform(0,3) = lidar_x_offset;
  transform(1,3) = lidar_y_offset;

  pcl::transformPointCloud(cloud, transformed_cloud, transform);

  return transformed_cloud;
}

std::vector<see_lidar_processing::ConeColor> SeeLidarConeDetector::getConeColor(const CloudOS1& cone)
{
  std::vector<see_lidar_processing::ConeColor> cone_colors;
  see_lidar_processing::ConeColor cone_color;

  cone_color.color = cone_color.CONE_COLOR_UNKNOWN;
  cone_color.probability = 1.0;

  cone_colors.push_back(cone_color);

  return cone_colors;
}

bool SeeLidarConeDetector::loadConfiguration()
{
  // TODO: needs rework
  // Detector cone size threshold is not used

  // keys to get the configuration parameters
  std::string lidar_key = "lidar";
  std::string lidar_x_offset_key = "x_offset";
  std::string lidar_y_offset_key = "y_offset";
  std::string lidar_pitch_key = "pitch";
  std::string lidar_roll_key = "roll";
  std::string lidar_yaw_key = "yaw";

  std::string detector_key = "detector";
  std::string small_cone_height_key = "small_cone_height";
  std::string small_cone_width_key = "small_cone_width";
  std::string small_cone_depth_key = "small_cone_depth";
  std::string big_cone_height_key = "big_cone_height";
  std::string big_cone_width_key = "big_cone_width";
  std::string big_cone_depth_key = "big_cone_depth";
  std::string cone_size_threshold_key = "cone_threshold";
  std::string use_pcl_key = "use_pcl";
  std::string multithreading_enabled_key = "multithreading";

  // variable to indicate if all parameters could be loaded.
  bool all_successfull = true;

  XmlRpc::XmlRpcValue lidar_config;
  XmlRpc::XmlRpcValue detector_config;

  // load lidar mounting configuration from the parameter server
  // to edit the parameters in the configuration file, have a look at the launch-file.
  // There is the file path to the configuration file specified
  if(!nodeHandle.getParam(lidar_key, lidar_config)) {
    ROS_INFO_STREAM("Could not read configuration for lidar mounting position.");
    ROS_INFO_STREAM("Kept all default values");
    all_successfull = false;
  } else {
    ROS_ASSERT(lidar_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Read parameter \"" << lidar_key << "\".");
    if(lidar_config.hasMember(lidar_x_offset_key)) {
      lidar_x_offset = (double) lidar_config[lidar_x_offset_key];
    } else {
      ROS_INFO_STREAM(lidar_x_offset_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(lidar_config.hasMember(lidar_y_offset_key)) {
      lidar_y_offset = (double) lidar_config[lidar_y_offset_key];
    } else {
      ROS_INFO_STREAM(lidar_y_offset_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(lidar_config.hasMember(lidar_pitch_key)) {
      lidar_pitch = (double) lidar_config[lidar_pitch_key];
    } else {
      ROS_INFO_STREAM(lidar_pitch_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(lidar_config.hasMember(lidar_roll_key)) {
      lidar_roll = (double) lidar_config[lidar_roll_key];
    } else {
      ROS_INFO_STREAM(lidar_roll_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(lidar_config.hasMember(lidar_yaw_key)) {
      lidar_yaw = (double) lidar_config[lidar_yaw_key];
    } else {
      ROS_INFO_STREAM(lidar_yaw_key << " not defined. Kept the default value");
      all_successfull = false;
    }
  }
  ROS_INFO_STREAM("Configured Lidar mounting position...\n\t" <<
    lidar_x_offset_key << ": " << lidar_x_offset << "\n\t" <<
    lidar_x_offset_key << ": " << lidar_y_offset << "\n\t" <<
    lidar_pitch_key << ": " << lidar_pitch << "\n\t" <<
    lidar_roll_key << ": " << lidar_roll << "\n\t" <<
    lidar_yaw_key << ": " << lidar_yaw
  );

  if(!nodeHandle.getParam(detector_key, detector_config)) {
    ROS_INFO_STREAM("Could not read configuration for Cone Detector.");
    ROS_INFO_STREAM("Kept all default values");
    all_successfull = false;
  } else {
    ROS_ASSERT(detector_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_INFO_STREAM("Read parameter \"" << detector_key << "\".");
    if(detector_config.hasMember(small_cone_height_key)) {
      small_cone_height_by_rules = (double) detector_config[small_cone_height_key];
    } else {
      ROS_INFO_STREAM(small_cone_height_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(small_cone_width_key)) {
      small_cone_width_by_rules = (double) detector_config[small_cone_width_key];
    } else {
      ROS_INFO_STREAM(small_cone_width_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(small_cone_depth_key)) {
      small_cone_depth_by_rules = (double) detector_config[small_cone_depth_key];
    } else {
      ROS_INFO_STREAM(small_cone_depth_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(big_cone_height_key)) {
      big_cone_height_by_rules = (double) detector_config[big_cone_height_key];
    } else {
      ROS_INFO_STREAM(big_cone_height_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(big_cone_width_key)) {
      big_cone_width_by_rules = (double) detector_config[big_cone_width_key];
    } else {
      ROS_INFO_STREAM(big_cone_width_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(big_cone_depth_key)) {
      big_cone_depth_by_rules = (double) detector_config[big_cone_depth_key];
    } else {
      ROS_INFO_STREAM(big_cone_depth_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(cone_size_threshold_key)) {
      cone_size_threshold = (double) detector_config[cone_size_threshold_key];
    } else {
      ROS_INFO_STREAM(cone_size_threshold_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(multithreading_enabled_key)) {
      multithreading = detector_config[multithreading_enabled_key];
    } else {
      ROS_INFO_STREAM(multithreading_enabled_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(detector_config.hasMember(use_pcl_key)) {
      use_pcl = (bool) detector_config[use_pcl_key];
    } else {
      ROS_INFO_STREAM(use_pcl_key << " not defined. Kept the default value");
      all_successfull = false;
    }
  }
  ROS_INFO_STREAM("Configured detector parameters...\n\t" <<
    small_cone_height_key << ": " << small_cone_height_by_rules << "\n\t" <<
    small_cone_width_key << ": " << small_cone_width_by_rules << "\n\t" <<
    small_cone_depth_key << ": " << small_cone_depth_by_rules << "\n\t" <<
    big_cone_height_key << ": " << big_cone_height_by_rules << "\n\t" <<
    big_cone_width_key << ": " << big_cone_width_by_rules << "\n\t" <<
    big_cone_depth_key << ": " << big_cone_depth_by_rules << "\n\t" <<
    cone_size_threshold_key << ": " << cone_size_threshold << "\n\t" <<
    multithreading_enabled_key << ": " << multithreading << "\n\t" <<
    use_pcl_key << ": " << use_pcl
  );

  return all_successfull;
}

bool SeeLidarConeDetector::isMultithreadingEnabled()
{
  return multithreading;
}

CloudOS1 SeeLidarConeDetector::createPositionVisualization(
  const see_lidar_processing::Coordinates& position,
  uint16_t intensity
)
{
  CloudOS1 visualization;
  for(uint64_t i = 0; i < 50; i++) {
    ouster_ros::OS1::PointOS1 p;
    p.x = position.x;
    p.y = position.y;
    p.z = -1.5 + i / 25.0;
    p.intensity = intensity;
    visualization.push_back(p);
  }
  return visualization;


  // For use with an edited ouster driver
  //CloudOS1 legende;
  //legende.width = 100;
  //legende.height = 1;
  //legende.points.resize(legende.width * legende.height); 
  //for(unsigned int o = 0; o < legende.points.size(); o++){
  //          legende.points[o].x = -5.5;
  //          legende.points[o].y = +5.0-o/(legende.width/10.0);
  //          legende.points[o].z = 0;
  //          legende.points[o].t = 0;
  //          legende.points[o].intensity = 512;
  //          legende.points[o].reflectivity = 512;
  //          legende.points[o].cone = o/100.0;
  //          legende.points[o].ring = 1;
  //      }
  //
  //conesPointCloud += legende;
}

CloudOS1 SeeLidarConeDetector::createLegendForVisualization(double width)
{
  CloudOS1 legend;
  for(uint16_t i = 0; i < 65000; i += 100) {
    ouster_ros::OS1::PointOS1 point;
    point.x = -5.5;
    point.y = (width / 2) - (i / 65000.0) * width;
    point.z = 0;
    point.t = 0;
    point.intensity = i;
    point.reflectivity = 0;
    point.ring = 0;
    legend.push_back(point);
  }
  return legend;
}

void SeeLidarConeDetector::publishPointcloud(
  const CloudOS1& cloud,
  const ros::Publisher& publisher,
  const std::string& frame_id,
  const ros::Time& stamp
)
{
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  publisher.publish(msg);
}

void SeeLidarConeDetector::run(uint64_t run_count)
{  
  ROS_INFO("run SeeLidarConeDetector");
}


void SeeLidarConeDetector::topicCallback(const std_msgs::String::ConstPtr& message)
{
  ROS_INFO("ConeDetector received a message: [%s]", message->data.c_str());
  std_msgs::String msg;
  msg.data = "message from SeeLidarConeDetector";
  publisher.publish(msg);
}

bool SeeLidarConeDetector::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_lidar_processing service response";
  return true;
}

