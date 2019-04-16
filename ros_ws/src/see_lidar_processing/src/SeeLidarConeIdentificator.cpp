#include "see_lidar_processing/SeeLidarConeIdentificator.hpp"

#include "see_lidar_processing/ConeContainer.h"
#include "see_lidar_processing/Cone.h"
#include "see_lidar_processing/Coordinates.h"
#include "see_lidar_processing/CovarianceMatrix.h"
#include "see_lidar_processing/ConeColor.h"
#include <string>
#include <XmlRpcValue.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_lidar_cone_identificator");
  ros::NodeHandle nodeHandle("see_lidar");

  SeeLidarConeIdentificator coneIdentificator(nodeHandle);

  ros::spin();
  return 0;
}

SeeLidarConeIdentificator::SeeLidarConeIdentificator(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "see_lidar_cone_point_clouds";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLidarConeIdentificator::topicCallback, this);

  std::string published_topic = "see_lidar_cone_container";
  publisher = nodeHandle.advertise<see_lidar_processing::ConeContainer>(published_topic, 1);

  /*std::string advertised_service = "see_lidar_pointcloud_prefilter_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLidarConeIdentificator::serviceCallback, this);*/
  
  run_count = 0;

  ROS_INFO("started node see_lidar_cone_identificator");

  std::string clusterer_config_test;

  if (!nodeHandle.getParam("test", clusterer_config_test)) {
    ROS_INFO_STREAM("Konnte Variable nicht lesen");
  } else {
    ROS_INFO_STREAM("Variable \"clusterer_config_test\" eingelesen.");
    ROS_INFO_STREAM("clusterer_config_test hat Wert: " << clusterer_config_test);
  }
  
}

SeeLidarConeIdentificator::~SeeLidarConeIdentificator()
{
}

void SeeLidarConeIdentificator::run(uint64_t run_count)
{  
  ROS_INFO("run SeeLidarConeIdentificator");
}


void SeeLidarConeIdentificator::topicCallback(const std_msgs::String::ConstPtr& message)
{
  ROS_INFO("ConeIdentificator received a message: [%s]", message->data.c_str());
  
  // The message data object
  see_lidar_processing::ConeContainer lidar_cones_message;

  // TODO: with run_count one can simulate a "moving car"
  float_t simulated_distance = run_count % 50;
  simulated_distance /= 10;

  std::vector<see_lidar_processing::Cone> cones;

  for(int i = 0; i < 3; i++) {
    see_lidar_processing::Coordinates right_cones_coordinates;
    right_cones_coordinates.x = 2;
    right_cones_coordinates.y = simulated_distance + 5 * i + 1;

    see_lidar_processing::Coordinates left_cones_coordinates;
    left_cones_coordinates.x = -2;
    left_cones_coordinates.y = simulated_distance + 5 * i + 1;

    see_lidar_processing::ConeColor left_cone_color;
    left_cone_color.color = left_cone_color.CONE_COLOR_UNKNOWN;
    left_cone_color.probability = 1.0;

    see_lidar_processing::ConeColor right_cone_color;
    right_cone_color.color = right_cone_color.CONE_COLOR_UNKNOWN;
    right_cone_color.probability = 1.0;
		
    see_lidar_processing::Cone left_cone;
    left_cone.coordinates = left_cones_coordinates;
    left_cone.cone_colors.push_back(left_cone_color);
    left_cone.cone_probability = 1.0;
		
    see_lidar_processing::Cone right_cone;
    right_cone.coordinates = right_cones_coordinates;
    right_cone.cone_colors.push_back(right_cone_color);
    right_cone.cone_probability = 1.0;
		
    cones.push_back(left_cone);
    cones.push_back(right_cone);
  }

  lidar_cones_message.cones = cones;
  lidar_cones_message.timestamp = ros::Time::now();
  lidar_cones_message.count = lidar_cones_message.cones.size();


  publisher.publish(lidar_cones_message);
  ROS_INFO("published a message containing the data of the cones.");
  
  run_count++;
}

bool SeeLidarConeIdentificator::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_lidar_processing service response";
  return true;
}

