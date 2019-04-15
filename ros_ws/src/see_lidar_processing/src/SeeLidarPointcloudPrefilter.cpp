#include "see_lidar_processing/SeeLidarPointcloudPrefilter.hpp"

//#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <XmlRpcValue.h>
#include <boost/thread.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_lidar_pointcloud_prefilter");
  ros::NodeHandle nodeHandle("see_lidar");

  SeeLidarPointcloudPrefilter pointcloudPrefilter(nodeHandle);

  // check for multithreading
  if(pointcloudPrefilter.isMultithreadingEnabled()) {
    ros::AsyncSpinner spinner(boost::thread::hardware_concurrency());
    spinner.start();
    ros::waitForShutdown();
  } else {
    ros::spin();    
  }

  return 0;
}

SeeLidarPointcloudPrefilter::SeeLidarPointcloudPrefilter(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  //std::string subscribed_topic = "see_lidar_whole_point_cloud";
  std::string subscribed_topic = "/os1_node/points";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLidarPointcloudPrefilter::topicCallback, this);

  std::string published_topic = "see_lidar_points_of_interest";
  publisher = nodeHandle.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

  std::string published_topic2 = "see_lidar_points_of_ground";
  publisher2 = nodeHandle.advertise<sensor_msgs::PointCloud2>(published_topic2, 1);

  /*std::string advertised_service = "see_lidar_pointcloud_prefilter_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLidarPointcloudPrefilter::serviceCallback, this);*/

  ROS_INFO("started node see_lidar_pointcloud_prefilter");

  auto all_successfull = loadConfiguration();
  if(!all_successfull) {
    ROS_INFO_STREAM("Could not read all parameters.");
  }
}

SeeLidarPointcloudPrefilter::~SeeLidarPointcloudPrefilter()
{
}

void SeeLidarPointcloudPrefilter::run(uint64_t run_count)
{
  ROS_INFO("Doing nothing");
}


void SeeLidarPointcloudPrefilter::topicCallback(const sensor_msgs::PointCloud2ConstPtr& message)
{
  ROS_INFO("publish message from PointcloudPrefilter -topicCallback-");
  sensor_msgs::PointCloud2 output = *message;

  auto frame = output.header.frame_id;
  auto timestamp = output.header.stamp;

  CloudOS1 pointCloud;
  pcl::fromROSMsg(*message, pointCloud);
  auto vector_for_cloud = std::vector<int>();

  vector_for_cloud = get_front_points(pointCloud);
  auto editable_cloud = CloudOS1(pointCloud, vector_for_cloud);

  //split pointcloud in diffrent parts of equal size f.e. 5²m --> split_point_cloud_in_areas(5.0, 5.0, editable_cloud)
  auto area_point_clouds_idieces = split_point_cloud_in_areas(10.0, 10.0, editable_cloud);
  CloudOS1 cone_cloud, groundpoints;
  for(unsigned int p = 0; p<area_point_clouds_idieces.size(); p++){
    if(area_point_clouds_idieces.at(p).size() > 0){
      if(p == 0 || p== 1){
        //points directly before the car
        cone_cloud += ransac(CloudOS1(editable_cloud, area_point_clouds_idieces.at(p)), 5, 50, 40, 0.035, &groundpoints, p);
      } else {
        cone_cloud += ransac(CloudOS1(editable_cloud, area_point_clouds_idieces.at(p)), 5, 20, 40, 0.035, &groundpoints, p);
      }
    }

  }

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cone_cloud, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = timestamp;
  publisher.publish(msg);

  pcl::toROSMsg(groundpoints, msg);
  msg.header.frame_id = frame;
  msg.header.stamp = timestamp;
  publisher2.publish(msg);

}

bool SeeLidarPointcloudPrefilter::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_lidar_processing service response";
  return true;
}


std::vector<int> SeeLidarPointcloudPrefilter::get_front_points(const CloudOS1& cloud) {
    auto front_points = std::vector<int>();
    double x,y,z;
    for(unsigned int i = 0; i < cloud.size(); i++) {
        x=cloud.at(i).x;
        y=cloud.at(i).y;
        z=cloud.at(i).z;
        if(x > 0 && x < field_of_interest_height && fabs(y) <= field_of_interest_width) {
            //remove front spoiler
            if( x > 0.4 || fabs(y) > 0.75){
                if(z < 0.2){
                //std::cout << atan(x/y) << std::endl;
                //without this the tan function have problems
                    if(y != 0){
                        double test = atan(x/y) * 180 / 3.14159265;
                        //cut left and right with 15 degrees
                        if(test > 15 || (test < -15)) {
                            front_points.push_back(i);
                        }
                    } else {
                        //all points with y=0 are relevant
                        front_points.push_back(i);
                    }
                }
            }
        }
    }
    return front_points;
}

void SeeLidarPointcloudPrefilter::get_indices_for_ground_plane(int cloud_size){
  //reset indices
  rand_idx1= -1;
  rand_idx2= -1;
  rand_idx3= -1;

  //create 3 unequal indices
  rand_idx1 = rand() % cloud_size;
  while(rand_idx2 == rand_idx1 || rand_idx2 == -1){
    rand_idx2 = rand() % cloud_size;
  }
  while(rand_idx3 == -1 || rand_idx3 == rand_idx1 || rand_idx3 == rand_idx2){
    rand_idx3 = rand() % cloud_size;
  }
}

//returns the x,y,z values of the point as vector
std::vector<double> SeeLidarPointcloudPrefilter::get_values_from_index(std::vector<double> point_vector, CloudOS1 editable_cloud, int rand_idx){
  point_vector.push_back(editable_cloud.at(rand_idx).x);
  point_vector.push_back(editable_cloud.at(rand_idx).y);
  point_vector.push_back(editable_cloud.at(rand_idx).z);
  return point_vector;
}

//calculates the cross product of two 3dimensional vectors according to schema f
std::vector<double> SeeLidarPointcloudPrefilter::calculate_cross_product(std::vector<double> vector1, std::vector<double> vector2){
  auto cross_product = std::vector<double>();
  cross_product.push_back(vector1.at(1)*vector2.at(2)-vector1.at(2)*vector2.at(1));
  cross_product.push_back(vector1.at(2)*vector2.at(0)-vector1.at(0)*vector2.at(2));
  cross_product.push_back(vector1.at(0)*vector2.at(1)-vector1.at(1)*vector2.at(0));
  return cross_product;
}

//creates different indices vectors for each area with the size "width" * "height" of the "editable_cloud and returns all vectors as a vector of int vectors"
std::vector<std::vector<int>> SeeLidarPointcloudPrefilter::split_point_cloud_in_areas(double width, double height, CloudOS1 editable_cloud){
  //calculate number of areas in x and y direction
  int num_height_areas = field_of_interest_height/height;
  int num_width_areas = field_of_interest_width*2/width;

  //create a vector with the size of number of areas
  std::vector<std::vector<int>> vector_of_point_cloud_indices(num_height_areas*num_width_areas);
  double x,y;
  int x_index, y_index;
  //std::cout << num_height_areas << " : " << num_width_areas << std::endl;
  //iterate over every point and determine area affiliation (calculate index)
  //indexes are ordered like this
  //|----------|----------|----------|----------|----------|----------|
  //|          |          |          |          |          |          |
  //|          |          |          |          |          |          |
  //|    23    |    21    |    19    |    18    |    20    |    22    |   x_idx = 3
  //|          |          |          |          |          |          |
  //|----------|----------|----------|----------|----------|----------|
  //|          |<-width-->|he        |          |          |          |
  //|          |          |i         |          |          |          |
  //|    17    |    15    |gh  13    |    12    |    14    |    16    |   x_idx = 2
  //|          |          |t         |          |          |          |
  //|----------|----------|----------|----------|----------|----------|
  //|          |          |          |          |          |          |
  //|          |          |          |          |          |          |
  //|    11    |    9     |    7     |    6     |    8     |    10    |   x_idx = 1
  //|          |          |          |          |          |          |
  //|----------|----------|----------|----------|----------|----------|
  //|          |          |          |          |          |          |
  //|          |          |          |          |          |          |
  //|    5     |    3     |    1     |    0     |    2     |    4     |   x_idx = 0
  //|          |          |          |          |          |          |
  //|----------|----------|----------|----------|----------|----------|
//   y_idx = 5   y_idx = 3  y_idx = 1 o y_idx = 0  y_idx = 2  y_idx = 4

//                                    ^
//                                    |
//                                    |
//                                  Lidar
  for(unsigned int i = 0; i< editable_cloud.size(); i++){
    x = editable_cloud.at(i).x;
    y = editable_cloud.at(i).y;

    //get x index (nach vorne)
    x_index = x/height;

    //get y index --> it must be considered whether y is greater than or less than 0
    if(y > 0){
      y_index = int(y/width)*2;
    } else {
      y_index = int(fabs(y)/width)*2 +1;
    }
    //add idx to calculated area/vector
    vector_of_point_cloud_indices.at(y_index+x_index*num_width_areas).push_back(i);
  }

  //now it may be that some areas contain very few points
  //so iterate from bottom to top and add areas with low number of points to the area over it
  //for example in the upper matrix add area with idx 5 to area with idx 11
  for(unsigned int i = 0; i< vector_of_point_cloud_indices.size(); i++){
    //do that if the pointcloud contains less than 100 points
    if(vector_of_point_cloud_indices.at(i).size() < 100){
      auto new_vector_of_indices = vector_of_point_cloud_indices.at(i);
      //if the area is on the top (in upper matrix all areas with x_idx=4) it will not work
      //solution: add it to the lower lying area with points
      try {
        //add the points to the upper area
        for(unsigned int j = 0; j<new_vector_of_indices.size(); j++){
          vector_of_point_cloud_indices.at(i+num_width_areas).push_back(new_vector_of_indices.at(j));
        }
        //and set the added area to zero
        vector_of_point_cloud_indices.at(i).clear();
      } catch (...) {
        //look if there is one area with points
        int test = int(i);
        while(test>=num_width_areas){
          test = test-num_width_areas;
        }
        //add the points to the lower area
        if(vector_of_point_cloud_indices.at(test).size() > 0){
          for(unsigned int j = 0; j<new_vector_of_indices.size(); j++){
            vector_of_point_cloud_indices.at(test).push_back(new_vector_of_indices.at(j));
          }
          //and set the added area to zero
          vector_of_point_cloud_indices.at(i).clear();
        }
      }
    }
  }
  //TODO eventually delete the vectors with size = 0 so the if query could deletet in the main
  return vector_of_point_cloud_indices;
}

CloudOS1 SeeLidarPointcloudPrefilter::ransac(CloudOS1 editable_cloud, int iterations, unsigned int min_points, int loops_per_iteration, double threshhold, CloudOS1 *ground_points, unsigned int p){
  CloudOS1 ground_cloud;
  unsigned int cloud_size = editable_cloud.size();;
  auto vector_Point_A = std::vector<double>();
  auto vector_Point_B = std::vector<double>();
  auto vector_Point_C = std::vector<double>();
  auto normal_straights = std::vector<CloudOS1>();

  int counter_loops = 0;
  auto rel2 = std::vector<int>();
  auto rel = std::vector<int>();
  auto relevant_point_clouds = std::vector<std::vector<int>>();
  auto non_relevant_point_clouds = std::vector<std::vector<int>>();

  auto degrees = std::vector<double>();

  auto number_ground_points = std::vector<int>();
  for(int b = 0; b<iterations && cloud_size > min_points; b++){
    counter_loops = 0;
    relevant_point_clouds.clear();
    non_relevant_point_clouds.clear();
    number_ground_points.clear();
    degrees.clear();
    normal_straights.clear();
    //this loop is executed multiple times because the result is random. the result with the most matches is taken.
    while(counter_loops < loops_per_iteration * (b+1)){
        CloudOS1 ownCloud;
        ownCloud.width = 50;
        ownCloud.height = 1;
        ownCloud.points.resize(ownCloud.width * ownCloud.height);
        rel.clear();
        rel2.clear();
        vector_Point_A.clear();
        vector_Point_B.clear();
        vector_Point_C.clear();

        //calculate 3 random idices (actually they are global)
        get_indices_for_ground_plane(cloud_size);

        //vector_Point_A = receptor point
        vector_Point_A = get_values_from_index(vector_Point_A, editable_cloud, rand_idx1);
        vector_Point_B = get_values_from_index(vector_Point_B, editable_cloud, rand_idx2);
        vector_Point_C = get_values_from_index(vector_Point_C, editable_cloud, rand_idx3);

        //calculate parameter equation of the plane
        // x = A + rB + sC
        for(int i = 0; i<3; i++){
            vector_Point_B.at(i) = vector_Point_B.at(i) - vector_Point_A.at(i);
            vector_Point_C.at(i) = vector_Point_C.at(i) - vector_Point_A.at(i);
        }

        //calculate normalenvector
        auto normal_straight = std::vector<double>();
        normal_straight = calculate_cross_product(vector_Point_B, vector_Point_C);

        auto standardized_normal_straight = std::vector<double>();
        double betrag = sqrt(normal_straight.at(0) * normal_straight.at(0) + normal_straight.at(1) * normal_straight.at(1) +normal_straight.at(2) * normal_straight.at(2));
        standardized_normal_straight.push_back(normal_straight.at(0)/ betrag);
        standardized_normal_straight.push_back(normal_straight.at(1)/ betrag);
        standardized_normal_straight.push_back(normal_straight.at(2)/ betrag);
        for(unsigned int o = 0; o < ownCloud.points.size(); o++){
            //vector_Point_A
            ownCloud.points[o].x = vector_Point_A.at(0) + standardized_normal_straight.at(0) * o/10;
            ownCloud.points[o].y = vector_Point_A.at(1) + standardized_normal_straight.at(1) * o/10;
            ownCloud.points[o].z = vector_Point_A.at(2) + standardized_normal_straight.at(2) * o/10;
            ownCloud.points[o].t = 0;
            ownCloud.points[o].intensity = 512;
            ownCloud.points[o].reflectivity = 512;
            ownCloud.points[o].ring = 1;
        }
        normal_straights.push_back(ownCloud);

        /*normal_straight.push_back(0);
        normal_straight.push_back(1);
        normal_straight.push_back(1);*/
        //determine coordinate form
        //ax1 + bx2 + cx3 = d
        double d = normal_straight.at(0)*vector_Point_A.at(0)+normal_straight.at(1)*vector_Point_A.at(1)+normal_straight.at(2)*vector_Point_A.at(2);
        //calculate cutting angle to xy-plane
        double param = fabs(normal_straight.at(2))/sqrt(normal_straight.at(0)*normal_straight.at(0)+normal_straight.at(1)*normal_straight.at(1)+normal_straight.at(2)*normal_straight.at(2));
        double gradient = asin (param) * 180.0 / 3.14159265;
        degrees.push_back(90-gradient);

        int counter_match = 0;
        int counter_missmatch = 0;

        //Determine distance of points to plane (Hesse normal form)
        double denominator = sqrt(normal_straight.at(0)*normal_straight.at(0)+normal_straight.at(1)*normal_straight.at(1)+normal_straight.at(2)*normal_straight.at(2));
        double numerator, distance;
        for(unsigned int i = 0; i < cloud_size; i++){
            numerator = normal_straight.at(0)*editable_cloud.at(i).x+normal_straight.at(1)*editable_cloud.at(i).y+normal_straight.at(2)*editable_cloud.at(i).z - d;
            distance = fabs(numerator/denominator);
            //if point near plane it's a match
            //if point belongs to a cone or a person it's a missmatch but attetion i only save those indices in rel
            if(distance > threshhold){
                counter_missmatch++;
                rel.push_back(i);
            }else{
                counter_match++;
                rel2.push_back(i);
            }
        }
        counter_loops++;

        //add the actually calculatet points to the result list
        relevant_point_clouds.push_back(rel);
        non_relevant_point_clouds.push_back(rel2);
        number_ground_points.push_back(counter_match);
    }

    // get the best result of the result list
    int max = 0;
    int matchs = 0;
    int index_of_max_matches = 0;
    //find max
    for(unsigned int k = 0; k<relevant_point_clouds.size(); k++){
      matchs = number_ground_points.at(k);
      if(matchs > max){
        max = matchs;
        index_of_max_matches = k;

      }
    }
    //save best result in rel (relevant_points)
    rel = relevant_point_clouds.at(index_of_max_matches);
    rel2 = non_relevant_point_clouds.at(index_of_max_matches);
    auto final_normal_straight = normal_straights.at(index_of_max_matches);
    //this query is important so that the point cloud does not become too small when iterating too often and therefore the cone itself is recognized as the "ground"
    //the new point clouds size must be less then halt of the original cloud
    //otherwise it is aborted
    if(rel.size() <= cloud_size*0.99){
      // the angles help validating the result easier, e.g. an angle lower than 85° is not realistic, as the ground plane
      // should not be as steep
      if(degrees.at(index_of_max_matches) < 20){
//         std::cout << degrees.at(index_of_max_matches) << " : " << p << " : " << b << std::endl;

        ground_cloud += CloudOS1(editable_cloud, rel2);
        ground_cloud += final_normal_straight;
        editable_cloud = CloudOS1(editable_cloud, rel);
      }
      if(cloud_size != editable_cloud.size()){
        cloud_size = editable_cloud.size();
      } else {
        break;
      }
    } else {
      break;
    }
  }
  *ground_points += ground_cloud;
  return editable_cloud;
}

bool SeeLidarPointcloudPrefilter::loadConfiguration()
{
  // default values
  ground_threshold = 0.3;
  area_size = 5;
  multithreading = true;

  // keys to get the configuration parameters
  std::string prefilter_key = "prefilter";
  std::string ground_threshold_key = "ground_threshold";
  std::string area_size_key = "area_size";
  std::string multithreading_enabled_key = "multithreading";

  // variable to indicate if all parameters could be loaded
  bool all_successfull = true;

  // TODO: this configuration needs rework

  XmlRpc::XmlRpcValue prefilter_config;
  
  // load configuration from parameter server
  // to edit the parameters in the configuration file, have a look at the launch-file.
  // There is the file path to the configuration file specified
  if(!nodeHandle.getParam(prefilter_key, prefilter_config)) {
    ROS_INFO_STREAM("Could not read config for Prefilter.");
    ROS_INFO_STREAM("Kept all default values");
    all_successfull = false;
  } else {
    ROS_ASSERT(prefilter_config.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    if(prefilter_config.hasMember(ground_threshold_key)) {
      ground_threshold = prefilter_config[ground_threshold_key];
    } else {
      ROS_INFO_STREAM(ground_threshold_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(prefilter_config.hasMember(area_size_key)) {
      area_size = prefilter_config[area_size_key];
    } else {
      ROS_INFO_STREAM(area_size_key << " not defined. Kept the default value");
      all_successfull = false;
    }
    if(prefilter_config.hasMember(multithreading_enabled_key)) {
      multithreading = prefilter_config[multithreading_enabled_key];
    } else {
      ROS_INFO_STREAM(multithreading_enabled_key << " not defined. Kept the default value");
      all_successfull = false;
    }
  }
  ROS_INFO_STREAM("Configured Prefilter...\n\t" <<
    ground_threshold_key << ": " << ground_threshold << "\n\t" <<
    area_size_key << ": " << area_size << "\n\t" <<
    multithreading_enabled_key << ": " << multithreading
  );

  return all_successfull;
}

bool SeeLidarPointcloudPrefilter::isMultithreadingEnabled()
{
  return multithreading;
}
