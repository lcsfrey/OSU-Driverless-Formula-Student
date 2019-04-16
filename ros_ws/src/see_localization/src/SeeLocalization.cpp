#include "see_localization/SeeLocalization.hpp"
#include "see_localization/SLAM.hpp"
#include <iostream>
//#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_localization");
  ros::NodeHandle nodeHandle("see_localization");

  SeeLocalization SeeLocalization(nodeHandle);

  ros::spin();
  
  return 0;
}


SeeLocalization::SeeLocalization(ros::NodeHandle& nodeHandle)
 : nodeHandle(nodeHandle) {
	std::string subscribed_topic = "see_localization_subscription";
	subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLocalization::lidarMsgReceive, this);

	//subscribing to lidar messages
	std::string subscribed_lidar_topic = "see_lidar_processing_subscription";
	subscriber_lidar = nodeHandle.subscribe(subscribed_lidar_topic, 100, &SeeLocalization::lidarMsgReceive, this);

	//subscribing to camera messages
	std::string subscribed_camera_topic = "see_camera_processing_subscription";
	subscriber_camera = nodeHandle.subscribe(subscribed_camera_topic, 100, &SeeLocalization::cameraMsgReceive, this);


	//subscribed message setup for egomotion
	std::string subscribed_ego_topic = "odom_filtered";
	subscriber_ego = nodeHandle.subscribe(subscribed_ego_topic,100,&SeeLocalization::ego_motion_callback,this);

	// std::string subscribed_topic = "see_localization_subscription";
	// subscriber_camera = nodeHandle.subscribe(subscribed_topic, 1, &SeeLocalization::cameraMsgReceive, this);

			
	//std::string subscribed_lidar_topic = "see_lidar/see_lidar_publishing";
	//lidar_subscriber = nodeHandle.subscribe(subscribed_lidar_topic, 1, &SeeLocalization::topicCallback, this);
	//std::string published_topic = "see_localization_publication";
	//publisher = nodeHandle.advertise<std_msgs::String>(published_topic, 1);

	//std::string published_topic = "see_localization/world_map";
	publisher_map = nodeHandle.advertise<see_localization::world_map_msg>("world_map", 10);
//	publisher_pose = nodeHandle.advertise<see_localization::vehicle_pose_msg>("vehicle_pose", 10);
	publisher_pose = nodeHandle.advertise<see_ego_motion::see_ego_motion_interface>("vehicle_pose", 10);
	publisher_rviz_lm = nodeHandle.advertise<visualization_msgs::MarkerArray>("visualization_marker",1);
	publisher_rviz_pose = nodeHandle.advertise<geometry_msgs::PoseStamped>("visualization_pose",1);
	publisher_rviz_real_pose = nodeHandle.advertise<geometry_msgs::PoseStamped>("visualization_real_pose",1);//publishes true pose from fake data
	publisher_rviz_pose_cluster = nodeHandle.advertise<geometry_msgs::PoseArray>("visualization_pose_cluster",1);

	std::string advertised_service = "see_localization_service";
	serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLocalization::serviceCallback, this);

	ROS_INFO("started node see_localization skidpad");
}

SeeLocalization::~SeeLocalization() {
}

void SeeLocalization::run() {
	ROS_INFO("in run");
/*	
	DataGenerator dg;
	
	see_localization::world_map_msg world_map;
	int mysize = static_cast<int>(dg.m.cones_.size());

	see_localization::SLAM_cone_msg cmsg;

	for (int i = 0; i < mysize; i++) {
		cmsg.x_ = dg.m.cones_[i].x_;
		cmsg.y_ = dg.m.cones_[i].y_;
		cmsg.type_probability_vector_ = dg.m.cones_[i].type_probability_vector_;
		world_map.cones_.push_back(cmsg);
	}
	world_map.map_time = ros::Time::now();
	world_map.closed_ = dg.m.closed_;

	see_localization::vehicle_pose_msg vehicle_pose;
	vehicle_pose.x_global = dg.gp.x_global;
	vehicle_pose.y_global = dg.gp.y_global;
	vehicle_pose.theta_global = dg.gp.theta_global;

	publisher_map.publish(world_map);
	//publisher_pose.publish(vehicle_pose);//TODO change message type
*/
}

//void SeeLocalization::topicCallback(SeeLidarProcessing::Cone& msg)
//void SeeLocalization::topicCallback(const std_msgs::String::ConstPtr& message)
//{
//}

//Actually runs SLAM on Lidar trigger
void SeeLocalization::lidarMsgReceive(const see_lidar_processing::ConeContainer& cone_container) {
	//std::vector<see_lidar_processing::Cone> mycones = cone_container.cones;
	ROS_INFO("in lidar callback");
	std::vector<Cone> mycones;
	std::vector<float> colorprob;
	//con_container cone vector, timestamp and the count (i.e. number correlating to that specific cone)
	for (auto& c : cone_container.cones) { // populate array of color proabilties.
		for(int i = 0; i < 6; i++){
			colorprob.push_back(c.cone_colors[i].probability);
		}
		mycones.push_back(Cone((float) (c.coordinates.x), (float) (c.coordinates.y), colorprob)); 
		colorprob.clear();
	}
	//create particles?
	if(saved_particles.size() == 0){//if no particles saved
		for(int i = 0; i < 50; i++){//500 particles?
			saved_particles.push_back(Particle());//populate vector of particles set to 0
		}
	}
	SLAM s1 = SLAM(mycones, saved_particles, gp);//will pass in old_particles and cones from cam/lidar
	saved_particles = s1.runSLAM();//will run fastslam and return particles
	//get best particle
	Particle Best = getBestParticle(saved_particles);
	//see_localization::vehicle_pose_msg p_msg = create_pose_msg(Best);
	//publisher_pose.publish(p_msg);//send pose to egomotion
	see_ego_motion::see_ego_motion_interface p_msg = create_pose_msg(Best);
	publisher_pose.publish(p_msg);//send pose to egomotion
	see_localization::world_map_msg m_msg = create_map_msg(Best);
	publisher_map.publish(m_msg);//send map and pose to trajectory

	visualization_msgs::MarkerArray mark = vis_msg_lm(saved_particles);
	//geometry_msgs::PoseStamped pose = vis_msg_pose(saved_particles);
	publisher_rviz_lm.publish(mark);
	//publisher_rviz_pose.publish(pose);
	geometry_msgs::PoseArray pose_array = vis_pose_array(saved_particles);
	publisher_rviz_pose_cluster.publish(pose_array);
	geometry_msgs::PoseStamped realPose = real_pose(index);
	publisher_rviz_real_pose.publish(realPose);
}

//Actually runs SLAM on Camera trigger
void SeeLocalization::cameraMsgReceive(const see_camera_processing::ConeContainer& cone_container) {
	//std::vector<see_lidar_processing::Cone> mycones = cone_container.cones;
	std::vector<Cone> mycones;
	std::vector<float> colorprob;
	//con_container cone vector, timestamp and the count (i.e. number correlating to that specific cone)
	for (auto& c : cone_container.cones) { // populate array of color proabilties.
		for(int i = 0; i < 6; i++){
			colorprob.push_back(c.cone_color[i].probability); //COMMENT: Camera calls it cone_color and lidar calls it cone_colors
		}
		mycones.push_back(Cone((float) (c.coordinates.x), (float) (c.coordinates.y), colorprob)); 
		colorprob.clear();
	}
	//create particles?
	if(saved_particles.size() == 0){//if no particles saved
		for(int i = 0; i < 50; i++){//500 particles?
			saved_particles.push_back(Particle());//populate vector of particles set to 0
		}
	}
	
	SLAM s1 = SLAM(mycones, saved_particles, gp);//will pass in old_particles and cones from cam/lidar
	saved_particles = s1.runSLAM();//will run fastslam and return particles
		
	//get best particle
	Particle Best = getBestParticle(saved_particles);
	//see_localization::vehicle_pose_msg p_msg = create_pose_msg(Best);
	//publisher_pose.publish(p_msg);//send pose to egomotion
	see_ego_motion::see_ego_motion_interface p_msg = create_pose_msg(Best);
	publisher_pose.publish(p_msg);//send pose to egomotion
	see_localization::world_map_msg m_msg = create_map_msg(Best);
	publisher_map.publish(m_msg);//send map and pose to trajectory

}


//egomotion message callback
void SeeLocalization::ego_motion_callback(const see_ego_motion::see_ego_motion_interface& ego_interface){
	ROS_INFO("in ego callback");
	gp.x_global = ego_interface.x_global;//add new pose into pose object in SeeLocalization
	gp.y_global = ego_interface.y_global;
	gp.theta_global = ego_interface.theta_global;

	index++;//used for visualizing fake data
}

Particle SeeLocalization::getBestParticle(std::vector<Particle> allParticles){
	Particle particle_max = allParticles.front();
	for (auto& w: allParticles){
		if (w.w_p > particle_max.w_p){
			particle_max = w; //update particle_max to the newest particle that has a higher weight
		}
		else{
			//do nothing
		}
	}
	return particle_max;
}

//create pose message for egomotion
see_ego_motion::see_ego_motion_interface SeeLocalization::create_pose_msg(Particle Best){
	see_ego_motion::see_ego_motion_interface pose_msg;//create message
	pose_msg.x_global = Best.gp_p.x_global;//set x position
	pose_msg.y_global = Best.gp_p.y_global;//set y position
	pose_msg.theta_global = Best.gp_p.theta_global;//set theta
	return pose_msg;
}

//create map message for trajectory planning
see_localization::world_map_msg SeeLocalization::create_map_msg(Particle Best){
	see_localization::world_map_msg map_msg;//create message
	std::vector<see_localization::SLAM_cone_msg> cone_v;
	for(int i = 0; i < Best.N; i++){//loop through all landmarks
		see_localization::SLAM_cone_msg cone;
		cone.x_ = Best.le_p[i].mean(0);
		cone.y_ = Best.le_p[i].mean(1);
		cone.type_probability_vector_ = Best.le_p[i].tpv;
		cone_v.push_back(cone);
		//map_msg.cones_[i].x_ = Best.le_p[i].mean(0);//set x pose
		//map_msg.cones_[i].y_ = Best.le_p[i].mean(1);//set y pose
		//map_msg.cones_[i].type_probability_vector_ = Best.le_p[i].tpv;//set type probability vector
	}
	map_msg.cones_ = cone_v;
	map_msg.x_global_vehicle = Best.gp_p.x_global;//set x pose of car
	map_msg.y_global_vehicle = Best.gp_p.y_global;//set y pose of car
	map_msg.theta_global_vehicle = Best.gp_p.theta_global;//set theta of car
	ros::Time begin = ros::Time::now();
	map_msg.map_time = begin;//set time
	map_msg.closed_ = false;//loop closure check needed
	return map_msg;
}
bool SeeLocalization::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
	response.success = true;
	response.message = "see_localization service response";
	return true;
}

//create messages to plot landmarks in rviz
visualization_msgs::MarkerArray SeeLocalization::vis_msg_lm(std::vector<Particle> v_particles){
        visualization_msgs::MarkerArray marker_array;
        //Particle one = getBestParticle(v_particles);
	int k = 0;
	for(auto& one:v_particles){
	marker_array.markers.resize(marker_array.markers.size() + one.le_p.size());
	//marker_array.markers.resize(one.le_p.size());
        for(unsigned int i = 0; i < one.le_p.size(); i++){
               	uint32_t shape = visualization_msgs::Marker::SPHERE;
                marker_array.markers[k].header.frame_id = "/slam";
                marker_array.markers[k].header.stamp = ros::Time::now();
                marker_array.markers[k].ns = "basic_shapes";
                marker_array.markers[k].id = i;
                marker_array.markers[k].type = shape;
                marker_array.markers[k].action = visualization_msgs::Marker::ADD;
                marker_array.markers[k].pose.position.x = one.le_p[i].mean(0);
                marker_array.markers[k].pose.position.y = one.le_p[i].mean(1);
                marker_array.markers[k].pose.position.z = 0.0;
                marker_array.markers[k].pose.orientation.x = 0.0;
                marker_array.markers[k].pose.orientation.y = 0.0;
                marker_array.markers[k].pose.orientation.z = 0.0;
                marker_array.markers[k].pose.orientation.w = 1.0;

                marker_array.markers[k].scale.x = 0.5;
                marker_array.markers[k].scale.y = 0.5;
                marker_array.markers[k].scale.z = 0.5;
		if(one.le_p[i].associated == true){//make red if this lm had association
			marker_array.markers[k].color.r = 1.0f;
			marker_array.markers[k].color.g = 0.0f;
		}
		else{
                	marker_array.markers[k].color.r = 0.0f;
	                marker_array.markers[k].color.g = 1.0f;
		}
                marker_array.markers[k].color.b = 0.0f;
                marker_array.markers[k].color.a = 0.5;
                marker_array.markers[k].lifetime = ros::Duration();
		k++;
        }
	}
        return marker_array;
}


//create messages to plot car pose in rviz
geometry_msgs::PoseStamped SeeLocalization::vis_msg_pose(std::vector<Particle> v_particles){
	geometry_msgs::PoseStamped car_pose;
	Particle one = getBestParticle(v_particles);
	car_pose.header.frame_id = "/slam";
	car_pose.header.stamp = ros::Time::now();
	car_pose.pose.position.x = one.gp_p.x_global;
	car_pose.pose.position.y = one.gp_p.y_global;
	double k = one.gp_p.theta_global;
	tf::Quaternion q = tf::createQuaternionFromRPY(0,0,k);
	car_pose.pose.orientation.w = q.w();
	car_pose.pose.orientation.x = q.x();
	car_pose.pose.orientation.y = q.y();
	car_pose.pose.orientation.z = q.z();
	return car_pose;
}

geometry_msgs::PoseArray SeeLocalization::vis_pose_array(std::vector<Particle> v_particles){
	geometry_msgs::PoseArray pose_cluster;
	pose_cluster.poses.resize(v_particles.size());
	int i = 0;
	for(auto& p: v_particles){
		pose_cluster.header.frame_id = "/slam";
		pose_cluster.header.stamp = ros::Time::now();
		pose_cluster.poses[i].position.x = p.gp_p.x_global;
		pose_cluster.poses[i].position.y = p.gp_p.y_global;
		double k = p.gp_p.theta_global;
		tf::Quaternion q = tf::createQuaternionFromRPY(0,0,k);
		pose_cluster.poses[i].orientation.w = q.w();
		pose_cluster.poses[i].orientation.x = q.x();
		pose_cluster.poses[i].orientation.y = q.y();
		pose_cluster.poses[i].orientation.z = q.z();
		i++;
	}
	return pose_cluster;
}

geometry_msgs::PoseStamped SeeLocalization::real_pose(int index){
        geometry_msgs::PoseStamped pose;
        std::ifstream carfile("src/see_localization/src/Data/carLocation.txt");
        if (carfile.is_open()) {
                carfile.clear();
                carfile.seekg(0, std::ios::beg);
                std::string pose_line;
                std::vector< std::vector<std::string>> poses;
                while (getline(carfile, pose_line)) {
                        std::stringstream ss1(pose_line);
                        std::istream_iterator<std::string> begin1(ss1);
                        std::istream_iterator<std::string> end1;
                        std::vector<std::string> vstrings(begin1, end1);

                        poses.push_back(vstrings);
                }
                std::vector<std::string> pose_string = poses[index-1];
                pose.header.frame_id = "/slam";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = stof(pose_string[0]);
                pose.pose.position.y = stof(pose_string[1]);
                double k = stof(pose_string[2]);
                tf::Quaternion q = tf::createQuaternionFromRPY(0,0,k);
                pose.pose.orientation.w = q.w();
                pose.pose.orientation.x = q.x();
                pose.pose.orientation.y = q.y();
                pose.pose.orientation.z = q.z();
        }
                return pose;

}
