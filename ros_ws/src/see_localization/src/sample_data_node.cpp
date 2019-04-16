#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <algorithm>
#include <sstream>
#include <iterator>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "see_localization/Cone.hpp"
#include "see_localization/GlobalPose.hpp"
#include "see_localization/Particle.hpp"
#include "see_localization/SLAM.hpp"
#include "see_lidar_processing/ConeContainer.h"
#include "see_lidar_processing/Cone.h"
#include "see_ego_motion/see_ego_motion_interface.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
using namespace std;
see_lidar_processing::ConeContainer SetCones(string line);
see_ego_motion::see_ego_motion_interface SetPose(string line);
visualization_msgs::MarkerArray plot_observed(see_lidar_processing::ConeContainer container);
visualization_msgs::MarkerArray real_cones();
GlobalPose gp_c;
std::vector<GlobalPose> gpv;

int main(int argc, char* argv[]){
	
	ros::init(argc, argv, "sample_data");
	ros::NodeHandle n;
	ros::Publisher publish_cones = n.advertise<see_lidar_processing::ConeContainer>("see_localization/see_lidar_processing_subscription", 1000);
	ros::Publisher publish_pose = n.advertise<see_ego_motion::see_ego_motion_interface>("see_localization/odom_filtered",1000);
	ros::Publisher publish_rviz_cones = n.advertise<visualization_msgs::MarkerArray>("visualization_observed",1);
	ros::Publisher publish_rviz_true_pose = n.advertise<geometry_msgs::PoseStamped>("visualization_true_pose",1);
	ros::Rate loop_rate(10);
	int count = 0;

	visualization_msgs::MarkerArray real_lm = real_cones();	
	int k = 1;
	std::ifstream file("src/see_localization/src/Data/dataSetOne.txt");
	if (file.is_open()) {
		file.clear();
		file.seekg(0, ios::beg);
		std::string line;
	
		cout << "before ros::ok" << endl;
		while(ros::ok()){
			if(publish_pose.getNumSubscribers() > 0 && publish_cones.getNumSubscribers() > 0){
				publish_rviz_cones.publish(real_lm);
				while (getline(file, line)) {

					see_lidar_processing::ConeContainer cont = SetCones(line);
					//visualization_msgs::MarkerArray marks = plot_observed(cont);
					//publish_rviz_cones.publish(marks);
					publish_pose.publish(SetPose(line));
					//if(k!=0){
					//}
					k++;
					sleep(1);
					publish_cones.publish(cont);
				}
			}
			ros::spinOnce();
			loop_rate.sleep();
			++count;
		}
		file.close();
		cout << "file closed" << endl;
	}

	return 0;
}
//read in fake cone observations into correct msg type
see_lidar_processing::ConeContainer SetCones(string line){
	std::vector<Cone> observed_cones;//vector to send to SLAM
	GlobalPose gp;
	std::vector<float> colorprob(6);
	int num_cones = line.at(0)-48;
	std::string mystr = line.substr(2,line.find(",")-2);//cones x,y,theta ...

	std::stringstream ss(mystr);
	std::istream_iterator<std::string> begin(ss);
	std::istream_iterator<std::string> end;
	std::vector<std::string> vstrings(begin, end);
	int i = 0;
	int j = 0;
	int color_x = 1.0;
	while(i < num_cones){//for all cones
		int k;
		cout << " num " << num_cones << endl;
		cout << vstrings[0] << endl;
//		std::istringstream(vstrings[j+2])>>k;
		k = 1.0;
		if(k == 0){
			colorprob[0] = 1;
			color_x = 0;
		}
		else{
			colorprob[1] = 1;
			color_x = 1;
		}
		observed_cones.push_back(Cone(stof(vstrings[j]),stof(vstrings[j+1]),colorprob));//add cone with x,y,tpv
		j = j + 3;
		i = i + 1;
	}
//now getting global pose

	std::string gp_string = line.substr(line.find(",")+2,line.length());

	std::stringstream ss1(gp_string);
        std::istream_iterator<std::string> begin1(ss1);
        std::istream_iterator<std::string> end1;
        std::vector<std::string> vstrings1(begin1, end1);
	gp.x_global = stof(vstrings1[0]);
	gp.y_global = stof(vstrings1[1]);
	gp.theta_global = stof(vstrings1[2]);

	//publish data in ConeContainer to SLAM
	see_lidar_processing::ConeContainer container;//create message
	std::vector<see_lidar_processing::Cone> cones_msg;
	//container.cones = observed_cones;
	for(int q = 0; q < num_cones; q++){
		see_lidar_processing::Cone single_cone;
		std::vector<see_lidar_processing::ConeColor> cc;
		single_cone.coordinates.x = observed_cones[q].x_;
		single_cone.coordinates.y = observed_cones[q].y_;
		for(int k = 0; k < 6; k++){
			see_lidar_processing::ConeColor c;
			c.color = k;
			c.probability = 0.0;
			cc.push_back(c);
		}
		cc[color_x].probability = 1.0;
		single_cone.cone_colors = cc;
		//single_cone.cone_colors[0].color = color_x;
		//single_cone.cone_colors[0].probability = 1;
		cones_msg.push_back(single_cone);
		//cones_msg[q].coordinates.x = observed_cones[q].x_;
		//cones_msg[q].coordinates.y = observed_cones[q].y_;
		//cones_msg[q].cone_colors
	}
	container.cones = cones_msg;
	//publish_cones.publish(container);
	
	return container;


}
//read fake egomotion data into msg format
see_ego_motion::see_ego_motion_interface SetPose(string line){
	        std::vector<Cone> observed_cones;//vector to send to SLAM
        GlobalPose gp;
        std::vector<float> colorprob(6);
        std::string mystr = line.substr(2,line.find(",")-2);//cones x,y,theta ...

        std::stringstream ss(mystr);
        std::istream_iterator<std::string> begin(ss);
        std::istream_iterator<std::string> end;
        std::vector<std::string> vstrings(begin, end);


	//now getting global pose

        std::string gp_string = line.substr(line.find(",")+2,line.length());

        std::stringstream ss1(gp_string);
        std::istream_iterator<std::string> begin1(ss1);
        std::istream_iterator<std::string> end1;
        std::vector<std::string> vstrings1(begin1, end1);
        gp.x_global = stof(vstrings1[0]);
        gp.y_global = stof(vstrings1[1]);
        gp.theta_global = stof(vstrings1[2]);

	//create message sending car pose
	see_ego_motion::see_ego_motion_interface pose;
	pose.x_global = gp.x_global;
	pose.y_global = gp.y_global;
	pose.theta_global = gp.theta_global;
	cout << pose.x_global << endl;
	return pose;
}

//change ConeContainer type into MakerArray type for rviz. plots cones in single observation.
//not done. needs to turned into global poses.
visualization_msgs::MarkerArray plot_observed(see_lidar_processing::ConeContainer container){
	//needs to be changed to global
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.resize(sizeof(container.cones));
	int i = 0;
	uint32_t shape = visualization_msgs::Marker::SPHERE;
	for(auto& c: container.cones){
		marker_array.markers[i].header.frame_id = "/slam";
		marker_array.markers[i].header.stamp = ros::Time::now();
		marker_array.markers[i].ns = "basic_shapes_1";
		marker_array.markers[i].id = i;
		marker_array.markers[i].type = shape;
		marker_array.markers[i].action = visualization_msgs::Marker::ADD;
                marker_array.markers[i].pose.position.x = c.coordinates.x;
                marker_array.markers[i].pose.position.y = c.coordinates.y;
		marker_array.markers[i].pose.orientation.w = 1.0;
		marker_array.markers[i].scale.x = 1.0;
                marker_array.markers[i].scale.y = 1.0;
                marker_array.markers[i].scale.z = 1.0;
                marker_array.markers[i].color.r = 1.0f;
                marker_array.markers[i].color.g = 1.0f;
                marker_array.markers[i].color.b = 0.0f;
                marker_array.markers[i].color.a = 1.0;
                marker_array.markers[i].lifetime = ros::Duration();
		i++;
	}
	return marker_array;
}

//plot true cone locations to rviz
visualization_msgs::MarkerArray real_cones(){
	cout << "in real cone file "<< endl;
	visualization_msgs::MarkerArray marker_array;
	marker_array.markers.resize(100);
	int i = 0;
	uint32_t shape = visualization_msgs::Marker::SPHERE;
	std::ifstream file1("src/see_localization/src/Data/coneLocation.txt");
        if (file1.is_open()) {
		cout << "file open" << endl;
                file1.clear();
                file1.seekg(0, ios::beg);
                std::string line;
		while (getline(file1, line)) {
			std::stringstream ss1(line);
		        std::istream_iterator<std::string> begin1(ss1);
		        std::istream_iterator<std::string> end1;
        		std::vector<std::string> vstrings(begin1, end1);


			marker_array.markers[i].header.frame_id = "/slam";
                	marker_array.markers[i].header.stamp = ros::Time::now();
        	        marker_array.markers[i].ns = "basic_shapes_2";
	                marker_array.markers[i].id = i;
        	        marker_array.markers[i].type = shape;
        	        marker_array.markers[i].action = visualization_msgs::Marker::ADD;

			marker_array.markers[i].pose.orientation.w = 1.0;
	                marker_array.markers[i].scale.x = 0.25;
	                marker_array.markers[i].scale.y = 0.25;
	                marker_array.markers[i].scale.z = 0.25;
	                marker_array.markers[i].color.r = 1.0f;
	                marker_array.markers[i].color.g = 1.0f;
	                marker_array.markers[i].color.b = 1.0f;
	                marker_array.markers[i].color.a = 1.0;
			marker_array.markers[i].lifetime = ros::Duration();
			marker_array.markers[i].pose.position.x = stof(vstrings[0]);
			marker_array.markers[i].pose.position.y = stof(vstrings[1]);
			cout << "x " << marker_array.markers[i].pose.position.x << endl;
			i++;	
		}
		file1.close();
                cout << "file closed" << endl;
	}
	return marker_array;
}

