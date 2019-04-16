#pragma once

#include <std_msgs/String.h>
#include <vector>
#include <ros/ros.h>
#include "see_localization/Cone.hpp"
//#include "obstacle.hpp"
//#include "coneMap.hpp"

// class map : public coneMap
// {
// public:

// 	vector<obstacle> obstacles_;
// 	//timestamp map_time; not used right now, because we don't know what the timestamp class looks like
// 	float map_time;
// 	bool closed_;

//   /*!
//    * Constructor
//    */
// 	map() : cone(vector<cone>(),vector<obstacle>(),0.0,false){};
// 	map(vector<cone> co, vector<obstacle> o, float mt, bool cl) : coneMap(co), obstacles_(o), map_time(mt), closed_(cl) {};
//   }

//   /*!
//    * Destructor.
//    */
//   ~obstacle();
// };

//class Map : public world_map_msg{
class Map{
  public:

	  std::vector<Cone> cones_;
	  //timestamp map_time; not used right now, because we don't know what the timestamp class looks like
	  ros::Time map_time;
	  bool closed_;

    /*!
     * Constructor
     */
	  //Map(){};
	  //Map(vector<Cone> c, ros::Time mt, bool cl) : cones_(o), map_time(mt), closed_(cl) {};

    Map(){closed_=false;};
    Map(std::vector<Cone> c, ros::Time mt, bool cl){
    cones_ = c;
    map_time = mt;
    closed_ = cl;
  }

  /*!
   * Destructor.
   */
  ~Map(){};
};
