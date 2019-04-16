/*#include "see_localization/SeeLocalization.hpp"
#include "see_lidar_processing/SeeLidarProcessing.hpp"

#include <string>

SeeLocalization::SeeLocalization(ros::NodeHandle& nodeHandle)
  : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "see_localization_subscription";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &SeeLocalization::topicCallback, this);

  //std::string published_topic = "see_localization_publication";
  //publisher = nodeHandle.advertise<std_msgs::String>(published_topic, 1);

  std::string published_topic = "see_localization_publication";
  publisher = nodeHandle.advertise<see_localization::map_update>(published_topic, 10);

  std::string advertised_service = "see_localization_service";
  serviceServer = nodeHandle.advertiseService(advertised_service, &SeeLocalization::serviceCallback, this);

  ROS_INFO("started node see_localization skidpad");
}

SeeLocalization::~SeeLocalization()
{
}*/

/*see_localization::Cone createCone(float x, float y, std::vector<float> prob)
  {
    see_localization::Cone o;
    o.x_ = x;
    o.y_ = y;
    o.type_probability_vector_ = prob;
    return o;
  }*/

/*void SeeLocalization::run()
{
  /*std_msgs::String message;
  message.data = "see_localization publication";
  publisher.publish(message);
  */



  /*see_localization::map_update message1;

  std::vector<float> r;
  r.push_back(0.0);
  r.push_back(0.0);
  r.push_back(0.0);
  r.push_back(0.0);
  r.push_back(1.0);
  //ROS_INFO("Hallöchen");

  std::vector<float> l;
  l.push_back(1.0);
  l.push_back(0.0);
  l.push_back(0.0);
  l.push_back(0.0);
  l.push_back(0.0);

  std::vector<float> be;
  be.push_back(0.0);
  be.push_back(1.0);
  be.push_back(0.0);
  be.push_back(0.0);
  be.push_back(0.0);

  message1.closed_=false;
  message1.map_time=ros::Time::now();
  std::vector<see_localization::Obstacle> obs;
  
  /*for (int idx = 0; idx < 20; idx++)
  {
    see_localization::Obstacle o;
    o.x_=13;
    o.y_=45;
    o.type_probability_vector_ = d;
    obs.push_back(o);
  }*/

  //skidpad map
  /*obs.push_back(createObstacle(0.0,1.5,be));
  obs.push_back(createObstacle(10.625,1.5,l));
  obs.push_back(createObstacle(13.543,2.08,l));
  obs.push_back(createObstacle(16.017,3.733,l));
  obs.push_back(createObstacle(17.67,6.207,l));
  obs.push_back(createObstacle(18.25,9.125,l));
  obs.push_back(createObstacle(17.67,12.043,l));
  obs.push_back(createObstacle(16.017,14.517,l));
  obs.push_back(createObstacle(13.543,16.17,l));
  obs.push_back(createObstacle(10.625,16.75,l));
  obs.push_back(createObstacle(7.707,16.17,l));
  obs.push_back(createObstacle(5.233,14.517,l));
  obs.push_back(createObstacle(3.58,12.043,l));
  obs.push_back(createObstacle(3.0,9.125,l));
  obs.push_back(createObstacle(3.58,6.207,l));
  obs.push_back(createObstacle(5.233,3.733,l));
  obs.push_back(createObstacle(7.707,2.08,l));
  obs.push_back(createObstacle(0.0,-1.5,be));
  obs.push_back(createObstacle(10.625,-1.5,r));
  obs.push_back(createObstacle(13.543,-2.08,r));
  obs.push_back(createObstacle(16.017,-3.733,r));
  obs.push_back(createObstacle(17.67,-6.207,r));
  obs.push_back(createObstacle(18.25,-9.125,r));
  obs.push_back(createObstacle(17.67,-12.043,r));
  obs.push_back(createObstacle(16.017,-14.517,r));
  obs.push_back(createObstacle(13.543,-16.17,r));
  obs.push_back(createObstacle(10.625,-16.75,r));
  obs.push_back(createObstacle(7.707,-16.17,r));
  obs.push_back(createObstacle(5.233,-14.517,r));
  obs.push_back(createObstacle(3.58,-12.043,r));
  obs.push_back(createObstacle(3.0,-9.125,r));
  obs.push_back(createObstacle(3.58,-6.207,r));
  obs.push_back(createObstacle(5.233,-3.733,r));
  obs.push_back(createObstacle(7.707,-2.08,r));
  obs.push_back(createObstacle(20.442,5.059,r));
  obs.push_back(createObstacle(21.25,9.125,r));
  obs.push_back(createObstacle(20.442,13.191,r));
  obs.push_back(createObstacle(18.138,16.638,r));
  obs.push_back(createObstacle(14.691,18.942,r));
  obs.push_back(createObstacle(10.625,19.75,r));
  obs.push_back(createObstacle(6.559,18.942,r));
  obs.push_back(createObstacle(3.112,16.638,r));
  obs.push_back(createObstacle(0.808,13.191,r));
  obs.push_back(createObstacle(0.0,9.125,r));
  obs.push_back(createObstacle(0.808,5.059,r));
  obs.push_back(createObstacle(20.442,-5.059,l));
  obs.push_back(createObstacle(21.25,-9.125,l));
  obs.push_back(createObstacle(20.442,-13.191,l));
  obs.push_back(createObstacle(18.138,-16.638,l));
  obs.push_back(createObstacle(14.691,-18.942,l));
  obs.push_back(createObstacle(10.625,-19.75,l));
  obs.push_back(createObstacle(6.559,-18.942,l));
  obs.push_back(createObstacle(3.112,-16.638,l));
  obs.push_back(createObstacle(0.808,-13.191,l));
  obs.push_back(createObstacle(0.0,-9.125,l));
  obs.push_back(createObstacle(0.808,-5.059,l));
  obs.push_back(createObstacle(21.25,1.5,be));
  obs.push_back(createObstacle(21.25,-1.5,be));

  message1.obstacles_ = obs;


  

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    /*publisher.publish(message1);


}


void SeeLocalization::topicCallback(SeeLidarProcessing::Cone& msg)
{
  

}

bool SeeLocalization::serviceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response)
{
  response.success = true;
  response.message = "see_localization service response";
  return true;
}
