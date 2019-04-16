#include <ros/ros.h>
#include "see_ego_motion/SeeEgoMotion.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_ego_motion");
  ros::NodeHandle nodeHandle("~");

  SeeEgoMotion rosPackageTemplate(nodeHandle);

  // @param double frequency [Hz] in which to repeat the loop
  ros::Rate rate(1);
  while(ros::ok())
  {
    rosPackageTemplate.run();
    rate.sleep();
  }
  return 0;
}