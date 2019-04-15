#include <ros/ros.h>
#include "see_ego_motion/testing/SeeEgoMotionTest.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_ego_motion_test");
  ros::NodeHandle nodeHandle("~");

  SeeEgoMotionTest rosPackageTemplate(nodeHandle);

  // @param double frequency [Hz] in which to repeat the loop
  // ros::Rate rate(1);
  while(ros::ok())
  {
    rosPackageTemplate.run();
    ros::spinOnce();
  }
  return 0;
}