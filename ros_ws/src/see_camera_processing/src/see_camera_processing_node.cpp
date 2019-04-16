#include <ros/ros.h>
#include "see_camera_processing/SeeCameraProcessing.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_camera_processing");
  ros::NodeHandle nodeHandle("~");

  SeeCameraProcessing rosPackageTemplate(nodeHandle);

  // @param double frequency [Hz] in which to repeat the loop
  ros::Rate rate(1);
  while(ros::ok())
  {
    rosPackageTemplate.run();
    rate.sleep();
  }
  return 0;
}