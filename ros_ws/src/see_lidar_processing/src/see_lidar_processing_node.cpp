#include <ros/ros.h>
#include "see_lidar_processing/SeeLidarProcessing.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_lidar_processing");
  ros::NodeHandle nodeHandle("~");

  // rosPackageTemplate (or rather SeeLidarProcessing) seems to be outdated/replaced by other classes and methods.
  // so renaming or something else (e.g. deleting) is probably necessary.
  SeeLidarProcessing rosPackageTemplate(nodeHandle);

  // @param double frequency [Hz] in which to repeat the loop
  ros::Rate rate(10);
  uint64_t run_count = 0;
  while(ros::ok())
  {
    rosPackageTemplate.run(run_count);
    rate.sleep();
    run_count++;
  }
  return 0;
}
