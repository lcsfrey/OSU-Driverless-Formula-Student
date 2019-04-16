//From : http://wiki.ros.org/image_transport/Tutorials/PublishingImages:
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <dirent.h>

int main(int argc, char **argv)
{
  //Path to working directory is
  ///home/tim/gfr19d_ws/devel/lib/see_camera_processing/image_generator

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("see_camera_processing/see_camera_processing_subscription", 1);

  cv::Mat image;
  sensor_msgs::ImagePtr msg;

  ros::Rate loop_rate(30);
  unsigned int i = 0;

  //Read images from directory:
  //From: https://stackoverflow.com/a/612176
  std::vector<cv::Mat> images;
  DIR *dir;
  struct dirent *ent;
  std::string path("../../../src/see_camera_processing/testdata/");
  if ((dir = opendir("../../../src/see_camera_processing/testdata/")) != NULL)
  {
    while ((ent = readdir(dir)) != NULL)
    {
      if (ent->d_name[0] != '.')
      {
        std::string name(ent->d_name);
        images.push_back(cv::imread(path + name, CV_LOAD_IMAGE_COLOR));
      }
    }
    closedir(dir);
  }
  else
  {
    ROS_ERROR("Could not open directory");
  }
  while (nh.ok())
  {
    if (!images[i].data)
    {
      ROS_ERROR("Image not found, place images inside the testdata folder");
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", images[i]).toImageMsg();
    pub.publish(msg);
    i++;
    if (i >= images.size())
    {
      i = 0;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
