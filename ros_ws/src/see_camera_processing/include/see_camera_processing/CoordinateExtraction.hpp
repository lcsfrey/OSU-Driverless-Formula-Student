#pragma once

#include "see_camera_processing/Algorithm.hpp"

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>

#include "see_camera_processing/ImageCones.h"
#include "see_camera_processing/ImageCone.h"
#include "see_camera_processing/ImageCoordinates.h"

#include "see_camera_processing/ConeContainer.h"
#include "see_camera_processing/Cone.h"
#include "see_camera_processing/Coordinates.h"
#include "see_camera_processing/CovarianceMatrix.h"
#include "see_camera_processing/ConeColor.h"

#include <opencv2/opencv.hpp>

//How long should the ellipse be?
#define SCALE_FACTOR_LONG_AXIS 0.05
//How broad should the ellipse be? (in rad)
#define SCALE_ANGLE 0.0349066
#define CHI_SQUARE_FACTOR 5.991

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class CoordinateExtraction
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  CoordinateExtraction(ros::NodeHandle &nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~CoordinateExtraction();

private:
  struct CoordinateColorPair
  {
    see_camera_processing::Coordinates coordinates;
    //Contains value defined in ConeColor-Enum:
    uint8_t color;
  };
  /*!
  * ROS topic callback method.
  * @param message the received message.
  */
  void topicCallback(const see_camera_processing::ImageCones &message);

  void calculateHomography(std::vector<cv::Rect> &boxes, std::vector<cv::Point2f> &coordinates, cv::Mat homographyMatrix);

  void calculateConversion(const see_camera_processing::ImageCones &message, std::vector<CoordinateColorPair> &coordinates);


  void calculateCovariance(cv::Mat &covarianceMatrix, float x, float y);

  //! ROS node handle.
  ros::NodeHandle &nodeHandle;

  //! ROS topic subscriber.
  ros::Subscriber subscriber;

  //! ROS topic publisher.
  ros::Publisher publisher;
};
