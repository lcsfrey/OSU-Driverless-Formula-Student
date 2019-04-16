#pragma once

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include <vector>
#include "see_camera_processing/ConeContainer.h"
#include "see_camera_processing/Cone.h"
#include "see_camera_processing/Coordinates.h"
#include "see_camera_processing/CovarianceMatrix.h"
#include "see_camera_processing/ConeColor.h"

//How long should the ellipse be?
#define SCALE_FACTOR_LONG_AXIS 0.05
//How broad should the ellipse be? (in rad)
#define SCALE_ANGLE 0.0349066
#define CHI_SQUARE_FACTOR 5.991


/*!
 * Main class for the node to handle the ROS interfacing.
 */
class OutputGenerator
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  OutputGenerator(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~OutputGenerator();

  /*!
   * Method for running the ROS node algorithm.
   */
  void run();
  


private:

  //Code For Synthetic Data Generation
  int fps = 60;
  double speedInMS = 0.5;
  double viewDistance = 50.0;
  double minViewCar = 3.0;
  double ConesWitdh = 3.0;
  double ConesDistanceVertical = 5.0;
  double cones[20][3];
  double LineDifference = 0.0;

  bool compareDoubles2(double A, double B);
  void initConePositions();
  void offsetLineDistance();
  see_camera_processing::ConeContainer populateConeContainer(double cones[][3] );
  /*!
   *Returns calculated the Covariance Matrix for the specified coordinates
   */
  std::vector<float> getMatrix(float xCoordinate, float yCoordinate);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle;

  //! ROS topic publisher.
  ros::Publisher publisher;
};
