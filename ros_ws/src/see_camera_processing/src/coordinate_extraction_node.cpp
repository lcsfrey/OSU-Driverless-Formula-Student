#include <ros/ros.h>
#include "see_camera_processing/CoordinateExtraction.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "see_camera_processing_coordinate_extraction");
  ros::NodeHandle nodeHandle("/see_camera_processing");

  CoordinateExtraction rosPackageTemplate(nodeHandle);

  ros::spin();

  return 0;
}

CoordinateExtraction::CoordinateExtraction(ros::NodeHandle &nodeHandle)
    : nodeHandle(nodeHandle)
{
  std::string subscribed_topic = "cone_boxes";
  subscriber = nodeHandle.subscribe(subscribed_topic, 1, &CoordinateExtraction::topicCallback, this);

  std::string published_topic = "see_camera_processing_publication";
  publisher = nodeHandle.advertise<see_camera_processing::ConeContainer>(published_topic, 1);

  ROS_INFO("started node see_camera_processing_coordinate_extraction");
}

CoordinateExtraction::~CoordinateExtraction()
{
}

void CoordinateExtraction::topicCallback(const see_camera_processing::ImageCones &message)
{
  see_camera_processing::ConeContainer msgConeContainer;
  
  msgConeContainer.timestamp = message.timestamp;

  int numOfCones = message.cones.size();
  msgConeContainer.count = numOfCones;
  msgConeContainer.cones.resize(numOfCones);

  //Convert the Bounding Boxes to RealWorldCoordinates
  std::vector<CoordinateColorPair> realWorldCones;
  calculateConversion(message, realWorldCones);

  //TODO: This is Bad Practie, maybe add a constant "EnumSize = 6" to ConeColor.msg
  //As defined in ConeColor.msgConeContainer
  int possibleConeColors = 6;
  //Loop over all Cones in the message
  for (int i = 0; i < numOfCones; i++)
  {
    msgConeContainer.cones[i].cone_color.resize(possibleConeColors);
    //Init all colors with probability 0%:
    for (int j = 0; j < possibleConeColors; j++)
    {
      msgConeContainer.cones[i].cone_color[j].color = j;
      msgConeContainer.cones[i].cone_color[j].probability = 0.0f;
    }
    //Change probability of detected color to 100%:
    msgConeContainer.cones[i].cone_color[realWorldCones[i].color].probability = 1.0f;
    msgConeContainer.cones[i].coordinates = realWorldCones[i].coordinates;

    //Generate Covariance for each cone
    cv::Mat covarianceMatrix = cv::Mat_<float>(2, 2);
    calculateCovariance(covarianceMatrix, realWorldCones[i].coordinates.x, realWorldCones[i].coordinates.y);
    msgConeContainer.cones[i].covariance_matrix.xx = covarianceMatrix.at<float>(0, 0);
    msgConeContainer.cones[i].covariance_matrix.xy = covarianceMatrix.at<float>(0, 1);
    msgConeContainer.cones[i].covariance_matrix.yx = covarianceMatrix.at<float>(1, 0);
    msgConeContainer.cones[i].covariance_matrix.yy = covarianceMatrix.at<float>(1, 1);

    msgConeContainer.cones[i].cone_probability = 1.0f;
  }

  publisher.publish(msgConeContainer);
}

void CoordinateExtraction::calculateCovariance(cv::Mat &covarianceMatrix, float x, float y)
{
  //longestEigenvector has the same direction as the major ellipse axis
  std::vector<float> eigenvectorLongAxis = std::vector<float>();
  cv::Point2f eigenVectorLong = cv::Point(SCALE_FACTOR_LONG_AXIS * x, SCALE_FACTOR_LONG_AXIS * y);
  //perpendicular eigenvector:
  cv::Point2f eigenVectorShort = cv::Point(-y, x);
  //largtestEigenvalue, belongs to "longest" Eigenvector:
  //from: http://www.visiondummy.com/wp-content/uploads/2014/04/error_ellipse.cpp
  //                         (         x²                   +              y²             ) / chi_square²
  float eigenvalueLongAxis = (pow(eigenVectorLong.x, 2) + pow(eigenVectorLong.y, 2) / pow(CHI_SQUARE_FACTOR, 2));
  //smallestEigenvalue, belongs to "shorter" Eigenvector:
  float shortAxisLength = tan(SCALE_ANGLE) * sqrt(pow(x, 2) + pow(y, 2));
  float eigenvalueShortAxis = pow(shortAxisLength / CHI_SQUARE_FACTOR, 2);
  cv::Mat MMat = (cv::Mat_<float>(2, 2) << eigenvalueShortAxis, 0.0f, 0.0f, eigenvalueLongAxis);
  cv::Mat SMat = (cv::Mat_<float>(2, 2) << eigenVectorShort.x, eigenVectorLong.x, eigenVectorShort.y, eigenVectorLong.y);
  cv::Mat SInverse;
  cv::invert(SMat, SInverse);
  covarianceMatrix = (SMat * (MMat * SInverse));
}

void CoordinateExtraction::calculateHomography(std::vector<cv::Rect> &boxes, std::vector<cv::Point2f> &coordinates, cv::Mat homographyMatrix)
{
  int boxCount = boxes.size();
  std::vector<cv::Point2f> centerPoints;
  for (int i = 0; i < boxCount; i++)
  {
    //Find central point in box to transform:
    //TODO: Move literals to YAML-File
    centerPoints.push_back(cv::Point2f(boxes[i].x + boxes[i].width / 0.5, boxes[i].y + boxes[i].height * 0.7));
  }
  std::vector<cv::Point2f> newPoints;
  cv::perspectiveTransform(centerPoints, newPoints, homographyMatrix);
  return;
}

void CoordinateExtraction::calculateConversion(const see_camera_processing::ImageCones &message, std::vector<CoordinateColorPair> &coordinates)
{
  int factor = 30000;
  //Factor for large cone should be 1.553 times larger because the cone is 1.553 times larger
  int bigOrangeFactor = 1.553 * factor;
  int cameraHeight = 70;
  //Point outside of image where distance would be 0:
  cv::Point2f centralPoint(message.imageWidth / 2, message.imageHeight * 1.5);
  std::vector<see_camera_processing::ImageCone> boxes = message.cones;

  int boxCount = boxes.size();
  coordinates.resize(boxCount);
  float distance;
  float angleRad;
  cv::Point2f coneCenter;
  for (int i = 0; i < boxCount; i++)
  {
    //Calculate Distance based on pixel height
    //Note that this is the 3d Euclidian distance
    if (boxes[i].coneType.color == boxes[i].coneType.CONE_COLOR_BIG_ORGANGE)
    {
      distance = bigOrangeFactor / boxes[i].boxHeight;
    }
    else
    {
      distance = factor / boxes[i].boxHeight;
    }
    // a² + b² = c² => a² = c² - b² => a = sqrt(c² - b²)
    distance = sqrt(distance * distance + cameraHeight * cameraHeight);
    //Calculate Angle
    coneCenter.x = boxes[i].coordinatesTopLeft.x + boxes[i].boxWidth / 0.5;
    coneCenter.y = boxes[i].coordinatesTopLeft.y + boxes[i].boxHeight * 0.7;

    // atan(y,x) with
    // y = horizontal distance from centralPoint, can be pos or neg
    // x = vertival distance from centralPoint, pos
    angleRad = atan2(centralPoint.x - coneCenter.x, centralPoint.y - coneCenter.y);
    //Polar to Carthesian:
    coordinates[i].color = boxes[i].coneType.color;
    coordinates[i].coordinates.y = distance * sin(angleRad);
    coordinates[i].coordinates.x = distance * cos(angleRad);
  }
}
