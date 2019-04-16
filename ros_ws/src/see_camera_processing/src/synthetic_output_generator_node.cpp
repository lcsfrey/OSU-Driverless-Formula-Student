#include "see_camera_processing/synthetic_output_generator.hpp"
//#include <ros/ros.h>
#include <string>

OutputGenerator::OutputGenerator(ros::NodeHandle& nodeHandle) : nodeHandle(nodeHandle) {
  std::string published_topic = "see_camera_processing_publication";
  publisher = nodeHandle.advertise<see_camera_processing::ConeContainer>(published_topic, 1);
}

OutputGenerator::~OutputGenerator(){

}

void OutputGenerator::run(){
    initConePositions();
	offsetLineDistance();
    ros::Rate rate(60);

	double DistanceOfLine = 0.0;
	while(ros::ok()) {
        for (int n = 0; n < 20; n++) {
        if (compareDoubles2(cones[n][0],0.0)) {
            break;
        }

        if (cones[n][0] <= minViewCar) {
            for (int m = 0; m < 19; m++) {
            cones[m][0] = cones[m + 1][0];
            cones[m][1] = cones[m + 1][1];
            cones[m][2] = cones[m + 1][2];
            }
            cones[19][0] = 0.0;
            cones[19][1] = 0.0;
            cones[19][2] = 0.0;
        }


        DistanceOfLine = cones[n][0];
        DistanceOfLine = DistanceOfLine - (speedInMS / fps);
        cones[n][0] = DistanceOfLine;

        }

        LineDifference = LineDifference + (speedInMS / fps);

        if (compareDoubles2(LineDifference, ConesDistanceVertical)) {
            for (int p = 0; p < 20; p++) {
                if (compareDoubles2(cones[p][0], 0)) {
                cones[p][0] = cones[p - 1][0] + ConesDistanceVertical;
                cones[p][1] = ConesWitdh / 2;
                cones[p][2] = -ConesWitdh / 2;
                LineDifference = 0.0;
                break;
                }
            }
        }

        see_camera_processing::ConeContainer messageData = OutputGenerator::populateConeContainer(cones);
        publisher.publish(messageData);
        ROS_INFO_STREAM("Sending Cone Data");
        rate.sleep();
	}
}

void OutputGenerator::initConePositions() {
	int CountDistancesTogether = 0;
	int numberOfLines = (int) (viewDistance / ConesDistanceVertical);
	for (int e = 0; e < numberOfLines; e++) {
		CountDistancesTogether = CountDistancesTogether + ConesDistanceVertical;
		cones[e][0] = CountDistancesTogether;
		cones[e][1] = ConesWitdh / 2;
		cones[e][2] = -ConesWitdh / 2;
	}
}

void OutputGenerator::offsetLineDistance() {
	int NumberOfLines = (int)(viewDistance / ConesDistanceVertical);
	double ValueOfRoundedLines = NumberOfLines * ConesDistanceVertical;
	LineDifference = viewDistance - ValueOfRoundedLines;
}

bool OutputGenerator::compareDoubles2(double A, double B){
	double diff = A - B;
	return (diff < 0.001) && (-diff < 0.001);
}

see_camera_processing::ConeContainer OutputGenerator::populateConeContainer(double cones[][3]) {
    see_camera_processing::ConeContainer cont;
    for (int i = 0; i < 20; i++) {
        if (compareDoubles2(cones[i][0],0.0)) {
            break;
        }
        std::vector<float> tempVector = std::vector<float>();
        //Create Cone Objects
        //Left Cone
        see_camera_processing::Cone leftCone;
        leftCone.coordinates.x = cones[i][0];
        leftCone.coordinates.y = cones[i][1];
        tempVector = getMatrix(float(cones[i][0]),float(cones[i][1]));
        leftCone.covariance_matrix.xx = tempVector[0];
        leftCone.covariance_matrix.xy = tempVector[1];
        leftCone.covariance_matrix.yx = tempVector[2];
        leftCone.covariance_matrix.yy = tempVector[3];
        see_camera_processing::ConeColor colorObj;
        for(int j = 0; j < 6; j++) {
            colorObj.color = j;
            colorObj.probability = 0.0f;
            leftCone.cone_color.push_back(colorObj);
        }
        leftCone.cone_color[0].probability = 1.0f;
        leftCone.cone_probability = 1.0f;

        //Right Cone
        see_camera_processing::Cone rightCone;
        rightCone.coordinates.x = cones[i][0];
        rightCone.coordinates.y = cones[i][2];
        tempVector = getMatrix(float(cones[i][0]),float(cones[i][2]));
        rightCone.covariance_matrix.xx = tempVector[0];
        rightCone.covariance_matrix.xy = tempVector[1];
        rightCone.covariance_matrix.yx = tempVector[2];
        rightCone.covariance_matrix.yy = tempVector[3];
        for(int j = 0; j < 6; j++) {
            colorObj.color = j;
            colorObj.probability = 0.0f;
            rightCone.cone_color.push_back(colorObj);
        }
        rightCone.cone_color[1].probability = 1.0f;
        rightCone.cone_probability = 1.0f;
        cont.cones.push_back(rightCone);
        cont.cones.push_back(leftCone);
    }
    cont.count = cont.cones.size();
    cont.timestamp = ros::Time::now();
    return cont;
}

std::vector<float> OutputGenerator::getMatrix(float xCoordinate, float yCoordinate) {
    //longestEigenvector has the same direction as the major ellipse axis
    std::vector<float> eigenvectorLongAxis = std::vector<float>();
    eigenvectorLongAxis.push_back(float(SCALE_FACTOR_LONG_AXIS*xCoordinate));
    eigenvectorLongAxis.push_back(float(SCALE_FACTOR_LONG_AXIS*yCoordinate));
    //perpendicular eigenvector:
    std::vector<float> eigenvectorShortAxis = std::vector<float>();
    eigenvectorShortAxis.push_back(float(-yCoordinate));
    eigenvectorShortAxis.push_back(float(xCoordinate));
    //largtestEigenvalue, belongs to "longest" Eigenvector:
    //from: http://www.visiondummy.com/wp-content/uploads/2014/04/error_ellipse.cpp
    //                         (         x²                   +              y²             ) / chi_square²
    float eigenvalueLongAxis = (pow(eigenvectorLongAxis[0],2)+pow(eigenvectorLongAxis[0],2)/ pow(CHI_SQUARE_FACTOR,2));
    //smallestEigenvalue, belongs to "shorter" Eigenvector:
    float shortAxisLength = tan(SCALE_ANGLE)*sqrt(pow(xCoordinate,2)+pow(yCoordinate,2));
    float eigenvalueShortAxis = pow(shortAxisLength/CHI_SQUARE_FACTOR,2);
    //Create Matrix fom Eigenvalues/-vectors:
    //https://math.stackexchange.com/a/1119690
    float M[2][2] = {   {eigenvalueShortAxis,  0.0f},
                        {0.0f,                    eigenvalueLongAxis}};
    float S[2][2] = {{eigenvectorShortAxis[0], eigenvectorLongAxis[0]},
                     {eigenvectorShortAxis[1], eigenvectorLongAxis[1]}};
    float determinantOfS = S[0][0]*S[1][1]-S[0][1]*S[1][0];
    //Formula for the inverse of 2x2:
    float Si[2][2] = {  { S[1][1]/determinantOfS, -S[0][1]/determinantOfS},
                        {-S[1][0]/determinantOfS,  S[0][0]/determinantOfS}};
    //Multiply M with Si:
    float multi[2][2] = {{0.0f,0.0f},{0.0f,0.0f}};
    int i,j,k;
    for(i = 0; i < 2; ++i) {
        for(j = 0; j < 2; ++j) {
            for(k = 0; k < 2; ++k)
            {
                multi[i][j] += M[i][k] * Si[k][j];
            }
        }
    }
    //Multiply S with multi
    float erg[2][2] = {{0.0f,0.0f},{0.0f,0.0f}};
    for(i = 0; i < 2; ++i) {
        for(j = 0; j < 2; ++j) {
            for(k = 0; k < 2; ++k)
            {
                erg[i][j] += S[i][k] * multi[k][j];
            }
        }
    }
    std::vector<float> returnVector = std::vector<float>();
    for(i = 0; i < 2; ++i)
    for(j = 0; j < 2; ++j)
    {
        returnVector.push_back(erg[i][j]);
    }
    return returnVector;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "see_camera_processing");
  ros::NodeHandle nodeHandle("~");

  OutputGenerator rosPackageTemplate(nodeHandle);

  // @param double frequency [Hz] in which to repeat the loop
  //ros::Rate rate(60);
  while(ros::ok())
  {
    rosPackageTemplate.run();
    //Sleep is temporarily implementet in the endless loop
    // rate.sleep();
  }
  return 0;
}
