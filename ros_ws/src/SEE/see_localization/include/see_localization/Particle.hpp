#pragma once
//#include "Map.hpp"
#include <see_localization/Sensor.hpp>
#include "see_localization/GlobalPose.hpp"
#include "see_localization/SLAMmath.hpp"
#include "see_localization/LandmarkEstimate.hpp"

class Particle {
public:
	GlobalPose gp_p;
	std::vector<LandmarkEstimate> le_p; //consists of mean, covariance matrix and counter
	float w_p;             //probability of how close the particle is to reality
	int N;                  //overall number of all landmarks the particle holds
	int r_perception = 5;
	int phi_perception = 60;

	Particle();
	void posePrediction(GlobalPose &odometry_commands);
	void addNewLandmarkEstimate(Eigen::Vector2f &zt, std::vector<float> color);
	void updateAllLandmarkEstimates(Eigen::Vector2f &zt_in, std::vector<float> color);
};
