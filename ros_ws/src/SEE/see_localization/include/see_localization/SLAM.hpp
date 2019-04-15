/*
 * SLAM.h
 *
 *  Created on: 23.11.2018
 */

#ifndef SEE_LOCALIZATION_SLAM_H_
#define SEE_LOCALIZATION_SLAM_H_

#include <see_localization/Sensor.hpp>
//#include <vector>
#include "see_localization/Particle.hpp"
//#include <random>


class SLAM {
	std::vector<Particle> particles; 
	std::vector<Cone> observed_cones;
	GlobalPose odometry_commands_;


public:
	SLAM();
	SLAM(std::vector<Cone> new_cones, std::vector<Particle> my_particles, GlobalPose gp_slam);
	virtual ~SLAM();
	std::vector<Particle> runSLAM();
	std::vector<Particle> normalize(std::vector<Particle> n_particles);
	std::vector<Particle> low_variance_resampling(std::vector<Particle> particles);
	Particle computeWeights(Eigen::Vector2f zt, Particle myparticle, Cone mycone);

};

#endif /* SEE_LOCALIZATION_SLAM_H_ */
