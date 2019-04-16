/*
 * SLAM.cpp
 *
 *  Created on: 23.11.2018
 */
#include <iostream>
#include "see_localization/SLAM.hpp"
using namespace std;
SLAM::SLAM(){}

SLAM::SLAM(std::vector<Cone> new_cones, std::vector<Particle> my_particles, GlobalPose gp_slam) {
	observed_cones = new_cones;
	particles = my_particles;
	odometry_commands_ = gp_slam;
	
}

SLAM::~SLAM() {
}


std::vector<Particle> SLAM::runSLAM() {

	for(auto& p:particles){//used for visualization
		for(auto& lm: p.le_p){
			lm.associated = false;//reset associated values
		}
	}

	if (observed_cones.empty()) {
		//no sensor
	} else {
		for(auto& mycone : observed_cones){
			Eigen::Vector2f observed_cone_vector = mycone.calcVector();
			for (auto& single_particle : particles) {
				single_particle.posePrediction(odometry_commands_); //sample pose
				single_particle = computeWeights(observed_cone_vector, single_particle, mycone);
				single_particle.updateAllLandmarkEstimates(observed_cone_vector, mycone.type_probability_vector_); 
			}
		}
		particles = low_variance_resampling(particles); //low_variance_resampling step
	}
	
	return particles;
}

std::vector<Particle> SLAM::low_variance_resampling(std::vector<Particle> r_particles) {
	cout << "in resample" << endl;
	r_particles = normalize(r_particles);
	std::vector<Particle> St;
	std::random_device rd;
	std::mt19937 eng(rd());
	std::uniform_real_distribution<> distr(0.0, (1.0 / float(r_particles.size())));
	float r = distr(eng);
	float c = r_particles[0].w_p;
	int i = 0;
	for (unsigned int m = 1; m <= r_particles.size(); m++) { 
		float u = r + (float(m) - 1.0) * (1.0 / float(r_particles.size()));
		while (u > c) {
			i++;
			c = c + r_particles[i].w_p;
		}
		St.push_back(r_particles[i]);
	}
	return St;
}

std::vector<Particle> SLAM::normalize(std::vector<Particle> n_particles){
	double sum = 0;
	for(auto& p: n_particles){
        	sum += p.w_p;
	}
	for(auto& p1: n_particles){
    		p1.w_p = (p1.w_p)/(sum);//normalize
	}
        return n_particles;
}

Particle SLAM::computeWeights(Eigen::Vector2f zt, Particle myparticle, Cone mycone){
	for(auto& landmark: myparticle.le_p){
		landmark.z_pred = landmark.g(myparticle.gp_p);//predict landmark through function g
		landmark.calcJacobian(myparticle.gp_p);//calculate jacobian for landmark
		landmark.calcQ();//calculate measerment covariance
		landmark.calcCorrespondenceWeight(zt);//calculate correspondence probability
	}
	return myparticle;
}
