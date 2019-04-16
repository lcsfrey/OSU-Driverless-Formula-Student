#include "see_localization/Particle.hpp"

Particle::Particle(){
	gp_p.x_global = 0;
	gp_p.y_global = 0;
	gp_p.theta_global = 0;
	N = 0;
	w_p = 1.0/50.0;
}

void Particle::posePrediction(GlobalPose &odometry_commands) {
	gp_p.x_global = sampleFromGaussian(odometry_commands.x_global, 0.01);
	gp_p.y_global = sampleFromGaussian(odometry_commands.y_global, 0.01);
	gp_p.theta_global = sampleFromGaussian(odometry_commands.theta_global, 0.01);
}

void Particle::addNewLandmarkEstimate(Eigen::Vector2f &zt_in, std::vector<float> color) {
	LandmarkEstimate le;
	le.mean = le.gInverse(zt_in, gp_p);
	le.G = le.calcJacobian(gp_p);
	le.cov = le.G.inverse() * le.R * le.G.inverse().transpose();
	le.ex_ind = 1;
	le.setProbabilityVector(color);
	le_p.push_back(le);
}

void Particle::updateAllLandmarkEstimates(Eigen::Vector2f &zt_in, std::vector<float> color) {
	N = le_p.size();
	bool has_cor = false;
	float max_corres_prob = 1e-300; //usually 1/N
	int max_corres_idx = 0;
	for (int h = 0; h < N; h++) {
		if (le_p[h].correspondence_prob > max_corres_prob) {
			max_corres_prob = le_p[h].correspondence_prob;
			max_corres_idx = h;
			has_cor = true;
		}
	}
	if (has_cor == false) {
		addNewLandmarkEstimate(zt_in, color);
	} else {
		le_p[max_corres_idx].isBest = true;
		le_p[max_corres_idx].associated = true;//used in visualization
	}
	for(unsigned int i = 0; i < le_p.size(); i++){
		if (le_p[i].isBest) {
			le_p[i].updateLandmarkEstimate(zt_in);
			w_p = le_p[i].getCorrespondenceProb(); //set particle weight to highest correspondence probability
			le_p[i].isBest = false;
		} /*else if (le_p[i].isInPerceptualRange(r_perception, phi_perception, gp_p)) {
			le_p[i].ex_ind--;
			if (le_p[i].ex_ind < 0) {
				le_p.erase(le_p.begin()+i); //erase landmark if we should have observed it altough we didn't
			}
		}*/
	}

}
