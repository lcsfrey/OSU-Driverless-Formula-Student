#include "see_localization/LandmarkEstimate.hpp"

// void LandmarkEstimate(Eigen::Vector2f m, Eigen::Matrix2f c, int ei){
//     mean = m;
//     cov = c;
//     ex_ind = ei;
// }

LandmarkEstimate::LandmarkEstimate() {
	Eigen::Matrix2f Rt;
	Rt(0, 0) = 0.001;
	Rt(0, 1) = 0;
	Rt(1, 0) = 0;
	Rt(1, 1) = 0.001;
	R = Rt;
	mean << 0, 0;
	cov << 0, 0, 0, 0;
	ex_ind = 0;
	correspondence_prob = 0.0;
	K << 0, 0, 0, 0;
	Q << 0, 0, 0, 0;
	G << 1.0, 0.0, 0.0, 1.0;
	isBest = false;
	associated = false;
}

bool LandmarkEstimate::isInPerceptualRange(int r_perception, int phi_perception, GlobalPose &mypose) {
	float r_landmark = sqrt(pow((mean(0) - mypose.x_global), 2) + pow((mean(1) - mypose.y_global), 2));
	float phi_landmark = atan2((mean(1) - mypose.y_global), (mean(0) - mypose.x_global)) - mypose.theta_global;
	if ((r_landmark < r_perception) && (abs(phi_landmark) < abs(phi_perception))) {
		return true;
	} else {
		return false;
	}
}

Eigen::Vector2f LandmarkEstimate::g(GlobalPose &gp) {
	Eigen::Vector2f z;
	z(0) = mean(0) - gp.x_global;
	z(1) = mean(1) - gp.y_global;
	return z;
}

Eigen::Vector2f LandmarkEstimate::gInverse(Eigen::Vector2f &z, GlobalPose &gp) {
	Eigen::Vector2f gp_mean;
	gp_mean(0) = z(0) + gp.x_global;
	gp_mean(1) = z(1) + gp.y_global;
	return gp_mean;
}

Eigen::Matrix2f LandmarkEstimate::calcJacobian(GlobalPose &gp) {
	double q = pow((mean(0) - gp.x_global),2) + pow((mean(1) - gp.y_global),2);
	G(0,0) = (mean(0)-gp.x_global)/sqrt(q);
	G(0,1) = (mean(1)-gp.y_global)/sqrt(q);
	G(1,0) = -((mean(1)-gp.y_global)/q);
	G(1,1) = (mean(0)-gp.x_global)/q;
	
	return G;
}

void LandmarkEstimate::calcQ() {
	Q = G.transpose() * cov * G + R;
}

void LandmarkEstimate::calcCorrespondenceWeight(Eigen::Vector2f &zt_in) {
	Eigen::Matrix2f x = 2 * M_PI * Q;

	correspondence_prob = (pow(x.determinant(), -0.5)
			* exp(-0.5 * (double) ((zt_in - z_pred).transpose() * Q.inverse() * (zt_in - z_pred))));
}

void LandmarkEstimate::updateLandmarkEstimate(Eigen::Vector2f &zt_in) {
	K = cov * G * Q.inverse();
	Eigen::Vector2f tran = (zt_in - z_pred).transpose();//could be a logic error. Changed to get code to build.
	mean = mean + K * tran;
	//mean = (mean + K * (zt_in - z_pred).transpose());
	cov = (Eigen::MatrixXf::Identity(2, 2) - K * G.transpose()) * cov;
	ex_ind++;
}

double LandmarkEstimate::getCorrespondenceProb() {
	return correspondence_prob;
}

void LandmarkEstimate::setProbabilityVector(std::vector<float> color_vector){
	tpv = color_vector;
}
