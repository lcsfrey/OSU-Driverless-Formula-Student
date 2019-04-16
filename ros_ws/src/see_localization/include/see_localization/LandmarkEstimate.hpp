#pragma once

#include <Eigen/Dense>
#include <math.h>
#include "see_localization/GlobalPose.hpp"
#include "see_localization/Cone.hpp"

class LandmarkEstimate{
    public:
        Eigen::Vector2f mean;
        Eigen::Matrix2f cov;
	std::vector<float> tpv;
        int ex_ind;                    //existence indicator: how many times could i observe the feature in a row
        double correspondence_prob;
        Eigen::Vector2f z_pred;
        Eigen::Matrix2f K;
        Eigen::Matrix2f Q;
        Eigen::Matrix2f R;
        Eigen::Matrix2f G;
        bool isBest;
	bool associated;//used in visualization to show what landmarks are being associated together

        LandmarkEstimate();
        bool isInPerceptualRange(int r_perception, int phi_perception, GlobalPose &mypose);
        Eigen::Vector2f g(GlobalPose &gp);
        Eigen::Vector2f gInverse(Eigen::Vector2f &z, GlobalPose &gp);
        Eigen::Matrix2f calcJacobian(GlobalPose &gp);
        void calcQ();
        void calcCorrespondenceWeight(Eigen::Vector2f &zt_in);
        void updateLandmarkEstimate(Eigen::Vector2f &zt_in);
        double getCorrespondenceProb();
	void setProbabilityVector(std::vector<float> color_vector);

};
