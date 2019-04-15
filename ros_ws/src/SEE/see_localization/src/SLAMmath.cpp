#include "see_localization/SLAMmath.hpp"


float gaussianProbability(float x, float mu, float v){
    return (float)(1/sqrt(2*M_PI*v)*exp(-0.5 * (pow((x-mu),2.0)/v)));
}

float sampleFromGaussian(float mu, float v){

    std::random_device rd;
    std::mt19937 e2(rd());
    std::normal_distribution<> dist(mu,sqrt(v));
    float sample = dist(e2);
    return sample;
}



// LandmarkCovariance calcInverse(LandmarkCovariance lc){
//     LandmarkCovariance invlc;
//     inverted_det = 1/((lc.xx * lc.yy) - (lc.xy * lx.yx));
//     invlc.xx = inverted_det * lc.yy;
//     invlc.xy = inverted_det * (-lc.xy);
//     invlc.yx = inverted_det * (-lc.yx);
//     invlc.yy = inverted_det * lc.xx;
//     return invlc;
// }

// LandmarkCovariance calcTransposed(LandmarkCovariance lc){
//     LandmarkCovariance translc;
//     translc.xx = lc.xx;
//     translc.xy = lc.yx;
//     translc.yx = lc.xy;
//     translc.yy = lc.yy;
//     return translc;
// }

// std::vector<float> g(std::vector<float> mu, GlobalPose gp){
//     std::vector<float> z;
//     z(1) = mu(1) - gp.x_global;
//     z(2) = mu(2) - gp.y_global;
//     return z;
// }

// Matrix calcJacobian(std::vector<float> mu, GlobalPose gp){
//     Matrix jacobian;
//     order = 1;
//     jacobian.xx = order*std::pow(mu(1),(order-1)) - 0;
//     jacobian.xy = 0 - 0;
//     jacobian.yx = 0 - 0;
//     jacobian.yy = 0 - order*std::pow(mu(2),(order-1));
//     return jacobian;
// }

// void FastSLAM1unknown_correspondences(Cone zt, GlobalPose odometry_commands, std::vector<Particle> St_1){
//     for (int m = 0; m < St_1.size(); m++){
//         St_1(m).posePrediction(odometry_commands);
//         for (int n = 0; n < St_1(m).N; n++){
//             std::vector<float> z_pred = g(St(m).le_p(n).mean, St_1(m).gp_p);
//             Matrix G = calcJacobian(St(m).le_p(n).mean, St_1(m).gp_p);
//             Matrix Q = calcTransposed(G) * St(m).le_p(n).cov * G + R_t;
//             float weight = abs(2*M_PI)
//         }


//     }

// }
