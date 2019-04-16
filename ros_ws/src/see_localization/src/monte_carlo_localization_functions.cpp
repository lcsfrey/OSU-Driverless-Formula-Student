//#define _USE_MATH_DEFINES
//#include <math.h>
//#include <vector>
//#include "Particle.hpp"
//#include "GlobalPose.hpp"
//
//// float gaussianProbability(float x, float mu, float v){
////     return (1/sqrt(2*M_PI*v)*exp(-0.5 * (pow((x-mu),2.0)/v));
//// }
//
//// std::vector<float> sampleFromGaussian(number_samples, mu, v){
////     std::random_device rd;
////     std::mt19937 e2(rd());
////     std::normal_distribution<> dist(mu,sqrt(v));
////     std::vector<float> s;
////     for(int i = 0; i < number_samples, i++){
////         //gaussian_samples(i) = sqrt(v) * s(i) + mu;
////         sample = std::round(dist(e2));
////         s.push_back(sample);
////     }
////     return s;
//// }
//
//// std::vector<float> initParticles(number_particles,mu_x,v_x,mu_y,v_y){
////     std:vector<Particle> p_vector;
////     for(int i = 0; i < number_samples, i++){
////         Particle p;
////         p.x_p = sampleFromGaussian(number_particles,mu_x,v_x);
////         p.y_p = sampleFromGaussian(number_particles,mu_y,v_y);
////         p.weight_ = 1/number_particles;
////         p_vector.push_back(p);
////     }
////     return p_vector;
//// }
//
//// void normalizeWeights(std::vector<Particle> myparticles)
//// {
////     mysize = myparticles.size();
////     int sum = 0;
////     for (int i = 0; i < mysize; i++){
////         sum+=myparticles(i).weight_;
////     }
////     for (int i = 0; i < mysize; i++){
////         myparticles(i).weight_ = myparticles(i).weight_ / sum;
////     }
//// }
//
//void posePrediction(std:vector<Particle> myparticles, GlobalPose xt_1, GlobalPose pos_est)
//{
//    GlobalPose odometry_commands;
//    odometry_commands.x_global = pos_est.x_global - xt_1.x_global;
//    odometry_commands.y_global = pos_est.y_global - xt_1.y_global;
//    odometry_commands.theta_global = pos_est.theta_global - xt_1.theta_global;
//
//    // for(int i = 0; i < myparticles.size(); i++){
//    //     myparticles(i).x_p += odometry_commands.x_global;
//    //     myparticles(i).y_p += odometry_commands.y_global;
//    //     myparticles(i).theta_global = odometry_commands.theta_global;
//    // }
//
//    std::vector<float> x_val = sampleFromGaussian(myparticles.size(), odometry_commands.x_global, v);
//    std::vector<float> y_val = sampleFromGaussian(myparticles.size(), odometry_commands.y_global, v);
//    std::vector<float> theta_val = sampleFromGaussian(myparticles.size(), odometry_commands.theta_global, v);
//
//    // for(int i = 0; i < myparticles.size(); i++){
//    //     myparticles(i).x_p = x_val(i);
//    //     myparticles(i).y_p = y_val(i);
//    //     myparticles(i).theta_p = theta_val(i);
//    // }
//
//}
//
//void posePrediction(std:vector<Particle> myparticles, GlobalPose xt_1, GlobalPose pos_est)
//{
//    GlobalPose odometry_commands;
//    odometry_commands.x_global = pos_est.x_global - xt_1.x_global;
//    odometry_commands.y_global = pos_est.y_global - xt_1.y_global;
//    odometry_commands.theta_global = pos_est.theta_global - xt_1.theta_global;
//
//    // for(int i = 0; i < myparticles.size(); i++){
//    //     myparticles(i).x_p += odometry_commands.x_global;
//    //     myparticles(i).y_p += odometry_commands.y_global;
//    //     myparticles(i).theta_global = odometry_commands.theta_global;
//    // }
//
//    std::vector<float> x_val = sampleFromGaussian(myparticles.size(), odometry_commands.x_global, v);
//    std::vector<float> y_val = sampleFromGaussian(myparticles.size(), odometry_commands.y_global, v);
//    std::vector<float> theta_val = sampleFromGaussian(myparticles.size(), odometry_commands.theta_global, v);
//
//    // for(int i = 0; i < myparticles.size(); i++){
//    //     myparticles(i).x_p = x_val(i);
//    //     myparticles(i).y_p = y_val(i);
//    //     myparticles(i).theta_p = theta_val(i);
//    // }
//
//}
//
//
//void correctionStep(std::vector<Particle> myparticles){}
//
////this function is called for every observed landmark, so in this case for every observed cone
//void FastSLAM1unknown_correspondences(Cone zt, GlobalPose xt_1, GlobalPose pos_est, std::vector<Particle> St_1){
//    for (int m = 0; m < St_1.size(); m++){
//        posePrediction()
//
//
//    }
//
//}
//
//LandmarkCovariance calcInverse(LandmarkCovariance lc){
//    LandmarkCovariance invlc;
//    inverted_det = 1/((lc.xx * lc.yy) - (lc.xy * lx.yx));
//    invlc.xx = inverted_det * lc.yy;
//    invlc.xy = inverted_det * (-lc.xy);
//    invlc.yx = inverted_det * (-lc.yx);
//    invlc.yy = inverted_det * lc.xx;
//    return invlc;
//}
//
//LandmarkCovariance calcTransposed(LandmarkCovariance lc){
//    LandmarkCovariance translc;
//    translc.xx = lc.xx;
//    translc.xy = lc.yx;
//    translc.yx = lc.xy;
//    translc.yy = lc.yy;
//    return translc;
//}
//
//void calcWeight(mv, )
