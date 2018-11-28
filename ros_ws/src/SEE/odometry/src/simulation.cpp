#include <ros/ros.h>
#include <filter/bootstrapfilter.h>
#include <sensor_msgs/Imu.h>

void accelerometerCallback(const sensor_msgs::Imu::ConstPtr& msg){
    ROS_INFO_STREAM(msg->orientation.x << "\n" << msg->orientation.y << "\n" << msg->orientation.z << "\n");
}

int main(int argc, char ** argv){
    ros::init(argc, argv, "particle_filter_testing");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("imu", 1000, &accelerometerCallback);
    ros::spin();

    // ColumnVector sysNoise_Mu(3);
    // sysNoise_Mu(1) = 0.0;
    // sysNoise_Mu(2) = 0.0;
    // sysNoise_Mu(3) = 0.0;

    // SymmetricMatrix sysNoise_Cov(3);
    // sysNoise_Cov(1,1) = pow(0.01,2);
    // sysNoise_Cov(1,2) = 0.0;
    // sysNoise_Cov(1,3) = 0.0;
    // sysNoise_Cov(2,1) = 0.0;
    // sysNoise_Cov(2,2) = pow(0.01,2);
    // sysNoise_Cov(2,3) = 0.0;
    // sysNoise_Cov(3,1) = 0.0;
    // sysNoise_Cov(3,2) = 0.0;
    // sysNoise_Cov(3,3) = pow(0.03,2);

    // Gaussian system_uncertainty(sysNoise_Mu, sysNoise_Cov);
}
