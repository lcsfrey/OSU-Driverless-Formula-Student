#pragma once
//#include "see_localization/vehicle_pose_msg.h"

class GlobalPose{
    public:
        float x_global;
        float y_global;
        float theta_global;

        //GlobalPose() : vehilce_pose_msg(){};
        GlobalPose() : GlobalPose(0.0,0.0,0.0){};
	    GlobalPose(float x_g, float y_g, float theta_g) : x_global(x_g), y_global(y_g), theta_global(theta_g) {};

        ~GlobalPose(){};
};
