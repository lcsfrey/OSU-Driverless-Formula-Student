#pragma once
#include "Map.hpp"
#include "GlobalPose.hpp"
//#include "see_localization/Cone.hpp"

class DataGenerator{
    public:
        Map m;
        GlobalPose gp;
        Cone myCone;

        DataGenerator(){
            std::vector<Cone> cones;

            std::vector<float> r;
            r.push_back(0.0);
            r.push_back(0.0);
            r.push_back(0.0);
            r.push_back(0.0);
            r.push_back(1.0);
            //ROS_INFO("Hallöchen");

            std::vector<float> l;
            l.push_back(1.0);
            l.push_back(0.0);
            l.push_back(0.0);
            l.push_back(0.0);
            l.push_back(0.0);

            std::vector<float> be;
            be.push_back(0.0);
            be.push_back(1.0);
            be.push_back(0.0);
            be.push_back(0.0);
            be.push_back(0.0);

            //skidpad map
            cones.push_back(Cone(0.0,1.5,be));
            cones.push_back(Cone(10.625,1.5,l));
            cones.push_back(Cone(13.543,2.08,l));
            cones.push_back(Cone(16.017,3.733,l));
            cones.push_back(Cone(17.67,6.207,l));
            cones.push_back(Cone(18.25,9.125,l));
            cones.push_back(Cone(17.67,12.043,l));
            cones.push_back(Cone(16.017,14.517,l));
            cones.push_back(Cone(13.543,16.17,l));
            cones.push_back(Cone(10.625,16.75,l));
            cones.push_back(Cone(7.707,16.17,l));
            cones.push_back(Cone(5.233,14.517,l));
            cones.push_back(Cone(3.58,12.043,l));
            cones.push_back(Cone(3.0,9.125,l));
            cones.push_back(Cone(3.58,6.207,l));
            cones.push_back(Cone(5.233,3.733,l));
            cones.push_back(Cone(7.707,2.08,l));
            cones.push_back(Cone(0.0,-1.5,be));
            cones.push_back(Cone(10.625,-1.5,r));
            cones.push_back(Cone(13.543,-2.08,r));
            cones.push_back(Cone(16.017,-3.733,r));
            cones.push_back(Cone(17.67,-6.207,r));
            cones.push_back(Cone(18.25,-9.125,r));
            cones.push_back(Cone(17.67,-12.043,r));
            cones.push_back(Cone(16.017,-14.517,r));
            cones.push_back(Cone(13.543,-16.17,r));
            cones.push_back(Cone(10.625,-16.75,r));
            cones.push_back(Cone(7.707,-16.17,r));
            cones.push_back(Cone(5.233,-14.517,r));
            cones.push_back(Cone(3.58,-12.043,r));
            cones.push_back(Cone(3.0,-9.125,r));
            cones.push_back(Cone(3.58,-6.207,r));
            cones.push_back(Cone(5.233,-3.733,r));
            cones.push_back(Cone(7.707,-2.08,r));
            cones.push_back(Cone(20.442,5.059,r));
            cones.push_back(Cone(21.25,9.125,r));
            cones.push_back(Cone(20.442,13.191,r));
            cones.push_back(Cone(18.138,16.638,r));
            cones.push_back(Cone(14.691,18.942,r));
            cones.push_back(Cone(10.625,19.75,r));
            cones.push_back(Cone(6.559,18.942,r));
            cones.push_back(Cone(3.112,16.638,r));
            cones.push_back(Cone(0.808,13.191,r));
            cones.push_back(Cone(0.0,9.125,r));
            cones.push_back(Cone(0.808,5.059,r));
            cones.push_back(Cone(20.442,-5.059,l));
            cones.push_back(Cone(21.25,-9.125,l));
            cones.push_back(Cone(20.442,-13.191,l));
            cones.push_back(Cone(18.138,-16.638,l));
            cones.push_back(Cone(14.691,-18.942,l));
            cones.push_back(Cone(10.625,-19.75,l));
            cones.push_back(Cone(6.559,-18.942,l));
            cones.push_back(Cone(3.112,-16.638,l));
            cones.push_back(Cone(0.808,-13.191,l));
            cones.push_back(Cone(0.0,-9.125,l));
            cones.push_back(Cone(0.808,-5.059,l));
            cones.push_back(Cone(21.25,1.5,be));
            cones.push_back(Cone(21.25,-1.5,be));

            //skidpadMap = Map(cones, ros::Time::now(), false);
            Map skidpadMap(cones, ros::Time::now(), false);
            GlobalPose glopose;


            m = skidpadMap;
            gp = glopose;
        };
	    DataGenerator(Map mymap, GlobalPose myPose) : m(mymap), gp(myPose) {};

        ~DataGenerator(){};


};
