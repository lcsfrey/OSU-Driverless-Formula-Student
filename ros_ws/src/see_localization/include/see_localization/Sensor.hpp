/*
 * Sensor.h
 *
 *  Created on: 23.11.2018
 */

#ifndef SEE_LOCALIZATION_SENSOR_H_
#define SEE_LOCALIZATION_SENSOR_H_
#include <vector>
#include "see_localization/Cone.hpp"

class Sensor {

	float r_perception;		//in m
	float phi_perception;	//in degree
//public:
	std::vector<Cone> cones;

public:
	float& get_r();
	float& get_phi();
	std::vector<Cone> get_cones();
	Sensor(float radius, float angle, std::vector<Cone> cone_list);
	Sensor(float radius, float angle);
	Sensor();
	~Sensor();
};

#endif /* SEE_LOCALIZATION_SENSOR_H_ */
