/*
 * Sensor.cpp
 *
 *  Created on: 23.11.2018
 */

#include <see_localization/Sensor.hpp>

float& Sensor::get_r(){
	return r_perception;
}

float& Sensor::get_phi(){
	return phi_perception;
}

std::vector<Cone> Sensor::get_cones(){
	return cones;
}

Sensor::Sensor(float radius, float angle){
	r_perception = radius;
	phi_perception = angle;
}

Sensor::Sensor() :
		Sensor(5, 60) {
}

Sensor::Sensor(float radius, float angle, std::vector<Cone> cone_list) {
	r_perception = radius;
	phi_perception = angle;
	cones = cone_list;
}

Sensor::~Sensor() {

}

