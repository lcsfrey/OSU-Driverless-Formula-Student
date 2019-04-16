#pragma once

#include <vector>
#include <Eigen/Dense>
//#include "see_localization/cone_msg.h"

//class Cone : public cone_msg
class Cone {
public:

	float x_;
	float y_;
	std::vector<float> type_probability_vector_;

	/*!
	 * Constructor
	 */
	//Cone() : Cone(vector<float>(),0.0,0.0){};


	//Cone(float x, float y, vector<float> tpv);

	Cone(float x, float y, std::vector<float> tpv); //{
//		x_ = x;
//		y_ = y;
//		type_probability_vector_ = tpv;
//	};

	Cone(); //{
//		x_ = 0;
//		y_ = 0;

//	};

//	Cone();
//	//Cone() : cone_msg();
//	Cone(float x, float y, std::vector<float> tpv);
	Eigen::Vector2f calcVector();

	/*!
	 * Destructor.
	 */
	~Cone() {};
};
