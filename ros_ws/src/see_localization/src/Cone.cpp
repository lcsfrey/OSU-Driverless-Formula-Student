#include "see_localization/Cone.hpp"

Cone::Cone(float x, float y, std::vector<float> tpv) {
	x_ = x;
	y_ = y;
	type_probability_vector_ = tpv;
}

Cone::Cone() {
	x_ = 0;
	y_ = 0;

}

Eigen::Vector2f Cone::calcVector() {
	Eigen::Vector2f cone_vec;
	cone_vec(0) = x_;
	cone_vec(1) = y_;
	return cone_vec;
}
