#pragma once

#include <std_msgs/String.h>
#include "cone.hpp"

class coneMap
{
public:

	vector<cone> cones_;

  /*!
   * Constructor
   */
	coneMap() : coneMap(vector<cone>()){};
	coneMap(vector<cone> c) : cones_(c){};
  }

  /*!
   * Destructor.
   */
  ~coneMap();
};
