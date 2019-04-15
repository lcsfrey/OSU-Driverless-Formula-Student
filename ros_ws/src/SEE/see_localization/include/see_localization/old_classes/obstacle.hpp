#pragma once

class obstacle
{
public:

	float x_;
	float y_;


  /*!
   * Constructor
   */
	obstacle() : obstacle(0.0,0.0){};
	obstacle(float x, float y) : x_(x), y_(y) {};

  /*!
   * Destructor.
   */
  ~obstacle();
};
