#pragma once
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
//#include <bits/random.h> // not sure why needed. Causes error. just <random> works
#include <random>
#include <iostream>

float gaussianProbability(float x, float mu, float v);
float sampleFromGaussian(float mu, float v);
