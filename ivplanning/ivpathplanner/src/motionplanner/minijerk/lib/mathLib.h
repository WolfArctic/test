
#ifndef _MATHLIB_H
#define _MATHLIB_H
#pragma once
#include "math.h"
#include <eigen3/Eigen/Dense>
#include <numeric>
#include <iostream>
using namespace std;
using namespace Eigen;

float constrain_float(const float Max,const float Min,float value);
VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order);
#endif