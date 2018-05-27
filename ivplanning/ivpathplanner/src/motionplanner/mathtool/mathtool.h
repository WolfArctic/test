/****************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
* 
*
* History: 
* Bo Yan    17/10/11    1.0.0    
****************************************************************/

#ifndef _CLASS_MATHTOOL_H
#define _CLASS_MATHTOOL_H
#pragma once

// c++ lib
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>


using namespace std;
typedef struct  sPt 
{
  double x;
  double y;
} sPt;
typedef std::vector<double> doubleVector;

//#define  DC_M_PI  3.1415926


class mathtool 
{
public:
    explicit mathtool();
    virtual ~mathtool();
public:
    void initParam();
    double GetDist(double x0, double y0, double x1, double y1);
    doubleVector GetCoeff(const vector<sPt> &sample, int n);


};
#endif 
