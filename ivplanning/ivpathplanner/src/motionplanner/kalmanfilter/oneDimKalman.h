#ifndef ONEDIMKALMAN_H_
#define ONEDIMKALMAN_H_

#pragma once

//#include "kalman/ekfilter.hpp"
#include "vekf.h"
#include <sstream>
#include <iostream>
#include <fstream>

#include <vector>

using namespace Kalman;
using namespace std;



class OneDimKalman
{
    public:
        OneDimKalman(double R);
        ~OneDimKalman();
        void init(double R);

        vector<double> getMessureMent(double Z);
    private:
        double R;
        MyMatrix P0;
        MyVector X;
        MyVector vR;
        MyVector vZ ;
        cVabsekf vkalman;
        int conter;



};








#endif


