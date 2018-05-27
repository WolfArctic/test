/*************************************************************************
	> File Name: oneDimKalman.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年01月26日 星期五 14时11分32秒
 ************************************************************************/

#include <iostream>
#include "oneDimKalman.h"

using namespace std;


OneDimKalman::OneDimKalman(double r)
{
    R = r;
    X.resize(3);
    P0.resize(3,3);
    vR.resize(1);
    vZ.resize(1);

    P0(0,0) = 0.04;
    P0(1,1) = 0.004;
    P0(2,2) = 0.004;


    conter = 0;


}

OneDimKalman::~OneDimKalman()
{
    
}


void OneDimKalman::init(double r)
{
    R = r;
}


vector<double> OneDimKalman::getMessureMent(double z)
{
    vector<double> res;
    if(conter == 0)
    {
        X(0) = z;
        X(1) = 0.0004;
        X(2) = 0.0003;
        vkalman.init(X, P0);
        res.push_back(z);
        res.push_back(0.0);
        res.push_back(0.0);
        conter++;
        return res;
    }
    conter++;
    
    vR(0) = R;
    vkalman.resetR(vR);
    vZ(0) = z;
    vkalman.measureUpdateStep(vZ);

    MyVector _u( 1, 0.0  );
    vkalman.timeUpdateStep ( _u  );
    X = vkalman.getX();

    res.push_back(X(0));
    res.push_back(X(1));
    res.push_back(X(2));
    return res;
}






