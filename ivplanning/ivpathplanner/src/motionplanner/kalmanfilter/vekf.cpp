// --------------------------- Example of Extended Kalman filter ------------------------//
/*
% A plane flights in a 2D space where the x axis is the distance traveled
% by the plane and y axis is its altitude.  This system can be represented
% by the fallowing equations:
% (This is just an example)
%
% xpp = F/m - bx/m * xp^2
% ypp = p/m * xp^2 - g
%
% where m is the plane's weight (1000 kg)
%       bx is the drag coefficient (0.35 N/m?/s?)
%       p is the lift force (3.92 N/m?/s?)
%       g is the gravitational acceleration (9.8 m/s?)
%       F is the motor's thrust
%
% A station on the ground (at the origin) mesures the angle between the
% plane and the ground (x axis) and the distance between the plane and the station.
% These measures are based and the fallowing equations:
%
% theta = atan2(y,x)
% r = sqrt(x^2+y^2)
%
% The variance error matrix of the mesures is:
%
% R = [0.01^2  0
%      0       50^2]
%
% V = [1 0;
%      0 1];
%
% The variance error matrix of the plane's model is: WQW'
%
% Q = [0.01^2    0;
%      0         0.01^2];
%
% W = [0 0;
%      1 0;
%      0 0;
%      0 1];
%
*/

#include "vekf.h"
#include <cmath>
#include <iostream>

using namespace std;
	// 0  1  2   3   4   5  
//x = [x, vx, ax]
//     0   
//z = [x]
#define PI 3.1415926
cVabsekf::cVabsekf() 
{
    setDim(3, 0, 3, 1, 1);
	Period = 0.05;
	tmp_R.resize(1);
}


void cVabsekf::makeBaseA()
{
	A(0,0) = 1.0;
	A(0,1) = Period;
	A(0,2) = Period*Period/2;
	
		
	A(1,0) = 0;
	A(1,1) = 1;
	A(1,2) = Period;
	
	A(2,0) = 0.0;
	A(2,1) = 0.0;
	A(2,2) = 1.0;

}

void cVabsekf::makeA()
{
	
}


void cVabsekf::makeBaseW()
{
	W(0,0) = 0.00040;
	W(1,1) = 0.000003;
	W(2,2) = 0.000003;
	
}


void cVabsekf::makeBaseQ()
{
	Q(0,0) = 0.000005;
	Q(1,1) = 0.0016;
	Q(2,2) = 0.000001;
	
}

void cVabsekf::makeBaseH()
{
	H(0,0) = 1.0;
	H(0,1) = 0.0;
	H(0,2) = 0.0;
	
}


void cVabsekf::makeBaseV()
{
	V(0,0) = 0.0004;
	
}


void cVabsekf::resetR(KVector<double, 0, 1> _tmp_R)
{
	tmp_R = _tmp_R;
}

void cVabsekf::makeR()
{
	R(0,0) = tmp_R(0);
	
}

void cVabsekf::makeProcess()
{
	Vector x_(x.size());
	x_(0) = (x(0) + x(1)*Period + 0.5*x(2)*Period*Period);
	x_(1) = (x(1) + x(2)*Period);
	x_(2) = x(2) ;

    //std::cout <<x_(0) << "\t"<<x_(1) << "\t" << x_(2) << std::endl; 
	x.swap(x_);
}

void cVabsekf::makeMeasure()
{
	z(0)=x(0);
	
}


