/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: kalmanpredict.h
Description: 
1. Kalman filter of the obsolute position of the object 

Write by Yang Xia
 * Basic Kalman Filter
 * state equation:       x(k) = A*x(k-1)+w(k)
 * measurement equation: z(k) = H*x(k)+v(k)
 * predict 1:            x(k|k-1) = A*x(k-1|k-1)
 * predict 2:            P(k|k-1) = A*P(k-1|k-1)*A'+Q
 * update 1:             K(k) = (P(k|k-1)*H')/(H*P(k|k-1)*H'+R)
 * update 2:             x(k|k) = x(k|k-1) + K(k)*(z(k)-H*x(k|k-1))
 * update 3:             P(k) = (I-K(k)*H)*P(k|k-1)
 
History:
<author>    <time>      <version>    <description>
hui li       17/06/13     1.0.0        achieve Kalman filter for predict(not tested)  
                                  
************************************************************/

#ifndef _KALMANPREDICT_H
#define _KALMANPREDICT_H
#pragma once

#include <cstdlib>
#include <string>
#include <sstream>

#include "Eigen/Dense"
#include "Eigen/Eigen"

#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include <vector>
#include "../glovariable.h"

static Eigen::Matrix<double,4,1> X0_Eigen_pred;  //4*1
static Eigen::Matrix<double,4,1> X1_Eigen_pred;  //4*1
static Eigen::Matrix<double,2,1> Z_Eigen_pred;   //2*1
static Eigen::Matrix<double,2,4> H_Eigen_pred; 	 //2*4

static Eigen::Matrix<double,4,4> A_Eigen_pred;   //4*4
static Eigen::Matrix<double,4,4> P0_Eigen_pred;  //4*4
static Eigen::Matrix<double,4,4> Q_Eigen_pred;   //4*4
static Eigen::Matrix<double,4,4> P1_Eigen_pred;  //4*4
static Eigen::Matrix<double,2,2> R_Eigen_pred; 	 //2*2
static Eigen::Matrix<double,4,2> K_Eigen_pred; 	 //4*2

/**************************************************************
Function: AbsolutePositionKalmanPredict
Author: hui li
Date: 17/06/13
Description: 
1. predict the position by Kalman Filter
 
Input: point absolute position
Output: filted absolute position
Return: void
Others: 
************************************************************/
static void InitKF(Eigen::Matrix<double,4,1> temp_X0, double dt, Eigen::Matrix<double,2,1> temp_Z_Eigen_pred)
{
    //constant
    X0_Eigen_pred = temp_X0;   
    A_Eigen_pred<< 1, dt, 0, 0,
		   0, 1, 0,  0,
		   0, 0, 1, dt, 
		   0, 0, 0,  1;
    if(gFirstTimeP0==true)
    {
	gFirstTimeP0=false;
	P0_Eigen_pred << 0.1, 0, 0, 0,
			  0, 0.1, 0, 0,
			  0, 0, 0.1, 0,
			  0, 0, 0, 0.1;
    }
    // cout << P0_Eigen_pred << endl;
    Q_Eigen_pred << 0.01,    0,    0, 0,
		    0, 0.01,    0, 0,
		    0,    0, 0.01, 0,
		    0,    0,    0, 0.01;    
    R_Eigen_pred << 0.5, 0,
		    0, 0.5;  
    H_Eigen_pred << 1, 0, 0, 0,
		    0, 0, 1, 0;  
    Z_Eigen_pred = temp_Z_Eigen_pred; 
}

static Eigen::Matrix<double,4,1> PredictEquationCalculate()
{
  Eigen::Matrix<double,2,1> temp_position;  
  X1_Eigen_pred = A_Eigen_pred * X0_Eigen_pred;   
  P1_Eigen_pred = A_Eigen_pred * P0_Eigen_pred * (A_Eigen_pred.transpose()) + Q_Eigen_pred;   
  // printf("-------\n");
  // cout << A_Eigen_pred << endl;
  // cout << P0_Eigen_pred << endl;
  // cout << A_Eigen_pred.transpose() << endl;
  // cout << Q_Eigen_pred << endl;
  // cout << P1_Eigen_pred << endl;
  // printf("!!!!\n");
  //K
  Eigen::Matrix<double,4,2> H_Eigen_pred_R = H_Eigen_pred.transpose();  
  Eigen::Matrix<double,2,2> temp = H_Eigen_pred * P1_Eigen_pred * H_Eigen_pred_R + R_Eigen_pred;  
  K_Eigen_pred = P1_Eigen_pred * H_Eigen_pred_R * temp.inverse(); 
  
  //X0
  Eigen::Matrix<double,4,4> II = Eigen::Matrix<double,4,4>::Identity(4,4);
  Eigen::Matrix<double,4,4> tmp_KH = (II - K_Eigen_pred*H_Eigen_pred);
  X0_Eigen_pred = tmp_KH * X1_Eigen_pred + K_Eigen_pred * Z_Eigen_pred;  
  
  //P0
  P0_Eigen_pred=(II-K_Eigen_pred*H_Eigen_pred)*P1_Eigen_pred;

  return X0_Eigen_pred;
}

static Eigen::Matrix<double,4,1> AbsolutePositionKalmanPredict(double xg0, double vx0, double yg0, double vy0, double dt,double xg,double yg)
{
  Eigen::Matrix<double,4,1> temp_X0;  
  Eigen::Matrix<double,2,1> temp_Z_Eigen_pred;   
  Eigen::Matrix<double,4,1> output_position;
  temp_X0( 0 ) = xg0;
  temp_X0( 1 ) = vx0;
  temp_X0( 2 ) = yg0;
  temp_X0( 3 ) = vy0;
  temp_Z_Eigen_pred ( 0 ) = xg;
  temp_Z_Eigen_pred ( 1 ) = yg;
  
  //init Kalman parameters
  
  InitKF(temp_X0,dt,temp_Z_Eigen_pred);  
  
//   //step 1 -- state equation
  output_position = PredictEquationCalculate();

  return output_position;
}

#endif    