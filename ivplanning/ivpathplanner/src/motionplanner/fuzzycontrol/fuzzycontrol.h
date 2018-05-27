/****************************************************************
* Copyright (C) 2015-2020, idriverplus(BeiJing ZhiXingZhe, Inc.)
* 
*
* History: 
* Bo Yan    17/10/11    1.0.0    
****************************************************************/

#ifndef _CLASS_FUZZYCONTROL_H
#define _CLASS_FUZZYCONTROL_H
#pragma once

// c++ lib
#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "../../../../avoslib/globalvariables.h"

#define NFB -10
#define NHB -9
#define NXB -8
#define NLB -7
#define NMB -6
#define NSB -5

#define NVB -4
#define NB -3
#define NM -2
#define NS -1
#define ZO 0
#define PS 1
#define PM 2
#define PB 3
#define PVB 4

using namespace std;

class fuzzycontroller 
{

public:
   int accel_mode;
private:
    const static int N = 10;//定义量化论域模糊子集的个数
    double e_upper;  //误差基本论域上限
    double e_lower;
    double de_upper; //误差辩化率基本论域的上限
    double de_lower; //误差辩化率基本论域的上限
    double u_upper;  //输出的上限
    double u_lower;
    double Ke;    //Ke=n/emax,量化论域为[-3,-2,-1,0,1,2,3]
    double Kde;   //Ke=n/demax,量化论域为[-3,-2,-1,0,1,2,3]
    double Ku;    //Ke=umax/n,量化论域为[-3,-2,-1,0,1,2,3]
    double accel_plus;
    double accel_minus;
    // int accel_mode;
    int accel_mode_last;

    // int ruleMatrix[N][N]={{NVB,NVB,NVB,NVB,NB,NB,NM,NM,NM}, 
    //                       {NVB,NVB,NB,NB,NM,NM,NM,NS,NS},//{NVB,NVB,NB,NB,NM,NM,NM,NM,NS},
    //                       {NVB,NB,NM,NM,NM,NM,NS,NS,ZO}, //{NVB,NB,NM,NM,NM,NM,NM,NS,ZO},
    //                       {NVB,NM,NS,NS,NS,NS,ZO,ZO,PS},//{NM,NM,NM,NS,NS,ZO,ZO,ZO,PS},
    //                       {NB,NM,ZO,ZO,ZO,ZO,ZO,PS,PM}, 
    //                       {NM,NS,ZO,ZO,PS,PS,PS,PM,PM}, //middle:ZO->PS TBD
    //                       {ZO,ZO,ZO,PS,PS,PM,PM,PM,PM},
    //                       {ZO,ZO,PS,PS,PM,PM,PM,PM,PB},
    //                       {ZO,PS,PS,PM,PM,PM,PM,PB,PB}}; //1218
    // int ruleMatrix[N][N]={{NVB,NVB,NVB,NVB,NB,NB,NM,NM,NM}, 
    //                       {NVB,NVB,NB,NB,NM,NM,NM,NS,NS},//{NVB,NVB,NB,NB,NM,NM,NM,NM,NS},
    //                       {NVB,NB,NM,NM,NM,NM,NS,NS,ZO}, //{NVB,NB,NM,NM,NM,NM,NM,NS,ZO},
    //                       {NM,NM,NM,NS,NS,NS,ZO,ZO,PS},//{NM,NM,NM,NS,NS,ZO,ZO,ZO,PS},
    //                       {NM,NM,ZO,ZO,ZO,ZO,ZO,PS,PM}, 
    //                       {NS,ZO,ZO,ZO,PS,PS,PS,PM,PM}, //middle:ZO->PS TBD
    //                       {ZO,ZO,ZO,PS,PS,PM,PM,PM,PM},
    //                       {ZO,ZO,PS,PS,PM,PM,PM,PM,PB},
    //                       {ZO,PS,PS,PM,PM,PM,PM,PB,PB}}; //1218

    // int ruleMatrix_plus[N][N]={{NVB,NVB,NVB,NVB,NB,NB,NM,NM,NM}, 
    //                             {NVB,NVB,NB,NB,NM,NM,NM,NS,NS},//{NVB,NVB,NB,NB,NM,NM,NM,NM,NS},
    //                             {NVB,NB,NM,NM,NM,NM,NS,NS,ZO}, //{NVB,NB,NM,NM,NM,NM,NM,NS,ZO},
    //                             {NB,NM,NM,NS,NS,NS,NS,ZO,PS},
    //                             {NB,NM,ZO,ZO,ZO,ZO,ZO,PS,PM},
    //                             {NM,ZO,ZO,ZO,ZO,ZO,PS,PS,PM},
    //                             {NS,ZO,ZO,PS,PS,PM,PM,PM,PM},
    //                             {ZO,ZO,PS,PS,PM,PM,PM,PM,PB},
    //                             {ZO,PS,PS,PM,PM,PM,PM,PB,PB}}; //0104

    // int ruleMatrix_minus[N][N]={{NVB,NVB,NVB,NVB,NB,NB,NM,NM,NM}, 
    //                             {NVB,NVB,NB,NB,NM,NM,NM,NS,NS},//{NVB,NVB,NB,NB,NM,NM,NM,NM,NS},
    //                             {NVB,NB,NM,NM,NM,NM,NS,NS,ZO}, //{NVB,NB,NM,NM,NM,NM,NM,NS,ZO},
    //                             {NB,NM,NS,ZO,ZO,ZO,ZO,ZO,PS},//{NM,NM,NS,ZO,ZO,ZO,ZO,ZO,PS},
    //                             {NM,NM,ZO,ZO,ZO,ZO,ZO,PS,PM},
    //                             {NM,ZO,ZO,PS,PS,PS,PM,PM,PM},
    //                             {NS,ZO,ZO,PS,PS,PM,PM,PM,PM},
    //                             {ZO,ZO,PS,PS,PM,PM,PM,PM,PB},
    //                             {ZO,PS,PS,PM,PM,PM,PM,PB,PB}}; //0104
    int ruleMatrix[N][N]={{NFB,NHB,NXB,NXB,NLB,NLB,NMB,NMB,NSB,NB},
                          {NLB,NSB,NVB,NVB,NVB,NB, NB, NM, NM, NM}, 
                          {NVB,NVB,NVB,NB, NB, NM, NM, NM, NS, NS},
                          {NVB,NVB,NB, NM, NM, NM, NM, NS, NS, ZO},
                          {NVB,NVB,NM, NS, NS, NS, NS, ZO, ZO, PS},
                          {NB, NB, NM, ZO, ZO, ZO, ZO, ZO, PS, PM},
                          {NM, NM, NS, ZO, ZO, PS, PS, PS, PM, PM},
                          {ZO, ZO, ZO, ZO, PS, PS, PM, PM, PM, PM},
                          {ZO, ZO, ZO, PS, PS, PM, PM, PM, PM, PB},
                          {ZO, ZO, PS, PS, PM, PM, PM, PM, PB, PB}}; //0207

    int ruleMatrix_plus[N][N]={{NFB,NHB,NXB,NXB,NLB,NLB,NMB,NMB,NSB,NB},
                               {NLB,NVB,NVB,NVB,NVB,NB, NB, NM, NM, NM}, 
                               {NVB,NVB,NVB,NB, NB, NM, NM, NM, NS, NS},
                               {NVB,NVB,NB, NM, NM, NM, NM, NS, NS, ZO},
                               {NVB,NB, NM, NM, NS, NS, NS, NS, ZO, PS},
                               {NB, NB, NS, ZO, ZO, ZO, ZO, ZO, PS, PM},//{NB, NB, NM, ZO, ZO, ZO, ZO, ZO, PS, PM},
                               {NM, NM, ZO, ZO, ZO, ZO, ZO, PS, PS, PM},
                               {ZO, NS, ZO, ZO, PS, PS, PM, PM, PM, PM},
                               {ZO, ZO, ZO, PS, PS, PM, PM, PM, PM, PB},
                               {ZO, ZO, PS, PS, PM, PM, PM, PM, PB, PB}}; //0207
                               
    int ruleMatrix_minus[N][N]={{NFB,NHB,NXB,NXB,NLB,NLB,NMB,NMB,NSB,NB},
                                {NLB,NVB,NVB,NVB,NVB,NB, NB, NM, NM, NM}, 
                                {NSB,NVB,NVB,NB, NB, NM, NM, NM, NS, NS},
                                {NVB,NVB,NB, NM, NM, NM, NM, NS, NS, ZO},
                                {NVB,NB, NM, NS, ZO, ZO, ZO, ZO, ZO, PS},
                                {NB, NM, NS, ZO, ZO, ZO, ZO, ZO, PS, PM},
                                {NM, NM, ZO, ZO, PS, PS, PS, PM, PM, PM},
                                {ZO, NS, ZO, ZO, PS, PS, PM, PM, PM, PM},
                                {ZO, ZO, ZO, PS, PS, PM, PM, PM, PM, PB},
                                {ZO, ZO, PS, PS, PM, PM, PM, PM, PB, PB}}; 
    //double e_mf_paras[27]={-10,-10,-4,-10,-4,-2,-4,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,4,2,4,10,4,10,10};//误差e的隶属度函数参数，这里隶属度函数为三角型，所以3个数据为一组
    //double e_mf_paras[27]={-10,-10,-4,-10,-4,-2,-4,-2,-0.5,-2,-0.5,0,-0.5,0,0.5,0,0.5,2,0.5,2,4,2,4,10,4,10,10};
    double e_mf_paras[30]={-20,-20,-10,-20,-10,-4,-10,-4,-2,-4,-2,-0.5,-2,-0.5,0,-0.5,0,0.5,0,0.5,2,0.5,2,4,2,4,10,4,10,10};
    //double de_mf_paras[27]={-7,-7,-4,-7,-4,-2,-4,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,4,2,4,7,4,7,7};//误差变化率de的模糊隶属度函数参数
    double de_mf_paras[30]={-10,-10,-7,-10,-7,-4,-7,-4,-2,-4,-2,-1,-2,-1,0,-1,0,1,0,1,2,1,2,4,2,4,7,4,7,7};//误差变化率de的模糊隶属度函数参数
    //double u_mf_paras[27]={-2.5,-2.5,-1.8,-2.5,-1.8,-1.0,-1.8,-1,-0.4,-1.0,-0.4,0,-0.4,0,0.2,0,0.2,0.5,0.2,0.5,0.9,0.5,0.9,1.5,0.9,1.5,1.5};//输出量u的隶属度函数参数
    //double u_mf_paras[30]={-8,-8,-2.5,-8,-2.5,-1.8,-2.5,-1.8,-1.0,-1.8,-1,-0.4,-1.0,-0.4,0,-0.4,0,0.2,0,0.2,0.5,0.2,0.5,0.9,0.5,0.9,1.5,0.9,1.5,1.5};//输出量u的隶属度函数参数
    double u_mf_paras[45]={-8,-8,-7,-8,-7,-6,-7,-6,-5,-6,-5,-4,-5,-4,-3,-4,-3,-2.5,-3,-2.5,-1.8,-2.5,-1.8,-1.0,-1.8,-1,-0.4,-1.0,-0.4,0,-0.4,0,0.2,0,0.2,0.5,0.2,0.5,0.9,0.5,0.9,1.5,0.9,1.5,1.5};//输出量u的隶属度函数参数
public:
	explicit fuzzycontroller();
    virtual ~fuzzycontroller();
    double realize(double e,double de, double accel, double relspeed);              //实现模糊控制
private:
    double trimf(double x,double a,double b,double c);             //三角隶属度函数
    double gaussmf(double x,double ave,double sigma);              //正态隶属度函数
    double trapmf(double x,double a,double b,double c,double d);   //梯形隶属度函数


};
#endif 
