
/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: motionbased.h
Description: 
1. short predict of the object using rel-position
2. if Buffer is less than frequency , output the current position
3. case 4 use rel-position but predict at the same position

History:
<author>    <time>      <version>    <description>
fang zhang   17/04/15     1.0.0        creat
hui li       17/06/06     1.0.1        achieve rel-position and case 4 function(static obj) ignore the cell for test
hui li       17/06/09     1.0.2        release cell      
zongwei	     17/06/22	  1.0.3		   revise relative predicting method.
hui li	     17/09/18	  1.0.4		   change the method of abs-predict and receive the speed from lidar.

************************************************************/
#ifndef _IVPREDICTMOTIONBASE_H
#define _IVPREDICTMOTIONBASE_H
#pragma once
//c++ lib

#include <vector>
#include <math.h>
#include <sstream>
#include <iostream>
#include <fstream>

// ROS lib
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "ivmap/ivmapmsglocpos.h"
#include "ivpredict/predictobj.h"
#include "ivpredict/objbuffer.h"

#include "../glovariable.h"
#include "../globaldefine.h"
//#include "../predictmethods/commonfunctions.h"
#include "../../../../avoslib/geotool.h"

using namespace std;

/**************************************************************
Function: PredictMotionBased
Author: hui li
Date: 17/06/06
Description: 
1. rel-position predict normal

Input: object buffer, car current pose, frequency
Output: short predict
Return: short predict result
Others: 
************************************************************/
static ivpredict::predictobj PredictMotionBasedRel(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos locpos,int frequency)
{
    int predict_buffer_size = frequency*1;
    int total_output_point = frequency*5;
    //int BUFFER_SIZE = frequency*3;
    int model = 0;
    float v_sumx = 0.0, v_sumy = 0.0, v_finalx , v_finaly ,v_objx, v_objy;
    float angle_pre[predict_buffer_size], angle_omiga;
    ivpredict::predictobj predict_result;
    ivpredict::predictpos result_pos;
    // if (gDebug)
    // {
    // 	cout << " ivpredict-debug-PredictMotionBasedRel objID: "<< pObj->id << " xmean: " << pObj->positions[pObj->positions.size()-1].xmean << " ymean: " << pObj->positions[pObj->positions.size()-1].ymean 
    // 	<< " vxrel: " << pObj->xvrel << " vyrel: " << pObj->yvrel << endl;
    // }
    int buffer_size = pObj->positions.size();
    if (pObj->positions.size() >= predict_buffer_size && abs(pObj->vabs) > gSpeed_stop) // && (pObj->positions[buffer_size-1].xmean > -1) || (abs(pObj->positions[buffer_size-1].ymean) > 1)
    {

        //calculate the v parameters for speed predict
        float v_abs = 0.0 , v_acc = 0.0;
        int y_dir = 1;

        //calculate the omiga parameters for turn predict
        float omiga[2], omiga_change = 0.0, r = 0, temp_omigax, temp_omigay,omiga_sum = 0.0,omiga_temp[total_output_point];

        //variables for initial process
        int turn_flag = 1, firstprocess = 1;
        int init = 0;

        for (int i = pObj->positions.size()-predict_buffer_size; i < (pObj->positions.size()-1); i++)
        {
            v_objx = pObj->positions[i+1].xmean - pObj->positions[i].xmean;
            v_objy = pObj->positions[i+1].ymean - pObj->positions[i].ymean;
            v_sumx = v_sumx +  v_objx;
            v_sumy = v_sumy +  v_objy;
        }
        
        //calculate the speed parameters
        v_finalx = v_sumx/(predict_buffer_size-1);
        v_finaly = v_sumy/(predict_buffer_size-1);

        // v_finalx = pObj->xvrel/(predict_buffer_size-1);
        // v_finaly = pObj->yvrel/(predict_buffer_size-1);

        v_sumx = 0;
        v_sumy = 0;
        v_abs = sqrt((v_finalx)*(v_finalx)+(v_finaly)*(v_finaly));

        if (v_finaly > 0)
        {
          y_dir = -1;
        }
        else
        {
          y_dir = 1;
        }


        omiga[0] = pObj->ave_angle;
        
        if ( v_finaly != 0)
        {
            omiga[1] = atan(v_finalx/v_finaly); //no abs because if will affect the omiga_change
        }
        else if ( v_finalx == 0)
        {
            omiga[1] = 0;
        }
        else if (v_finalx > 0)
        {
            omiga[1] = M_PI /2.0;
        }
        else
        {
            omiga[1] = - M_PI / 2.0;
        }
        pObj->ave_angle = omiga[1];

        //ignore the influence of the atan() function
        if (abs( omiga[1] - omiga[0] ) > 0.5)
        {
          omiga_change = pObj->change_omiga/20;

        }
        else
        {
          omiga_change = (omiga[1] - omiga[0])/20;
        }
        pObj->change_omiga = omiga_change;

        if (gDebug)
        {
          //cout << "predict-debug-PredictMotionBasedRel : id " << pObj->id << " " << pObj->vabs << endl;
          // cout << "predict-debug-PredictMotionBasedRel x y : " << pObj->positions[pObj->positions.size()-1].xmean << " " << pObj->positions[pObj->positions.size()-1].ymean << endl;
          // cout << "predict-debug-PredictMotionBasedRel vx vy: " << pObj->xvrel << " " << pObj->yvrel << endl;
          // cout << "predict-debug-PredictMotionBasedRel calculate vx vy: " << v_finalx << " " << v_finaly << endl;
        }

        //turn left or turn right
        if (omiga_change < 0)
        {
          turn_flag = 1;
        }
        else
        {
          turn_flag = -1;
        }

        //calculate the r
        r = abs(v_abs/omiga_change);

        // //approximately 5 degree = straight waiting for adjust
        // if (abs(omiga_change) < 0.0001) //0.005
        // {

        //   for (int i = 0; i < (total_output_point); i++)
        //   {
        //     result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean + v_finalx * i;
        //     result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean + v_finaly * i;
        //     result_pos.xgmean = pObj->positions[pObj->positions.size()-1].xgmean;
        //     result_pos.ygmean = pObj->positions[pObj->positions.size()-1].ygmean;
        //     predict_result.positions.push_back(result_pos);
        //   }
        // }
        // //turn some angle
        // else 
        {

          switch(model)
          {
            case 0:
            {
              for (int i = 0; i < total_output_point; i++)
              {
                omiga_temp[i] = omiga_change*i;
              }
            }

            default:
            {

            }
          }

          ivpredict::predictpos temp_pos;
          //calculate the turn circle
          for (int i = 0; i < (total_output_point); i++)
          {
            temp_omigax = r * (1-cos(omiga_temp[i]));
            temp_omigay = r * sin(omiga_temp[i]);

            result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean + (temp_omigax * cos(omiga[1]) + temp_omigay * sin(omiga[1])) * y_dir * turn_flag;
            result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean + (temp_omigay * cos(omiga[1]) - temp_omigax * sin(omiga[1])) * y_dir * turn_flag;            
            result_pos.xgmean = pObj->positions[pObj->positions.size()-1].xgmean;
            result_pos.ygmean = pObj->positions[pObj->positions.size()-1].ygmean;
            predict_result.positions.push_back(result_pos);

          }
        }
	    predict_result.v_abs = pObj->vabs;
    }
    else
    {
        int size = pObj->positions.size();
        result_pos.xmean = pObj->positions[size-1].xmean;
        result_pos.ymean = pObj->positions[size-1].ymean;
        result_pos.xgmean = pObj->positions[size-1].xgmean;
        result_pos.ygmean = pObj->positions[size-1].ygmean;
        for (int i = 0;i<total_output_point;i++)
        {
            predict_result.positions.push_back(result_pos);
        }
	    predict_result.v_abs = 0;
    }

    //  use method 1 to compute predict_result
    predict_result.id = pObj->id;
    predict_result.length = pObj->length;
    predict_result.width = pObj->width;
    predict_result.height = pObj->height;
    predict_result.cell = pObj->cell;
    predict_result.time_interval = 1./frequency ;
    predict_result.steps = total_output_point;
    predict_result.xvrel = v_finalx * (predict_buffer_size-1);
    predict_result.yvrel = v_finaly * (predict_buffer_size-1);
    // predict_result.xvrel = pObj->xvrel;
    // predict_result.yvrel = pObj->yvrel;
    predict_result.vrel = pObj->vrel;
    predict_result.type = pObj->property;
    return predict_result;

}

/**************************************************************
Function: PredictMotionBasedAbs
Author: hui li
Date: 17/06/06
Description: 
1. Absolute-position predict normal

Input: object buffer, car current pose, frequency
Output: short predict
Return: short predict result
Others: 
************************************************************/
static ivpredict::predictobj PredictMotionBasedAbs(ivpredict::objbuffer *pObj,ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap,int frequency)
{
    // cout << pObj->positions.size() << endl;
    float gMinVelThresholdshort=0.3;
    int predict_buffer_size = 5;
    int check_size = 10;
    int time_delay = 10;
    int total_output_point = frequency*5;
  //  int BUFFER_SIZE = frequency*3;
    int model = 0;
    float v_sumx = 0.0, v_sumy = 0.0, v_finalx , v_finaly ,v_objx, v_objy;
    float angle_pre[predict_buffer_size], angle_omiga;
    ivpredict::predictobj predict_result;
    ivpredict::predictpos result_pos;
    float v_abs = 0.0 , v_acc = 0.0;

    //speed control
    bool speedcontrol = false;
    int speed_size_check = pObj->positions.size();
    //cout << " lihui speed-point: " << pObj->vabs  << " obj:" << pObj->id << endl;
    int last_state = pObj->state; //stay_count
    if (pObj->positions.size() >= predict_buffer_size)
    {
        int speed_up_count = 0;
	    for (int i = 0; i < check_size; ++i)
	    {
	    	if (abs(pObj->vabs) > gMinVelThresholdshort)
		    {
                speed_up_count++;
                if (speed_up_count > 8)
                {
                    speedcontrol = true;
                    break;
                }

		    }
	    }
	}
     if (last_state != -1 && speedcontrol == false)
     {
        pObj->stay_count++;
        if (pObj->stay_count > time_delay)
        {
            pObj->stay_count = 0;
            speedcontrol = false;
        }
        else
        {
            speedcontrol = true;
        }
     }

    if (pObj->positions.size() >= predict_buffer_size && speedcontrol && (pObj->positions[speed_size_check-1].xmean > 0 || fabs(pObj->positions[speed_size_check-1].ymean) > 1 ))
    {
        int y_dir = 1;

        //calculate the omiga parameters for turn predict
        float omiga[2], omiga_change = 0.0, r = 0, temp_omigax, temp_omigay,omiga_sum = 0.0,omiga_temp[total_output_point];

        //variables for initial process
        int turn_flag = 1, firstprocess = 1;
        int init = 0;
        for (int i = pObj->positions.size()-predict_buffer_size; i < (pObj->positions.size()-1); i++)
        {

          //  int pose_start = (pObj->positions.size()-predict_buffer_size-1);
            v_objx = pObj->positions[i+1].xgmean - pObj->positions[i].xgmean;
            v_objy = pObj->positions[i+1].ygmean - pObj->positions[i].ygmean;
            // v_objx = pObj->positions[i+1].xmean - pObj->positions[i].xmean;
            // v_objy = pObj->positions[i+1].ymean - pObj->positions[i].ymean;
            v_sumx = v_sumx +  v_objx;
            v_sumy = v_sumy +  v_objy;

          //  angle_pre[i] = atan(v_objx[i]/v_objy[i]);
        }
        
        //calculate the speed parameters
        // v_finalx = v_sumx/(predict_buffer_size-1);
        // v_finaly = v_sumy/(predict_buffer_size-1);
        v_finalx = pObj->xvabs/float(frequency-1);
        v_finaly = pObj->yvabs/float(frequency-1);

        // cout << v_finalx << " " << v_finaly << endl;
        v_sumx = 0;
        v_sumy = 0;
        // v_abs = pObj->vabs/float(predict_buffer_size-1);
        v_abs = sqrt((v_finalx)*(v_finalx)+(v_finaly)*(v_finaly));
        // pObj->vabs = v_abs*float(predict_buffer_size-1);
        //v_abs = (v_abs*6 + pObj->vabs/frequency*4)/10;
        // if (v_abs < 0.2)
        // {
        //  v_abs = 0.001;
        // }
      //  printf("-----------------------\n");
        // cout << pObj->id << " " << v_abs << endl;

        if (v_finaly > 0)
        {
          y_dir = -1;
        }
        else
        {
          y_dir = 1;
        }


        omiga[0] = pObj->ave_angle;
        // cout << omiga[0] << endl;
        if ( v_finaly != 0)
        {
            omiga[1] = atan(v_finalx/v_finaly); //no abs because if will affect the omiga_change
        }
        else if ( v_finalx == 0)
        {
            omiga[1] = 0;
        }
        else if (v_finalx > 0)
        {
            omiga[1] = M_PI /2.0;
        }
        else
        {
            omiga[1] = - M_PI / 2.0;
        }
        pObj->ave_angle = omiga[1];
        // cout << omiga[1] << endl;
        //ignore the influence of the atan() function
        if (abs( omiga[1] - omiga[0] ) > 0.5)
        {
          omiga_change = pObj->change_omiga;///pObj->vabs;
        }
        else
        {
          omiga_change = (omiga[1] - omiga[0]);///pObj->vabs;
        //  printf("%f\n",omiga_change);
        }
        pObj->change_omiga = omiga_change/20;
        // cout << pObj->vabs <<" "<<gChangeOmiga << endl;
        if (omiga_change < 0)
        {
          turn_flag = 1;
        }
        else
        {
          turn_flag = -1;
        }


        //calculate the r
        r = abs(v_abs/omiga_change);

        // if (abs(omiga_change) < 0.002) //0.005
        {

          for (int i = 0; i < (total_output_point); i++)
          {
              result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean;
              result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean;

              result_pos.xgmean = pObj->positions[pObj->positions.size()-1].xgmean + v_finalx * i;
              result_pos.ygmean = pObj->positions[pObj->positions.size()-1].ygmean + v_finaly * i;
              predict_result.positions.push_back(result_pos);
          }
          // cout << predict_result.positions[0].xgmean << " " << predict_result.positions[0].ygmean << endl;
        //
        }

       // else //turn a circle
        /*
        {

        //  printf("%d\n", model);
          switch(model)
          {
            case 0:
            {
              for (int i = 0; i < total_output_point; i++)
              {
                omiga_temp[i] = omiga_change*i;
                if ( omiga_temp[i] > 3.14/2)
                {
                  omiga_temp[i] = 3.14/2;
                }
                if ( omiga_temp[i] < -3.14/2)
                {
                  omiga_temp[i] = -3.14/2;
                }
              }
            }

            default:
            {

            }
          }

          ivpredict::predictpos temp_pos;
          //calculate the turn circle
          for (int i = 0; i < (total_output_point); i++)
          {
            temp_omigax = r * (1-cos(omiga_temp[i]));
            temp_omigay = r * sin(omiga_temp[i]);
            result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean;
            result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean;

            //objects to global
            result_pos.xgmean = pObj->positions[pObj->positions.size()-1].xgmean + (temp_omigax * cos(omiga[1]) + temp_omigay * sin(omiga[1])) * y_dir * turn_flag;
            result_pos.ygmean = pObj->positions[pObj->positions.size()-1].ygmean + (temp_omigay * cos(omiga[1]) - temp_omigax * sin(omiga[1])) * y_dir * turn_flag;
            predict_result.positions.push_back(result_pos);

          }
          // cout << predict_result.positions[0].xgmean << " " << predict_result.positions[0].ygmean << endl;
        }
        */
	    predict_result.v_abs = pObj->vabs;
        predict_result.state = 1;
    }
    else
    {
    // for (int i = 0;i<total_output_point;i++)
    //{
        int size = pObj->positions.size();
    //    cout << size << endl;    
        result_pos.xmean = pObj->positions[size-1].xmean;
        result_pos.ymean = pObj->positions[size-1].ymean; 
        result_pos.xgmean = pObj->positions[size-1].xgmean;
        result_pos.ygmean = pObj->positions[size-1].ygmean;
        for (int i = 0;i<total_output_point;i++)
        {
            predict_result.positions.push_back(result_pos);
        }
        // cout << predict_result.positions[0].xgmean << " " << predict_result.positions[0].ygmean << endl;
	     predict_result.v_abs = 0.0;
	if(speed_size_check < predict_buffer_size)
	predict_result.state = 0;
	else
        predict_result.state = -1;
    }

    predict_result.id = pObj->id;
    predict_result.length = pObj->length;
    predict_result.width = pObj->width;
    predict_result.height = pObj->height;
    predict_result.cell = pObj->cell;
    predict_result.time_interval = 1./frequency ;
    predict_result.steps = total_output_point;
    predict_result.xvrel = pObj->xvrel;
    predict_result.yvrel = pObj->yvrel;
    predict_result.vrel = pObj->vrel;
    predict_result.type = pObj->property;
    pObj->state = predict_result.state;
    
    
    // gAbsXV = v_finalx * frequency;
    // gAbsYV = v_finalx * frequency;
    // gAbsV = v_abs * frequency;

    return predict_result;

}

/**************************************************************
Function: PredictMotionStatic
Author: hui li
Date: 17/06/06
Description: 
1. rel-position predict static

Input: object buffer, frequency
Output: short predict
Return: short predict result
Others: 
************************************************************/
static ivpredict::predictobj PredictMotionStatic(ivpredict::objbuffer *pObj, int frequency)
{
  int total_output_point = frequency*5;
  ivpredict::predictobj predict_result;
  ivpredict::predictpos result_pos;

  predict_result.id = pObj->id;
    
  predict_result.length = pObj->length;
  predict_result.width = pObj->width;
  predict_result.height = pObj->height;
  predict_result.cell = pObj->cell;
  predict_result.time_interval = 1./frequency ;
  predict_result.steps = total_output_point;
  predict_result.xvrel = pObj->xvrel;
  predict_result.yvrel = pObj->yvrel;
  predict_result.vrel = pObj->vrel;
  predict_result.type = pObj->property;
  predict_result.v_abs = pObj->vabs;
  result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean;
  result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean;

  for (int i = 0;i<total_output_point;i++)
  {
      predict_result.positions.push_back(result_pos);
  }
  return predict_result;
}

/**************************************************************
Function: PredictMotionBasedAbsCar
Author: hui li
Date: 17/06/16
Description: 
1. Absolute-position predict normal type car object

Input: object buffer, car current pose, frequency
Output: short predict
Return: short predict result
Others: 
************************************************************/
static ivpredict::predictobj PredictMotionBasedRule(ivpredict::objbuffer *pObj,ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap,int frequency)
{
    // cout << pObj->positions.size() << endl;
    int predict_buffer_size = frequency*1;
    int total_output_point = frequency*2;
  //  int BUFFER_SIZE = frequency*3;
    int model = 0;
    float v_sumx = 0.0, v_sumy = 0.0, v_finalx , v_finaly ,v_objx, v_objy;
    float angle_pre[predict_buffer_size], angle_omiga;
    ivpredict::predictobj predict_result;
    ivpredict::predictpos result_pos;
    float v_abs = 0.0 , v_acc = 0.0;

    ros::Time nowTime = ros::Time::now();
    float time_exit = (ros::Time::now()).toSec() - pObj->init_time;

    if (pObj->positions.size() >= predict_buffer_size && time_exit > 0.5)
    {
        int y_dir = 1;

        //calculate the omiga parameters for turn predict
        float omiga[2], omiga_change = 0.0, r = 0, temp_omigax, temp_omigay,omiga_sum = 0.0,omiga_temp[total_output_point];

        //variables for initial process
        int turn_flag = 1, firstprocess = 1;
        int init = 0;
        for (int i = pObj->positions.size()-predict_buffer_size; i < (pObj->positions.size()-1); i++)
        {

          //  int pose_start = (pObj->positions.size()-predict_buffer_size-1);
            v_objx = pObj->positions[i+1].xgmean - pObj->positions[i].xgmean;
            v_objy = pObj->positions[i+1].ygmean - pObj->positions[i].ygmean;
            // v_objx = pObj->positions[i+1].xmean - pObj->positions[i].xmean;
            // v_objy = pObj->positions[i+1].ymean - pObj->positions[i].ymean;
            v_sumx = v_sumx +  v_objx;
            v_sumy = v_sumy +  v_objy;

          //  angle_pre[i] = atan(v_objx[i]/v_objy[i]);
        }
        
        //calculate the speed parameters
        v_finalx = v_sumx/(predict_buffer_size-1);
      //  cout << v_finalx << v_finaly << endl;
        v_finaly = v_sumy/(predict_buffer_size-1);
        v_sumx = 0;
        v_sumy = 0;
        v_abs = sqrt((v_finalx)*(v_finalx)+(v_finaly)*(v_finaly));
        v_abs = (v_abs*8 + pObj->vabs/frequency*2)/10;
      //  printf("-----------------------\n");

        if (v_finaly > 0)
        {
          y_dir = -1;
        }
        else
        {
          y_dir = 1;
        }


        omiga[0] = pObj->ave_angle;
        //
        if ( v_finaly != 0)
        {
            omiga[1] = atan(v_finalx/v_finaly); //no abs because if will affect the omiga_change
        }
        else if ( v_finalx == 0)
        {
            omiga[1] = 0;
        }
        else if (v_finalx > 0)
        {
            omiga[1] = M_PI /2.0;
        }
        else
        {
            omiga[1] = - M_PI / 2.0;
        }
        pObj->ave_angle = omiga[1];

        //ignore the influence of the atan() function
        if (abs( omiga[1] - omiga[0] ) > 0.5)
        {
          omiga_change = pObj->change_omiga/pObj->vabs/5;
        }
        else
        {
          omiga_change = (omiga[1] - omiga[0])/pObj->vabs/5;
        //  printf("%f\n",omiga_change);
        }
        pObj->change_omiga = omiga_change;

        if (omiga_change < 0)
        {
          turn_flag = 1;
        }
        else
        {
          turn_flag = -1;
        }


        //calculate the r
        r = abs(v_abs/omiga_change);

        if (abs(omiga_change) < 0.001) //0.005
        {

          for (int i = 0; i < (total_output_point); i++)
          {
              result_pos.xgmean = pObj->positions[pObj->positions.size()-1].xgmean + v_finalx  * i;
              result_pos.ygmean = pObj->positions[pObj->positions.size()-1].ygmean + v_finaly * i;
              predict_result.positions.push_back(result_pos);
          }

        //
        }

        else //turn a circle
        {

          if(v_abs * frequency < 5)
          {
            model = 0;
          }
          else
          {
            model = 1;
          }

          switch(model)
          {
            case 0:
            {
              for (int i = 0; i < total_output_point; i++)
              {
                omiga_temp[i] = omiga_change*i;
                if ( omiga_temp[i] > 270)
                {
                  omiga_temp[i] = 270;
                }
              }
              break;
            }
            case 1:
            {
              for (int i = 0; i < total_output_point; i++)
              {
                omiga_temp[i] = omiga_change*i/5;
                if ( omiga_temp[i] > 270)
                {
                  omiga_temp[i] = 270;
                }
              }
              break;
            }

            default:
            {

            }
          }

          ivpredict::predictpos temp_pos;
          //calculate the turn circle
          for (int i = 0; i < (total_output_point); i++)
          {
            temp_omigax = r * (1-cos(omiga_temp[i]));
            temp_omigay = r * sin(omiga_temp[i]);

            //objects to global
            result_pos.xgmean = pObj->positions[pObj->positions.size()-1].xgmean + (temp_omigax * cos(omiga[1]) + temp_omigay * sin(omiga[1])) * y_dir * turn_flag;
            result_pos.ygmean = pObj->positions[pObj->positions.size()-1].ygmean + (temp_omigay * cos(omiga[1]) - temp_omigax * sin(omiga[1])) * y_dir * turn_flag;
            predict_result.positions.push_back(result_pos);

          }
        }
    }
    else
    {
    // for (int i = 0;i<total_output_point;i++)
    //{
        int size = pObj->positions.size();
    //    cout << size << endl;     
        result_pos.xgmean = pObj->positions[size-1].xgmean;
        result_pos.ygmean = pObj->positions[size-1].ygmean;
        for (int i = 0;i<total_output_point;i++)
        {
            predict_result.positions.push_back(result_pos);
        }
    }

    //  use method 1 to compute predict_result
    predict_result.id = pObj->id;
    predict_result.length = pObj->length;
    predict_result.width = pObj->width;
    predict_result.height = pObj->height;
    //predict_result.cell = pObj->cell;
    predict_result.time_interval = 1./frequency ;
    predict_result.steps = total_output_point;

    predict_result.xvrel = pObj->xvrel;
    predict_result.yvrel = pObj->yvrel;
    predict_result.vrel = pObj->vrel;
    predict_result.type = pObj->property;
    predict_result.v_abs = pObj->vabs;
    
    return predict_result;

}

#endif
