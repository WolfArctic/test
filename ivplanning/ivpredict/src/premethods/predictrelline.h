/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: predicttrlline.h
Description: 
1. userfunction rel-line predict 
2. 

History:
<author>    <time>      <version>    <description>
hui li       17/08/23     1.0.0        use rel line for predict 
hui li       17/08/30     1.0.1        solve interreaction bug & add smoothing  
hui li       17/09/02     1.0.2        cut in and out first version
                                  
************************************************************/
#ifndef _IVPREDICTREALINE_H
#define _IVPREDICTREALINE_H
#pragma once

static ivpredict::predictobj SmoothPathx_relline(ivpredict::predictobj initPrePath, int dataNumForSmooth)
{
  int totalNum=initPrePath.positions.size();  
  int halfNumForSmooth=(dataNumForSmooth-1)/2;
  for(int i=halfNumForSmooth;i<totalNum-halfNumForSmooth;i++)
  {
    double sumX=0;
    double sumY=0;
    for(int j=i-halfNumForSmooth;j<=i+halfNumForSmooth;j++)
    {
      sumX+=initPrePath.positions[j].xmean;
      sumY+=initPrePath.positions[j].ymean;
    }
    initPrePath.positions[i].xmean=sumX/(dataNumForSmooth*1.0);
    initPrePath.positions[i].ymean=sumY/(dataNumForSmooth*1.0);
  }
  return initPrePath;
}

/**************************************************************
Function: CalPathIncludeVelUseCurveB()
Author: zongwei
Date: 2017.4.24
Description: // 函数功能、性能等的描述
1. cosidering velocity of the predicted paths and then resampling points
2. -------
3. -------
Input:
1. initPrePath: predicted paths without considering velocity
2. fitOrder: order for fitting
3. objVel: velocity of the object
4. timeLength: time length for predicting. unit s.
5. timeStep: time step length.unit s
Output: // 对输出参数的说明
Return: predicted paths
Others: // 其他说明
************************************************************/
static ivpredict::predictobj CalPathIncludeVelUseCurveBx_relline(ivpredict::predictobj initPrePath,int fitOrder,double objVel,double timeLength,double timeStep)
{
  ivpredict::predictobj velPrePath;
  ivpredict::predictpos tmpPoint;
  velPrePath.positions.push_back(initPrePath.positions[0]);
  
  if (fitOrder==3)
  { 
    double detaX=(1.0/6.0-1.0)*initPrePath.positions[0].xmean+4.0/6.0*initPrePath.positions[1].xmean+1.0/6.0*initPrePath.positions[2].xmean;
    double detaY=(1.0/6.0-1.0)*initPrePath.positions[0].ymean+4.0/6.0*initPrePath.positions[1].ymean+1.0/6.0*initPrePath.positions[2].ymean;
    double length=sqrt(detaX*detaX+detaY*detaY);
    
    int totalSize=initPrePath.positions.size();
    for(int i=0;i<=totalSize-1-fitOrder;i++)
    { 
      double detaRate=0.1;
      for(double t=detaRate;t<=1;t+=detaRate)
      {     
        double dP_dtX =(-t*t+2*t-1)/2*initPrePath.positions[i].xmean+(3*t*t-4*t)/2*initPrePath.positions[i+1].xmean;
        double dP_dtY =(-t*t+2*t-1)/2*initPrePath.positions[i].ymean+(3*t*t-4*t)/2*initPrePath.positions[i+1].ymean;
        dP_dtX+=(-3*t*t+2*t+1)/2*initPrePath.positions[i+2].xmean+t*t/2*initPrePath.positions[i+3].xmean;
        dP_dtY+=(-3*t*t+2*t+1)/2*initPrePath.positions[i+2].ymean+t*t/2*initPrePath.positions[i+3].ymean;  
        
        double tmpLength=length+sqrt(dP_dtX*dP_dtX+dP_dtY*dP_dtY)*detaRate;
        if (tmpLength<timeStep*objVel)
        {
          length=tmpLength;
        }
        else
        {   
          double minDetaRate=detaRate/ceil(tmpLength/timeStep/objVel)/5.0;
          if(minDetaRate < 0.001)
          {
            minDetaRate = 0.001;
          }
          
          for(double At=t-detaRate+minDetaRate; At<=t; At+=minDetaRate)
          {
            double dP_dtX1 =(-At*At+2*At-1)/2*initPrePath.positions[i].xmean+(3*At*At-4*At)/2*initPrePath.positions[i+1].xmean;
            double dP_dtY1 =(-At*At+2*At-1)/2*initPrePath.positions[i].ymean+(3*At*At-4*At)/2*initPrePath.positions[i+1].ymean;       
            dP_dtX1+=(-3*At*At+2*At+1)/2*initPrePath.positions[i+2].xmean+At*At/2*initPrePath.positions[i+3].xmean;
            dP_dtY1+=(-3*At*At+2*At+1)/2*initPrePath.positions[i+2].ymean+At*At/2*initPrePath.positions[i+3].ymean;
            length+=sqrt(dP_dtX1*dP_dtX1+dP_dtY1*dP_dtY1)*minDetaRate;
            int tmpCount=floor(length/(timeStep*objVel));
            if (tmpCount>=1)
            {
              double Px=(-At*At*At+3*At*At-3*At+1)/6*initPrePath.positions[i].xmean+(3*At*At*At-6*At*At+4)/6*initPrePath.positions[i+1].xmean;
              double Py=(-At*At*At+3*At*At-3*At+1)/6*initPrePath.positions[i].ymean+(3*At*At*At-6*At*At+4)/6*initPrePath.positions[i+1].ymean;          
              Px+=(-3*At*At*At+3*At*At+3*At+1)/6*initPrePath.positions[i+2].xmean+At*At*At/6*initPrePath.positions[i+3].xmean;
              Py+=(-3*At*At*At+3*At*At+3*At+1)/6*initPrePath.positions[i+2].ymean+At*At*At/6*initPrePath.positions[i+3].ymean;    
              tmpPoint.xmean=Px;
              tmpPoint.ymean=Py;
              velPrePath.positions.push_back(tmpPoint);
              length-=timeStep*objVel;
            }
          }
        }
      }
    }
  }
  
  int totalNum;
  switch(true)
  {
    case true:
      totalNum=round(timeLength/timeStep);
      break;
    case false:
      // totalNum=round(gPathDisSet/initPrePath.v_abs/timeStep);
      break;
    default:
      break;
  } 
  // cout << "lihui velPrePath size : " << velPrePath.positions.size() << endl;

  if (velPrePath.positions.size()>totalNum)
  {
    velPrePath.positions.erase(velPrePath.positions.begin()+totalNum,velPrePath.positions.end());
  }
/*
  if (velPrePath.positions.size()<initPrePath.positions.size())
  {
    for (int i = velPrePath.positions.size(); i < initPrePath.positions.size(); i++)
    {
      ivpredict::predictpos tmpPoint;
      tmpPoint.xmean=initPrePath.positions[i].xmean;
      tmpPoint.ymean=initPrePath.positions[i].ymean;
      velPrePath.positions.push_back(tmpPoint);
    }
 }
*/


  velPrePath.id=initPrePath.id;
  velPrePath.time_interval=timeStep;
  velPrePath.steps=totalNum;  
  velPrePath.length=initPrePath.length;
  velPrePath.width=initPrePath.width;
  velPrePath.height=initPrePath.height;
  velPrePath.cell=initPrePath.cell;
  velPrePath.xvrel=initPrePath.xvrel;
  velPrePath.yvrel=initPrePath.yvrel;
  velPrePath.vrel=initPrePath.vrel;
  velPrePath.v_abs=initPrePath.v_abs;
  velPrePath.type=initPrePath.type;
  velPrePath.probability=initPrePath.probability;
  
  return velPrePath;
}

/**************************************************************
Function: PredictUserFuncRel
Author: hui li
Date: 17/06/06
Description: 
1. predict the user object at the same position use relative position
 
Input: user object
Output: predict object
Return: predict object
Others: 
************************************************************/
static ivpredict::predictobj PredictAbsLineRelfunc(ivpredict::objbuffer *pObj, int frequency, ivpathplanner::ivmsgpath *Reference_path_line, ivmap::ivmapmsglocpos ivlocpos_line, ivmap::ivmapmsgvap ivvap_line)
{
	ivpredict::predictobj predict_result;
    ivpredict::predictpos result_pos;
    int jump = 10;
    int predict_buffer_size = frequency*1;
    int TOTALOUTPUT = frequency*5;
    int size = pObj->positions.size();
    int choosen_point = 0;
    // cout << "lihui v car : " << ivvap_line.v << " yvral : " << pObj->yvrel << endl;
    float obj_speed = ivvap_line.v + pObj->xvrel;
    //cout << "lihui v speed : " << obj_speed << endl;
    if(obj_speed < 0)
    {
        obj_speed = 0;
    }
    float predict_dis = (obj_speed) /frequency*jump;//ivvap_line.v + pObj->yvrel
    float dis_car = sqrt( pObj->positions[size-1].xmean * pObj->positions[size-1].xmean + pObj->positions[size-1].ymean * pObj->positions[size-1].ymean);
    //(abs(pObj->positions[size-1].xmean) < 10 ) && (abs(pObj->positions[size-1].ymean) < 50 ) 
    if (abs(obj_speed) > 3 && pObj->positions.size() >= predict_buffer_size && (abs(pObj->positions[size-1].ymean) < 5) && dis_car < 50 && pObj->positions[size-1].xmean > 0)
    {
        
        float disx,disy,dis,dismin = 9999;
        float nearx, neary;
        
        //get the dismin
      for(long j = 0; j < Reference_path_line->points.size(); j = j + 1)
      {
        
        disx = abs(pObj->positions[size-1].xmean - Reference_path_line->points[j].x);
        disy = abs(pObj->positions[size-1].ymean - Reference_path_line->points[j].y);
        dis = sqrt(disx * disx + disy * disy);
        if (dis < dismin)
        {
          dismin = dis;
          choosen_point = j;
        }
      }
      float kalman_lihui = 0, predict_dismin = 0;
      if(size > 2)
      {
        kalman_lihui = pObj->positions[size-2].xdis - pObj->positions[size-3].xdis;
      }
      predict_dismin = pObj->positions[size-3].xdis + kalman_lihui;
      dismin = (dismin + predict_dismin)/2;
      pObj->positions[size-1].xdis = dismin;
      //cout << " lihui dis " << dismin << endl;

      //calculate the window speed
      // float temp_v = 0;
      // for (int i = size - 6; i < size - 1; i++)
      // {
      //   temp_v = temp_v + (pObj->positions[i+1].xdis - pObj->positions[i].xdis);
      // }
      // float vfusion = temp_v/5;
      int pow_num;
      if ( abs(obj_speed) < 10) //need add back off 
      {
        pow_num = floor((20 - obj_speed)/4);
        if (pow_num < 1)
        {
          pow_num = 1;
        }
      }
      else
      {
        pow_num = 1;
      }
      // float pow_answer = pow(obj_speed,pow_num);
      float vfusion = (pObj->positions[size-1].xdis - pObj->positions[0].xdis)/pow(obj_speed,pow_num)*20;
      // cout << "lihui  objID: " << pObj->id << " ymean : "  << pObj->positions[size-1].ymean << endl;
      // cout << pObj->id << " lihui " << vfusion << " " << dismin << " " << obj_speed << endl;

      //predict select 1 target getin 2 in our window
      ivpredict::predictpos temp_result_pos;
      if ( (pObj->positions[size-1].ymean > 0 && (vfusion) < -1.5 && dismin < 3.9) || dismin < 1.75 || (pObj->positions[size-1].ymean < 0 && (vfusion) > 1.5 && dismin < 3.9))
      {

        cout << "lihui  objID: " << pObj->id << " ymean : "  << pObj->positions[size-1].ymean << endl;
        cout << pObj->id << " lihui " << vfusion << " " << dismin << " " << obj_speed << endl;

        // int next_point = 10;
        temp_result_pos.xmean = pObj->positions[size-1].xmean;
        temp_result_pos.ymean = pObj->positions[size-1].ymean;
        predict_result.positions.push_back(temp_result_pos);
        
//predict test 1
        for (int i = 0;i < TOTALOUTPUT/jump + 1; i++)//TOTALOUTPUT
        {

            // float dispoint = sqrt(Reference_path_line->points[choosen_point+next_point].x * Reference_path_line->points[choosen_point+next_point].x + Reference_path_line->points[choosen_point].x * Reference_path_line->points[choosen_point].x);
            // cout << "lihui choosen point : " << choosen_point << " predict_dis " << predict_dis << endl;
            for (int j = 0; j < Reference_path_line->points.size(); j++) //Reference_path_line->points.size()
            {
                float temp_dis1 = Reference_path_line->points[choosen_point+j].x - Reference_path_line->points[choosen_point].x;
                float temp_dis2 = Reference_path_line->points[choosen_point+j].y - Reference_path_line->points[choosen_point].y;
                float dispoint = sqrt(temp_dis1 * temp_dis1 + temp_dis2 * temp_dis2);
                // cout << "lihui dispoint check: " << predict_dis << endl;
                if(dispoint >  predict_dis*(i+1))// && dispoint < 20)
                {            
                    temp_result_pos.xmean = Reference_path_line->points[choosen_point+j].x;//(predict_result.positions[i].xmean + Reference_path_line->points[choosen_point+j*10].x)/2;
                    temp_result_pos.ymean = Reference_path_line->points[choosen_point+j].y;//(predict_result.positions[i].ymean + Reference_path_line->points[choosen_point+j*10].y)/2;
                    predict_result.positions.push_back(temp_result_pos);
                    // cout << "lihui point index : " << choosen_point+j*10 << endl;
                    break;
                }
            }
            
        }



        //cout << "lihui size : " << predict_result.positions.size() << endl;
        predict_result=SmoothPathx_relline(predict_result,3); //filter IAPredict.objects[i].v_abs
        //cout << "lihui size in : " << predict_result.positions.size() << endl;
        ivpredict::predictobj input_predict_result=CalPathIncludeVelUseCurveBx_relline(predict_result,3,obj_speed,5,0.25);
        //cout << "lihui size out : " << input_predict_result.positions.size() << endl;
        predict_result = input_predict_result;

        
        //method for 100 point
        // predict_result.positions.clear();
        // temp_result_pos.xmean = pObj->positions[size-1].xmean;
        // temp_result_pos.ymean = pObj->positions[size-1].ymean;
        // predict_result.positions.push_back(temp_result_pos);

        // for (int i = 0;i<input_predict_result.positions.size() - 1 ;i++)
        // {
        //   float disx = (input_predict_result.positions[i+1].xmean - input_predict_result.positions[i].xmean)/jump;
        //   float disy = (input_predict_result.positions[i+1].ymean - input_predict_result.positions[i].ymean)/jump;
        //   double angle = atan(disy/disx);
        //   for (int j = 1; j <= jump; j++)
        //   {
        //     temp_result_pos.xmean = input_predict_result.positions[i].xmean + disx * j;//disx * j;
        //     temp_result_pos.ymean = input_predict_result.positions[i].ymean + disy * j;
        //     predict_result.positions.push_back(temp_result_pos);
        //   }

        // }

        // // predict_result
        // // cout << "lihui size out final : " << predict_result.positions.size() << endl;
        // predict_result.positions.erase(predict_result.positions.end());
        // cout << "lihui size out final : " << predict_result.positions.size() << endl;
        //method for 100 point
//predict test 1


      }
      else // buffer too small
      {
        float disx,disy,dis,dismin = 9999;
        float nearx, neary;
        int size2 = pObj->positions.size();
        result_pos.xmean = pObj->positions[size2-1].xmean;
        result_pos.ymean = pObj->positions[size2-1].ymean;
        // for (int i = 0;i<TOTALOUTPUT;i++)
        // {
            predict_result.positions.push_back(result_pos);
        // }

        for(long j = 0; j < Reference_path_line->points.size(); j = j + 1)
        {
            int size = pObj->positions.size();
            disx = abs(pObj->positions[size-1].xmean - Reference_path_line->points[j].x);
            disy = abs(pObj->positions[size-1].ymean - Reference_path_line->points[j].y);
            dis = sqrt(disx * disx + disy * disy);
              // pointMsg.length += calc_distance(pointMsg_last, pointMsg);
            if (dis < dismin)
            {
              dismin = dis;
            }
        }
        float kalman_lihui = 0, predict_dismin = 0;
        if(size2 > 2)
        {
          kalman_lihui = pObj->positions[size2-2].xdis - pObj->positions[size2-3].xdis;
        }
        predict_dismin = pObj->positions[size2-3].xdis + kalman_lihui;
        dismin = (dismin + predict_dismin)/2;
        if (dismin > 3)
        {
          dismin = 3;
        }
        pObj->positions[size2-1].xdis = dismin;

      }
      // cout << "lihui objID: " << pObj->id << " dis-to-route: " << dismin << " car angle: " << ivlocpos_line.angle << endl;
      // cout << "lihui rel-dis-to-car: " << sqrt(pObj->x * pObj->x + pObj->y * pObj->y) << endl;
      // cout << "lihui nearpoint: x:" << nearx << " y: " << neary << endl;
      // printf("--------------------------------------------------\n");
        // result_pos.xmean = pObj->positions[size-1].xmean;
        // result_pos.ymean = pObj->positions[size-1].ymean;
        // for (int i = 0;i<TOTALOUTPUT;i++)
        // {
        //     predict_result.positions.push_back(result_pos);
        // }

    }
    else
    {
        float disx,disy,dis,dismin = 9999;
        float nearx, neary;
        int size2 = pObj->positions.size();
        result_pos.xmean = pObj->positions[size2-1].xmean;
        result_pos.ymean = pObj->positions[size2-1].ymean;
        // for (int i = 0;i<TOTALOUTPUT;i++)
        // {
            predict_result.positions.push_back(result_pos);
        // }

        // if (pObj->positions.size() < predict_buffer_size)
        {
            for(long j = 0; j < Reference_path_line->points.size(); j = j + 1)
            {
                int size = pObj->positions.size();
                disx = abs(pObj->positions[size-1].xmean - Reference_path_line->points[j].x);
                disy = abs(pObj->positions[size-1].ymean - Reference_path_line->points[j].y);
                dis = sqrt(disx * disx + disy * disy);
                  // pointMsg.length += calc_distance(pointMsg_last, pointMsg);
                if (dis < dismin)
                {
                  dismin = dis;
                }
            }
            float kalman_lihui = 0, predict_dismin = 0;
            if(size2 > 2)
            {
              kalman_lihui = pObj->positions[size2-2].xdis - pObj->positions[size2-3].xdis;
            }
            predict_dismin = pObj->positions[size2-3].xdis + kalman_lihui;
            dismin = (dismin + predict_dismin)/2;
            if (dismin > 3)
            {
              dismin = 3;
            }
            pObj->positions[size2-1].xdis = dismin;
        }

        //find nearest distance
       
        //cout << " lihui dis " << dismin << endl;
    }

    predict_result.id = pObj->id;
    predict_result.length = pObj->length;
    predict_result.width = pObj->width;
    predict_result.height = pObj->height;
    predict_result.cell = pObj->cell;
    predict_result.time_interval = 1./predict_result.positions.size() ;
    predict_result.steps = TOTALOUTPUT;
    predict_result.v_abs = pObj->vabs;
    predict_result.xvrel = pObj->xvrel;
    predict_result.yvrel = pObj->yvrel;
    predict_result.vrel = pObj->vrel;
    predict_result.type = pObj->property;
    return predict_result;
}



#endif    
