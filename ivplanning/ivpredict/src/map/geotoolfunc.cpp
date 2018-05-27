
#include "geotoolfunc.h"

/**************************************************************
Function: PredictGCC2VCS
Author: hui li
Date: 17/06/12
Description: 
1. change the obsolute posstion to rel-position and store

Input: void
Output: void
Return: void
Others: 
************************************************************/
geotoolfunc::geotoolfunc()
{}
geotoolfunc::~geotoolfunc()
{}


int geotoolfunc::PredictGCC2VCS(ivpredict::ivmsgpredict *target_Predict, ivmap::ivmapmsglocpos *pIvlocpos)
{
  geotool gt;
  sPointOfGCCS carPos;
  carPos.xg = pIvlocpos->xg;
  carPos.yg = pIvlocpos->yg;
  carPos.angle = pIvlocpos->angle;

  sPointOfGCCS source;
  sPointOfVCS result;
  float movex,movey;
  //ivpredict::predictpos result_pos;
  for (int i = 0; i < target_Predict->objects.size(); i++)
  {
    //get the origin x and y
    source.xg = target_Predict->objects[i].positions[0].xgmean;
    source.yg = target_Predict->objects[i].positions[0].ygmean;
    result = gt.GCCS2VCS(carPos,source);
    movex = result.x - target_Predict->objects[i].positions[0].xmean;
    movey = result.y - target_Predict->objects[i].positions[0].ymean;

    for (int j = 1; j < target_Predict->objects[i].positions.size(); j++)
    {
      source.xg = target_Predict->objects[i].positions[j].xgmean;
      source.yg = target_Predict->objects[i].positions[j].ygmean;
      result = gt.GCCS2VCS(carPos,source);
      target_Predict->objects[i].positions[j].xmean = result.x - movex;
      target_Predict->objects[i].positions[j].ymean = result.y - movey;
    }
  }
  return 0;
}

int geotoolfunc::PredictGCC2VCS_one(ivpredict::predictobj* predict_result, ivmap::ivmapmsglocpos *pIvlocpos)
{
  geotool gt;
  sPointOfGCCS carPos;
  carPos.xg = pIvlocpos->xg;
  carPos.yg = pIvlocpos->yg;
  carPos.angle = pIvlocpos->angle;

  sPointOfGCCS source;
  sPointOfVCS result;

  for (int j = 0; j < predict_result->positions.size(); j++)
  {
    source.xg = predict_result->positions[j].xgmean;
    source.yg = predict_result->positions[j].ygmean;
    result = gt.GCCS2VCS(carPos,source);
    predict_result->positions[j].xmean = result.x;
    predict_result->positions[j].ymean = result.y;
  }

  return 0;
}