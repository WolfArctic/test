/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: predictusefun.h
Description: 
1. userfunction rel-position predict (static position)
2. userfunction GPS-position predict (static position)

History:
<author>    <time>      <version>    <description>
hui li       17/06/06     1.0.0        achieve userfunction predict(not tested)  
                                  
************************************************************/

#ifndef _IVPREDICTUSERFUN_H
#define _IVPREDICTUSERFUN_H
#pragma once

typedef struct{
    double  lat;
    double  lon;
}sGpspoint, *pGpspoint;

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
static ivpredict::predictobj PredictUserFuncRel(ivmap::mapobject userfun_relobj, int frequency_in)
{
    int totaloutput = frequency_in*5;
	ivpredict::predictobj predict_result;
    ivpredict::predictpos result_pos;

    predict_result.id = userfun_relobj.id;
    predict_result.length = userfun_relobj.length;
    predict_result.width = userfun_relobj.width;
    predict_result.height = userfun_relobj.height;
    
    int cell_num = userfun_relobj.cell.size();
    ivpredict::mapobjcell temp_cell;
    for (int k = 0; k<cell_num;k++)
    {
        temp_cell.xc = userfun_relobj.cell[k].xc;
        temp_cell.yc = userfun_relobj.cell[k].yc;
        predict_result.cell.push_back(temp_cell);
    }

    predict_result.time_interval = 1./frequency_in ;
    predict_result.steps = totaloutput;
    predict_result.xvrel = userfun_relobj.vxrel;
    predict_result.yvrel = userfun_relobj.vyrel;
    predict_result.vrel = userfun_relobj.vrel;
    predict_result.type = userfun_relobj.type;

    result_pos.xmean = userfun_relobj.x;
    result_pos.ymean = userfun_relobj.y;

    for (int i = 0;i<totaloutput;i++)
    {
        predict_result.positions.push_back(result_pos);
    }
    return predict_result;
}
/**************************************************************
Function: PredictUserFuncRel
Author: hui li
Date: 17/06/06
Description: 
1. get the GPS position of the object
2. predict the user object at the same position in relative position
 
Input: user object, car pose
Output: predict object
Return: predict object
Others: 
************************************************************/
static ivpredict::predictobj PredictUserFuncGPS(ivmap::mapobject userfun_gpsobj, ivmap::ivmapmsglocpos locpos,int frequency_in)
{
	ivpredict::predictobj predict_result;
	ivpredict::predictpos result_pos;
    int TOTALOUTPUT = frequency_in*5;

	predict_result.id = userfun_gpsobj.id;
    predict_result.length = userfun_gpsobj.length;
    predict_result.width = userfun_gpsobj.width;
    predict_result.height = userfun_gpsobj.height;

    int cell_num = userfun_gpsobj.cell.size();
	ivpredict::mapobjcell temp_cell;
	for (int k = 0; k<cell_num;k++)
	{
	    temp_cell.xc = userfun_gpsobj.cell[k].xc;
	    temp_cell.yc = userfun_gpsobj.cell[k].yc;
	    predict_result.cell.push_back(temp_cell);
	}

    predict_result.time_interval = 1./frequency_in ;
    predict_result.steps = TOTALOUTPUT;
    predict_result.xvrel = userfun_gpsobj.vxrel;
    predict_result.yvrel = userfun_gpsobj.vyrel;
    predict_result.vrel = userfun_gpsobj.vrel;
    predict_result.type = userfun_gpsobj.type;

	geotool gt;
    ivlocmsg::ivsensorgps iabasemapTL,objgps,carpos;
    sPointOfGCCS temp_result;
    sPointOfVCS temp_answer;

    ros::NodeHandle mh;
    mh.param("iabasemaptllon",iabasemapTL.lon,iabasemapTL.lon);
    mh.param("iabasemaptllat",iabasemapTL.lat,iabasemapTL.lat);
	iabasemapTL.heading = 90;
	// iabasemapTL.lon = GGPSY;
 //    iabasemapTL.lat = GGPSX; 
   	objgps.lon = userfun_gpsobj.lon;
    objgps.lat = userfun_gpsobj.lat;
    carpos.lon = locpos.lon;
	carpos.lat = locpos.lat;
	carpos.heading = locpos.heading;

    temp_result = gt.GPS2GCCS(iabasemapTL,objgps);
    temp_answer = gt.GCCS2VCS(carpos,temp_result);

	result_pos.xmean = temp_answer.x;
    result_pos.ymean = temp_answer.y;

	for (int i = 0;i<TOTALOUTPUT;i++)
    {
        predict_result.positions.push_back(result_pos);
    }
    return predict_result;
}
#endif    