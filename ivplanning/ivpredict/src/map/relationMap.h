
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
hui li    2017/04/15     2.0.0        creat         
zongwei   2017/06/15     2.0.1        format and simplify the document, modidfy function names
************************************************************/
#ifndef RELATIONMAP_H
#define RELATIONMAP_H

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
#include "../../../../avoslib/geotool.h"
#include "../globaldefine.h"

using namespace std;
#define EARTHRADIUS 6378137.0 //6378245
#define JUMPPREDICT 5
// #define GGPSY 116.324562809
// #define GGPSX 40.0003363001

int temCount=0;
string fileroute;

sSegRel LoadSegment(string type, int number)
{
    sSegRel segment_temp;
    int temp_num = 0;
    sGCCSpoint temp_GCCS;
    // sGPSpoint temp_sGPSpoint;
    // ifstream segmentfile;
    string segmentfile_txt = fileroute + "/" +  to_string(number) + "-seg";
    std::ifstream fin(segmentfile_txt.c_str());
    std::string   line,temp;
    int temp_jump = 0;
    // sRelationList temp_list;
    int temp_id;
   while(getline(fin,line)){
    temp = "";
    temp_jump ++;
    // temp_list.segmentType = loadtype;
        //ivlocmsg::ivsensorgps rp;
        // sBoundaryPoint rp;
    int strnum = 0;
    for (unsigned int i = 0; i < line.size(); ++i){
      if (line[i] == ',' ){
        // list_number++;
        std::stringstream stream;
        stream << temp;
        switch(strnum){   
                    case 13:{
                         stream >> temp_GCCS.xg;
                         //temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 14:{
                         stream >> temp_GCCS.yg;
                         //temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    // case 16:{
                    //      stream >> temp_id;
                    //      temp_list.parallelID.push_back(temp_id);
                    //      stream.str(""); 
                    //      temp=""; 
                    //      break;
                    // }
                    // case 17:{
                    //      stream >> temp_id;
                    //      temp_list.parallelID.push_back(temp_id);
                    //      stream.str(""); 
                    //      temp=""; 
                    //      break;
                    // }
                    default:{
                         stream.str(""); 
                         temp="";
                         break;
                    }   
                }
        strnum++;
      }
      else {
        temp += line[i];
      }
    }
    if (temp_jump == JUMPPREDICT)
    {
      segment_temp.GCCS.push_back(temp_GCCS);
      temp_jump = 0;
      temp_num++;
    }

    
  }
  fin.close();

    segment_temp.segmenttype = "c";
    segment_temp.segmentnumber = number;
    segment_temp.maxLineNum = temp_num;
    // segment_temp = GetGCCS(segment_temp);
    return segment_temp;
} 

sMapInformation LoadMap(string inputType)
{
  //get the segments file
  ros::NodeHandle n;
  n.getParam("routemap", fileroute);
  sMapInformation gLoadedMap;
  sMapInformationAll outputMap;
  int list_number = 0;
  string crosslines_name;
  string crosslines_number;
  string loadtype = inputType;
  // ifstream crosslines;
  string crosslines_txt = fileroute + "/" + "relationbegin_var";

  std::ifstream fin(crosslines_txt.c_str());
  std::string   line,temp;
  sRelationList temp_list;
  int temp_id;
   while(getline(fin,line)){
    temp = "";
    temp_list.fatherID.clear();
    temp_list.sonID.clear();
    temp_list.parallelID.clear();
    temp_list.segmentType = loadtype;
        //ivlocmsg::ivsensorgps rp;
        // sBoundaryPoint rp;
    int strnum = 0;
    for (unsigned int i = 0; i < line.size(); ++i){

      if (line[i] == ',' ){
        list_number++;
        std::stringstream stream;
        stream << temp;
        switch(strnum){   
                    case 0:{
                         stream >> temp_list.segmentNumber;
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 1:{
                         stream >> temp_list.length;
                         stream.str(""); 
                         temp=""; 
                        break;
                    }
                    case 2:{
                         stream >> temp_list.speed;
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 3:{
                         stream >> temp_id;
                         temp_list.fatherID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 4:{
                         stream >> temp_id;
                         temp_list.fatherID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 5:{
                         stream >> temp_id;
                         temp_list.fatherID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 6:{
                         stream >> temp_id;
                         temp_list.fatherID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 7:{
                         stream >> temp_id;
                         temp_list.fatherID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 8:{
                         stream >> temp_id;
                         temp_list.sonID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 9:{
                         stream >> temp_id;
                         temp_list.sonID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 10:{
                         stream >> temp_id;
                         temp_list.sonID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 11:{
                         stream >> temp_id;
                         temp_list.sonID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 12:{
                         stream >> temp_id;
                         temp_list.sonID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 13:{
                         stream >> temp_id;
                         temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 14:{
                         stream >> temp_id;
                         temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 15:{
                         stream >> temp_id;
                         temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 16:{
                         stream >> temp_id;
                         temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    case 17:{
                         stream >> temp_id;
                         temp_list.parallelID.push_back(temp_id);
                         stream.str(""); 
                         temp=""; 
                         break;
                    }
                    default:{
                         stream.str(""); 
                         temp="";
                         break;
                    }   
                }
        strnum++;
      }
      else {
        temp += line[i];
      }
    }
    gLoadedMap.mysRelationship.list.push_back(temp_list);
  }
  fin.close();

  sSegRel segments_buffer;   
  for (int i = 0; i < gLoadedMap.mysRelationship.list.size(); i++)
  {
      segments_buffer = LoadSegment(gLoadedMap.mysRelationship.list[i].segmentType,gLoadedMap.mysRelationship.list[i].segmentNumber);
      gLoadedMap.segments_buffer.push_back(segments_buffer);
      //printf("%f\n", segments_buffer[i].segment[1].x);
  }
  // outputMap.mapType = inputType;
  // outputMap.mapStructure.push_back(gLoadedMap)

  return gLoadedMap;
}

#endif