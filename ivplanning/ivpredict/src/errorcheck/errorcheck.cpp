/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: predictclass.cpp
Description: 
1. main function of ivpredict
2. message subscribe
3. 

History:
<author>    <time>      <version>    <description>

************************************************************/
#include "errorcheck.h"
#include "../glovariable.h"
//#include "omegapredict.h"
errorcheck::errorcheck()
{
  // predictMonitor = new Monitor(20);  
}

errorcheck::~errorcheck()
{
    // delete predictMonitor;
    // predictMonitor = NULL;
}

int errorcheck::BuffermsgCheck(ivpredict::ivpredictbuffer* buffer_check)
{
  int buffersize = buffer_check->objects.size();
  int msg_buffer_size = 10;
  int error_msg = 0;
  if (buffersize == 0)
  {
    
    //cout << "ivpredict-omega: buffer output size 0! " << endl;
  
    return 1;
  }
  int check_count[15] = {0};
  for (int i = 0; i < buffer_check->objects.size(); ++i)
  {
    // if (buffer_check->objects[i].id == buffer_check->objects[0].id)
    // {
    //   check_count[0]++;
    //   if (check_count[0] == 1)
    //   {
    //     cout << "ivpredict-omega-buffer error : HAVE same ID " << endl;
    //     error_msg = 10;
    //   }
    // }
    for (int j = i+1; j < buffer_check->objects.size(); ++j)
    {
      if (buffer_check->objects[j].id == buffer_check->objects[i].id && buffer_check->objects[j].id != -88)
      {
       
        //cout << "ivpredict-omega-buffer error : HAVE same ID " << buffer_check->objects[i].id << endl;
       
        error_msg = 10;
      }
    }
    
    if (buffer_check->objects[i].property == buffer_check->objects[0].property)
    {
      check_count[1]++;
      if (check_count[1] == (buffer_check->objects.size()-1))
      {
       
        //cout << "ivpredict-omega-buffer error : ALL same property " << endl;
     
        error_msg = 11;
      }
    }
    if (buffer_check->objects[i].length == buffer_check->objects[0].length)
    {
      check_count[3]++;
      if (check_count[3] == (buffer_check->objects.size()-1))
      {
               //cout << "ivpredict-omega-buffer error : ALL same length/width/height " << endl;
               error_msg = 13;
      }
    }
    if (buffer_check->objects[i].cell.size() == buffer_check->objects[0].cell.size())
    {
      check_count[4]++;
      if (check_count[4] == (buffer_check->objects.size()-1))
      {
            //cout << "ivpredict-omega-buffer error : ALL same cell size " << endl;
                error_msg = 14;
      }
    }
    if (buffer_check->objects[i].xvrel == buffer_check->objects[0].xvrel)
    {
      check_count[5]++;
      if (check_count[5] == (buffer_check->objects.size()-1))
      {
             //cout << "ivpredict-omega-buffer error : ALL same xvrel " << endl;
              error_msg = 15;
      }
    }
    if (buffer_check->objects[i].yvrel == buffer_check->objects[0].yvrel)
    {
      check_count[6]++;
      if (check_count[6] == (buffer_check->objects.size()-1))
      {
        //cout << "ivpredict-omega-buffer error : ALL same yvrel " << endl;
        error_msg = 16;
      }
    }
    if (buffer_check->objects[i].vrel == buffer_check->objects[0].vrel)
    {
      check_count[7]++;
      if (check_count[7] == (buffer_check->objects.size()-1))
      {
       //cout << "ivpredict-omega-buffer error : ALL same vrel " << endl;
     error_msg = 17;
      }
    }
    if (buffer_check->objects[i].xvabs == buffer_check->objects[0].xvabs)
    {
      check_count[8]++;
      if (check_count[8] == (buffer_check->objects.size()-1))
      {
        //cout << "ivpredict-omega-buffer error : ALL same xvabs " << endl;
        error_msg = 18;
      }
    }
    if (buffer_check->objects[i].yvabs == buffer_check->objects[0].yvabs)
    {
      check_count[9]++;
      if (check_count[9] == (buffer_check->objects.size()-1))
      {
        //cout << "ivpredict-omega-buffer error : ALL same yvabs " << endl;
        error_msg = 19;
      }
    }
    if (buffer_check->objects[i].vabs == buffer_check->objects[0].vabs)
    {
      check_count[10]++;
      if (check_count[10] == (buffer_check->objects.size()-1))
      {
     //cout << "ivpredict-omega-buffer error : ALL same vabs " << endl;
        error_msg = 20;
      }
    }

  }
  return error_msg;
}

int errorcheck::PredictmsgCheck(ivpredict::ivmsgpredict* predictobj_check)
{
  int buffersize = predictobj_check->objects.size();
  int msg_buffer_size = 10;
  int error_msg = 0;
  if (buffersize == 0)
  {
   
    //cout << "ivpredict-omega: predict final output size 0! " << endl;
   
    return 1;
  }
  int check_count[15] = {0};
  for (int i = 0; i < predictobj_check->objects.size(); ++i)
  {
    for (int j = i+1; j < predictobj_check->objects.size(); ++j)
    {
      if (predictobj_check->objects[j].id == predictobj_check->objects[i].id && predictobj_check->objects[j].id != -88)
      {
        
        //cout << "ivpredict-omega-output error : HAVE same ID " << predictobj_check->objects[i].id << endl;
       
        error_msg = 10;
      }
    }

    if (predictobj_check->objects[i].type == predictobj_check->objects[0].type)
    {
      check_count[1]++;
      if (check_count[1] == (predictobj_check->objects.size()-1))
      {
        //cout << "ivpredict-omega-output error : ALL same type " << endl;
        error_msg = 11;
      }
    }
    if (predictobj_check->objects[i].time_interval * predictobj_check->objects[0].steps != gTimeLength)
    {
      //cout << "ivpredict-omega-output time or steps wrong! " << endl;
      error_msg = 13;
    }
    if (predictobj_check->objects[i].length == predictobj_check->objects[0].length)
    {
      check_count[2]++;
      if (check_count[2] == (predictobj_check->objects.size()-1))
      {
        //cout << "ivpredict-omega-output error : ALL same length/width/height" << endl;
        error_msg = 12;
      }
    }
    if (predictobj_check->objects[i].cell.size() == predictobj_check->objects[0].cell.size())
    {
      check_count[4]++;
      if (check_count[4] == (predictobj_check->objects.size()-1))
      {
      //cout << "ivpredict-omega-output error : ALL same cell size " << endl;
       error_msg = 14;
      }
    }
    if (predictobj_check->objects[i].xvrel == predictobj_check->objects[0].xvrel)
    {
      check_count[5]++;
      if (check_count[5] == (predictobj_check->objects.size()-1))
      {
 //cout << "ivpredict-omega-output error : ALL same xvrel " << endl;
       error_msg = 15;
      }
    }
    if (predictobj_check->objects[i].yvrel == predictobj_check->objects[0].yvrel)
    {
      check_count[6]++;
      if (check_count[6] == (predictobj_check->objects.size()-1))
      {
     //cout << "ivpredict-omega-output error : ALL same yvrel " << endl;
      error_msg = 16;
      }
    }
    if (predictobj_check->objects[i].vrel == predictobj_check->objects[0].vrel)
    {
      check_count[7]++;
      if (check_count[7] == (predictobj_check->objects.size()-1))
      {
        //cout << "ivpredict-omega-output error : ALL same vrel " << endl;
        error_msg = 17;
      }
    }
    if (predictobj_check->objects[i].v_abs == predictobj_check->objects[0].v_abs)
    {
      check_count[8]++;
      if (check_count[8] == (predictobj_check->objects.size()-1))
      {
        
        //cout << "ivpredict-omega-output error : ALL same v_abs " << endl;
       
        error_msg = 18;
      }
    }
  }
  return error_msg;
}

void errorcheck::Inputcheck(ivmap::ivmapmsgobj* inputobj_check)
{
  bool check_by_hand = true;
  if (check_by_hand == true)
  {
    for (int i = 0; i < inputobj_check->obj.size(); ++i)
    {
      //cout << " ivpredict-debug by hand (rel): " << (int)inputobj_check->obj[i].id << " " << inputobj_check->obj[i].x << " " << inputobj_check->obj[i].y << " " << inputobj_check->obj[i].vxrel << " " << inputobj_check->obj[i].vyrel << endl;
     
      //cout << " ivpredict-debug by hand (abs): " << (int)inputobj_check->obj[i].id << " " << inputobj_check->obj[i].xg << " " << inputobj_check->obj[i].yg << " " << inputobj_check->obj[i].vxabs << " " << inputobj_check->obj[i].vyabs << " " << inputobj_check->obj[i].vabs << endl;   
    }
  }
}
