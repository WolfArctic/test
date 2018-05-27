/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: buffer.cpp
Description: 
1. 
2. 
3. 

History:
<author>    <time>      <version>    <description> 
hui li      17/11/10    1.0.0       buffer moved to container  

************************************************************/
#include "buffer.h"

buffer::buffer()
{
}

buffer::~buffer()
{
}

/**************************************************************
Function: buffer Update
Author: fang zhang
Date: 17/04/15
Description: 
1. store object information from iamap
2. if a new object, creat a new one or update the invalid object
3. if an exist object, update object information when buffer is full (BUFFER_SIZE) 
4. if an object disappear, set the ID INVALID_OBJECT_ID
Input: iamap
Output: void
Return: void
Others: store value in Buffer.obj_info (declear in .h file)
************************************************************/
int buffer::Update(sIAMap iamap)
{
  int num_obj = iamap.ivobj.obj.size();
  int num_buffer = obj_info.objects.size();

  //buffer size() all objs
  //if (num_obj == 0)
  //{
  //  cout << " ivpredict-omega: buffer input size 0! " << endl;
  //  return 1;
  //}

  //check all objs
  for (int i = 0; i < num_obj; i++)
  {
    int new_obj_flag = 1;
    int double_obj_flag = 1;
    int tempid = iamap.ivobj.obj[i].id;

    //check exist obj
    for (int j = 0;j < num_buffer;j++) //
    {
      if (tempid == obj_info.objects[j].id)
      {
        //new_obj.property = iamap.ivobj.obj[i].type;
        //exchange only
        ivpredict::predictposbuffer new_pos;
        obj_info.objects[j].length = iamap.ivobj.obj[i].length;
        obj_info.objects[j].width= iamap.ivobj.obj[i].width;
        obj_info.objects[j].height = iamap.ivobj.obj[i].height;
        obj_info.objects[j].property = iamap.ivobj.obj[i].type;
        obj_info.objects[j].xvrel = iamap.ivobj.obj[i].vxrel;
        obj_info.objects[j].yvrel = iamap.ivobj.obj[i].vyrel;
        obj_info.objects[j].vrel = iamap.ivobj.obj[i].vrel;
        if (iamap.ivobj.obj[i].x == obj_info.objects[j].positions[obj_info.objects[j].positions.size()-1].xmean &&
          iamap.ivobj.obj[i].y == obj_info.objects[j].positions[obj_info.objects[j].positions.size()-1].ymean)
        {
          new_obj_flag = 0;
          obj_info.objects[j].update = false;
          // double_obj_flag = 0;
          break;
        }
        obj_info.objects[j].update = true;
        new_pos.xmean = iamap.ivobj.obj[i].x;
        new_pos.ymean = iamap.ivobj.obj[i].y; 
        int cell_num = iamap.ivobj.obj[i].cell.size();
        ivpredict::mapobjcell temp_cell;
        obj_info.objects[j].cell.clear();
        for (int k = 0; k<cell_num;k++)
        {
            temp_cell.xc = iamap.ivobj.obj[i].cell[k].xc;
            temp_cell.yc = iamap.ivobj.obj[i].cell[k].yc;
            temp_cell.x = iamap.ivobj.obj[i].cell[k].x;
            temp_cell.y = iamap.ivobj.obj[i].cell[k].y;
            obj_info.objects[j].cell.push_back(temp_cell);
        }

        //save obj positions               
        new_pos.xgmean = iamap.ivobj.obj[i].xg;
        new_pos.ygmean = iamap.ivobj.obj[i].yg;
        new_pos.vxabs = iamap.ivobj.obj[i].vxabs;
        new_pos.vyabs = iamap.ivobj.obj[i].vyabs;
        new_pos.vabs = iamap.ivobj.obj[i].vabs;
        new_pos.timestamp = ros::Time::now(); //.toSec()
        int buffer_number = obj_info.objects[j].positions.size();
        if (obj_info.objects[j].positions.size() < FREQUENCY)
        {
          obj_info.objects[j].xvabs = iamap.ivobj.obj[i].vxabs;
          obj_info.objects[j].yvabs = iamap.ivobj.obj[i].vyabs;
          obj_info.objects[j].vabs = iamap.ivobj.obj[i].vabs;                              
          obj_info.objects[j].positions.push_back(new_pos);
        }
        else
        {          
          switch (predict_lidar)
          {
            //read position and speed direct
            //case 0:
            case 0:
            {
              obj_info.objects[j].xvabs = iamap.ivobj.obj[i].vxabs;
              obj_info.objects[j].yvabs = iamap.ivobj.obj[i].vyabs;
              obj_info.objects[j].vabs = sqrt(pow(iamap.ivobj.obj[i].vxabs,2)+pow(iamap.ivobj.obj[i].vyabs,2));   
              break;
            }

            //use kalman for speed and positions
            //case 1:
            case 1:
            {

              int buffer_number = obj_info.objects[j].positions.size();
              Eigen::Matrix<double,4,1> temp_vector = AbsolutePositionKalmanPredict(obj_info.objects[j].positions[buffer_number-1].xgmean,obj_info.objects[j].xvabs, obj_info.objects[j].positions[buffer_number-1].ygmean, obj_info.objects[j].yvabs, 1.0/FREQUENCY, iamap.ivobj.obj[i].xg,iamap.ivobj.obj[i].yg);  
              new_pos.xgmean = temp_vector(0);
              new_pos.ygmean = temp_vector(2);
              obj_info.objects[j].xvabs = temp_vector(1);
              obj_info.objects[j].yvabs = temp_vector(3);             
              // obj_info.objects[j].vabs = sqrt(temp_vector(1)*temp_vector(1) + temp_vector(3)*temp_vector(3));
              // Eigen::Matrix<double,4,1> temp_vector2 = AbsolutePositionKalmanPredict(obj_info.objects[j].positions[buffer_number-1].xmean,obj_info.objects[j].xvrel, obj_info.objects[j].positions[buffer_number-1].ymean, obj_info.objects[j].yvrel, 1.0/FREQUENCY, iamap.ivobj.obj[i].x,iamap.ivobj.obj[i].y);  
              // new_pos.xmean = temp_vector(0);
              // new_pos.ymean = temp_vector(2);
              // obj_info.objects[j].xvrel = temp_vector(1);
              // obj_info.objects[j].yvrel = temp_vector(3);             
              // obj_info.objects[j].vrel = sqrt(temp_vector(1)*temp_vector(1) + temp_vector(3)*temp_vector(3));
              break;
            }
            case 2:
            {
              float v_objx = 0,v_objy = 0,v_sumx = 0,v_sumy = 0;
              for (int i = obj_info.objects[j].positions.size()-FREQUENCY; i < (obj_info.objects[j].positions.size()-1); i++)
              {
                v_objx = obj_info.objects[j].positions[i+1].xmean - obj_info.objects[j].positions[i].xmean;
                v_objy = obj_info.objects[j].positions[i+1].ymean - obj_info.objects[j].positions[i].ymean;
                v_sumx = v_sumx +  v_objx;
                v_sumy = v_sumy +  v_objy;
              }
              obj_info.objects[j].xvabs = v_sumx/(FREQUENCY-1);
              obj_info.objects[j].yvabs = v_sumy/(FREQUENCY-1);             
              obj_info.objects[j].vabs = sqrt(obj_info.objects[j].xvabs*obj_info.objects[j].xvabs + obj_info.objects[j].yvabs*obj_info.objects[j].yvabs);
              break;
            }
            default:
            {
              //cout << "ivpredict-omega: lidar methods wrong!" << endl;
              //predMonitor->sendWarnning(2, 0); 
            }
          }
          obj_info.objects[j].positions.erase(obj_info.objects[j].positions.begin());
          obj_info.objects[j].positions.push_back(new_pos);
        }
        new_obj_flag = 0;
        break;
      }
    }
      // find a new object
      if (new_obj_flag == 1)
      {
        ivpredict::objbuffer new_obj;
        ivpredict::predictposbuffer new_pos;
        new_obj.id = tempid;
        new_obj.property = iamap.ivobj.obj[i].type;
        new_obj.length = iamap.ivobj.obj[i].length;
        new_obj.width= iamap.ivobj.obj[i].width;
        new_obj.height= iamap.ivobj.obj[i].height;
        new_obj.xvrel= iamap.ivobj.obj[i].vxrel;
        new_obj.yvrel= iamap.ivobj.obj[i].vyrel;
        new_obj.vrel= iamap.ivobj.obj[i].vrel;               
        new_obj.xvabs = iamap.ivobj.obj[i].vxabs;
        new_obj.yvabs = iamap.ivobj.obj[i].vyabs;
        new_obj.vabs = iamap.ivobj.obj[i].vabs; 
        new_obj.update = true;
        int cell_num = iamap.ivobj.obj[i].cell.size();
        ivpredict::mapobjcell temp_cell;
        for (int k = 0; k<cell_num;k++)
        {
            temp_cell.xc = iamap.ivobj.obj[i].cell[k].xc;
            temp_cell.yc = iamap.ivobj.obj[i].cell[k].yc;
            temp_cell.x = iamap.ivobj.obj[i].cell[k].x;
            temp_cell.y = iamap.ivobj.obj[i].cell[k].y;
            new_obj.cell.push_back(temp_cell);
        }
      
        //save obj in GCCS                 
        new_pos.xmean = iamap.ivobj.obj[i].x;
        new_pos.ymean = iamap.ivobj.obj[i].y; 
        new_pos.xgmean = iamap.ivobj.obj[i].xg;
        new_pos.ygmean = iamap.ivobj.obj[i].yg; 
        new_pos.vxabs = iamap.ivobj.obj[i].vxabs;
        new_pos.vyabs = iamap.ivobj.obj[i].vyabs;
        new_pos.vabs = iamap.ivobj.obj[i].vabs;
        new_pos.timestamp = ros::Time::now();
        new_obj.positions.push_back(new_pos);
        new_obj.change_time = 88;                             
        
        int insert_flag = 0;
        for (int j = 0; j < num_buffer;j++)
        {
          if (obj_info.objects[j].id == INVALID_OBJECT_ID)
          {
              obj_info.objects[j].id = tempid;
              obj_info.objects[j].property = iamap.ivobj.obj[i].type;
              obj_info.objects[j].positions.clear();
              obj_info.objects[j].positions.push_back(new_pos);
              obj_info.objects[j].length = iamap.ivobj.obj[i].length;
              obj_info.objects[j].width = iamap.ivobj.obj[i].width;
              obj_info.objects[j].height = iamap.ivobj.obj[i].height;
              obj_info.objects[j].xvrel= iamap.ivobj.obj[i].vxrel;
              obj_info.objects[j].yvrel= iamap.ivobj.obj[i].vyrel;
              obj_info.objects[j].vrel= iamap.ivobj.obj[i].vrel; 
              int cell_num = iamap.ivobj.obj[i].cell.size();
              ivpredict::mapobjcell temp_cell;
              obj_info.objects[j].cell.clear();
              for (int k = 0; k<cell_num;k++)
              {
                  temp_cell.xc = iamap.ivobj.obj[i].cell[k].xc;
                  temp_cell.yc = iamap.ivobj.obj[i].cell[k].yc;
                  temp_cell.x = iamap.ivobj.obj[i].cell[k].x;
                  temp_cell.y = iamap.ivobj.obj[i].cell[k].y;
                  obj_info.objects[j].cell.push_back(temp_cell);
              }
              obj_info.objects[j].update = true;
              obj_info.objects[j].xvabs = 0;
              obj_info.objects[j].yvabs = 0;
              obj_info.objects[j].vabs = 0;
              obj_info.objects[j].change_time = 88;
              insert_flag = 1;
              break;
          }
        }      
        if (insert_flag == 0)
        {
            obj_info.objects.push_back(new_obj);
        }
      }
    
  }

  // count how many times the object is not updated
  for (int i = 0; i < num_buffer;i++)
  {
    int inactive_flag = 1;
    for (int j = 0; j< num_obj;j++)
    {
      if (iamap.ivobj.obj[j].id == obj_info.objects[i].id)
      {
          obj_info.objects[i].inactivecount = 0;
          inactive_flag = 0;
          break;
      }
    }

    if (inactive_flag == 1)
    {
      obj_info.objects[i].inactivecount ++ ;
      if (obj_info.objects[i].inactivecount > 0)
      {
          obj_info.objects[i].id = INVALID_OBJECT_ID;
          obj_info.objects[i].positions.clear();
      }
    }
  }
  return 0;
 // Buffer.obj_info = obj_info;

}
