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
fang zhang  17/04/15    1.0.0       initialize 
hui li      17/06/06    1.0.1       add predictuserfun declaration 
hui li      17/06/09    1.1.0       add absolute position PredictManager function
hui li      17/06/13    1.1.1       add map, write ivrviz(map.h,map.cpp,loadmap.h,ivrviz.cpp,CMakelist.txt,package.xml) and ivtf(tf.cpp) with Yang Xia
hui li      17/06/14    1.1.2       add Kalman filter
hui li      17/06/16    1.1.3       add temp_vx,vy,vabs; add car object limit
zongwei     17/06/15    1.2.1       format and simplify the document, modidfy function names
zongwei     17/06/15    1.2.2       format and simplify the document, modidfy function names. meantime, add function PredictVCS2GCC().
hui li      17/11/15    1.2.3       re-struct ivpredict add monitor(lnh) 
************************************************************/
#include "predict_manager.h"

PredictManager::PredictManager(ros::NodeHandle mh)
{
    //sub
    sub1_ = mh.subscribe(SUB_TOPIC_OBJ,1000,&PredictManager::chatterCallbackObj,this);
    sub2_ = mh.subscribe(SUB_TOPIC_ROAD,1000,&PredictManager::chatterCallbackRoad,this);
    sub3_ = mh.subscribe(SUB_TOPIC_VSP,1000,&PredictManager::chatterCallbackVsp,this);
    sub4_ = mh.subscribe(SUB_TOPIC_VAP,1000,&PredictManager::chatterCallbackVap,this);
    sub5_ = mh.subscribe(SUB_TOPIC_APP,1000,&PredictManager::chatterCallbackApp,this);
    sub6_ = mh.subscribe(SUB_TOPIC_USER,1000,&PredictManager::chatterCallbackUser,this);
    sub7_ = mh.subscribe(SUB_TOPIC_TRAFFIC,1000,&PredictManager::chatterCallbackTraffic,this);
    sub8_ = mh.subscribe(SUB_TOPIC_LOCPOS,1000,&PredictManager::chatterCallbackLocpos,this);
    sub9_ = mh.subscribe(SUB_TOPIC_LOCTAG,1000,&PredictManager::chatterCallbackLoctag,this);
    sub_motionplanner = mh.subscribe("ivmotionplanner", 1000, &PredictManager::subCallback_motionplanner, this);
    
    //pub
    pub1_ = mh.advertise<ivpredict::ivmsgpredict>(PUB_TOPIC_IAPREDICT,1000);
    pub_predictDebug = mh.advertise<ivpredict::ivmsgpredictdebug>("ivpredictdebug",1000);

    // predMonitor = new Monitor(20);
    timeObj     = ros::Time::now().toSec() * 1000;
    timeRoad    = ros::Time::now().toSec() * 1000;
    timeVsp     = ros::Time::now().toSec() * 1000;
    timeVap     = ros::Time::now().toSec() * 1000;
    timeApp     = ros::Time::now().toSec() * 1000;
    timeUser    = ros::Time::now().toSec() * 1000;
    timeTraffic = ros::Time::now().toSec() * 1000;
    timeLocpos  = ros::Time::now().toSec() * 1000;
    timeLoctag  = ros::Time::now().toSec() * 1000;
    InitParams();

    timeRecord.open("ivpredict_time.txt");
    time_init = ros::Time::now().toSec()*1000;
    timeRecord << setprecision(20) << time_init << endl;
}

PredictManager::~PredictManager()
{
    // delete predMonitor;
    // predMonitor = NULL;
timeRecord.close();
}

void PredictManager::InitParams()
{
    navigationPlanEnable = true;
    ros::NodeHandle mh;
    numPathSegs = 10;
    mh.param("numPathSegs",numPathSegs,numPathSegs);
    test_segid = 1001;
    mh.param("test_segid",test_segid,test_segid);
    test_enable = false;
    mh.param("test_enable",test_enable,test_enable);

    if (false == test_enable)
    {
        RefPath.LoadAllMaps(numPathSegs);
        RefPathRaw.LoadAllMaps(numPathSegs);       
    }
    else
    {
        RefPath.LoadAllMaps(test_segid);
        RefPathRaw.LoadAllMaps(test_segid);         
    }

    if (true == navigationPlanEnable)
    {
        navigationPlanEnable = false;
        segSequency.clear();
        segSequency.push_back(0);

        if (!segSequency.empty())
        {
            //std::cout<<"navigationPlanEnable is true !!!"<<std::endl;

            RefPath.PathSegsToSmoothPath(segSequency);
            RefPathRaw.PathSegsToSmoothPath(segSequency);               
        }
    }
    if (true == test_enable)
    {
        segSequency.clear();
        segSequency.push_back(test_segid - 1);   
        RefPath.PathSegsToSmoothPath(segSequency);
        RefPathRaw.PathSegsToSmoothPath(segSequency);     
    }

}

PredictManager::PredictPedestrian::PredictPedestrian()
{
}

PredictManager::PredictPedestrian::~PredictPedestrian()
{
}
ivpredict::predictobj PredictManager::PredictPedestrian::RelShort(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos locpos)
{
    ivpredict::predictobj predict_result;
    predict_result = PredictMotionBasedRel(pObj,locpos,FREQUENCY); 
    return predict_result;
}

int PredictManager::PredictPedestrian::oemga(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos locpos, ivpredict::predictobj* predict_result, ivpredict::predictobj* self_predict)
{
  EvaluateStates.UpdateState(pObj, self_predict, predict_result);
  // *predict_result = PredictOmega(pObj,locpos,FREQUENCY); 
}

ivpredict::predictobj PredictManager::PredictPedestrian::AbsShort(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap)
{
    ivpredict::predictobj predict_result;
    predict_result = PredictMotionBasedAbs(pObj,pIvlocpos,pIvvap,FREQUENCY); 
    //predict_result = PredictMotionBasedAbs_cui(pObj,pIvlocpos,pIvvap,FREQUENCY); 
    return predict_result;
}

//added lihui 0616
PredictManager::PredictCar::PredictCar()
{
}

PredictManager::PredictCar::~PredictCar()
{
}
ivpredict::predictobj PredictManager::PredictCar::AbsLong(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap)
{
    ivpredict::predictobj predict_result;
    predict_result = PredictMotionBasedAbs(pObj,pIvlocpos,pIvvap,FREQUENCY); 
    //predict_result = PredictMotionBasedAbs_cui(pObj,pIvlocpos,pIvvap,FREQUENCY); 
    return predict_result;
    //predMonitor->sendWarnning(2, 0); //Sending warnning to monitor topic 28,3,1
}

PredictManager::PredictUnknown::PredictUnknown()
{
}

PredictManager::PredictUnknown::~PredictUnknown()
{
}
ivpredict::predictobj PredictManager::PredictUnknown::run(ivpredict::objbuffer *pObj, ivmap::ivmapmsglocpos *pIvlocpos,ivmap::ivmapmsgvap *pIvvap)
{
    ivpredict::predictobj predict_result;
    predict_result = PredictMotionStatic(pObj, FREQUENCY); 
    return predict_result;
}

/**************************************************************
Function: OMEGA
Author: hui li / fang zhang
Date: 17/06/06
Description: 
1. choose the type of the object and PredictManager
2. finish the init(now as Pedestrian) PredictManager
3. finish the unknow PredictManager (put it at it is)
Input: void
Output: void
Return: void
Others: set the IAPredict_omega output
************************************************************/
int PredictManager::PredictOmega()
{
    //predictuserfun userfunction;
  
    int num_buffer = Buffer.obj_info.objects.size();   
    for (int i =0; i < num_buffer ;i++) //should be num_buffer
    {
      // printf("lihui debug==========obj=====================\n");
      if (Buffer.obj_info.objects[i].id != INVALID_OBJECT_ID)
      {
        switch (Buffer.obj_info.objects[i].property)
        {
            case 0:
            {
              PredictPedestrian Predict;
              ivpredict::predictobj Predict_result;
              int omega_finish = Predict.oemga(&Buffer.obj_info.objects[i],iamap.ivlocpos,&Predict_result, &car_pose);           
              IAPredict_omega.objects.push_back(Predict_result);
              break;
            }
            case 1:
            {
              break;
            }
            case 2:
            {
              break;
            }
            case 3:
            {
              break;
            }
            //unknow object
            case 4:
            {
              PredictUnknown Predict;
              ivpredict::predictobj Predict_result = Predict.run(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
              IAPredict_omega.objects.push_back(Predict_result);
              break;
            }
            case 5:
            {
              break;
            }

            default:
            {
	            PredictUnknown Predict;
              ivpredict::predictobj Predict_result = Predict.run(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
              IAPredict_omega.objects.push_back(Predict_result);
              cout << "ivpredict-omega: type is large than 5!" << endl;
              break;
            }
        }

      }
        
    }
    return 0;
}

/**************************************************************
Function: PredictAbsoluteShort
Author: hui li	zongwei
Date: 17/06/12	2017/6/12
Description: 
1. choose the type of the object and PredictManager using obsolute position
2. finish the init(now as Pedestrian) PredictManager using obsolute position
3. finish the unknow PredictManager (put it at it is) using obsolute position
Input: void
Output: void
Return: void
Others: set the IAPredict output
modified by --------------lihui------------0628
************************************************************/
void PredictManager::PredictAbsoluteShort()
{
    //predictuserfun userfunction;
  
    int num_buffer = Buffer.obj_info.objects.size();
    for (int i =0; i < num_buffer ;i++) //should be num_buffer
    {
      //printf("ivpredict-debug==========obj=========\n");
        if (Buffer.obj_info.objects[i].id != INVALID_OBJECT_ID)
        {
            switch (Buffer.obj_info.objects[i].property)
            {
                case 0:
                { 
                  PredictCar Predict;
		              //5s short paths for calculating each path's probability
                  ivpredict::predictobj predictResult = Predict.AbsLong(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
            		  IAPredict_shortForProbal.objects.push_back(predictResult);	
            		  //2s short paths
            		  // for(int k=predictResult.positions.size()-1;k>=20;k--)
            		  // {
            		  //   predictResult.positions.erase(predictResult.positions.begin()+k);
            		  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
                }
                case 1:
                {
                  PredictCar Predict;
            		  //5s short paths for calculating each path's probability
                              ivpredict::predictobj predictResult = Predict.AbsLong(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
            		  IAPredict_shortForProbal.objects.push_back(predictResult);	
            		  //2s short paths
            		  // for(int k=predictResult.positions.size()-1;k>=20;k--)
            		  // {
            		  //   predictResult.positions.erase(predictResult.positions.begin()+k);
            		  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
                }
                case 2:
                {
                  PredictPedestrian Predict;
		              //5s short paths for calculating each path's probability
                  ivpredict::predictobj predictResult = Predict.AbsShort(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
            		  IAPredict_shortForProbal.objects.push_back(predictResult);	
            		  //2s short paths
            		  // for(int k=predictResult.positions.size()-1;k>=20;k--)
            		  // {
            		  //   predictResult.positions.erase(predictResult.positions.begin()+k);
            		  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
                }
                case 3:
                {
                  PredictPedestrian Predict;
		              //5s short paths for calculating each path's probability
                  ivpredict::predictobj predictResult = Predict.AbsShort(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
            		  IAPredict_shortForProbal.objects.push_back(predictResult);	
            		  //2s short paths
            		  // for(int k=predictResult.positions.size()-1;k>=20;k--)
            		  // {
            		  //   predictResult.positions.erase(predictResult.positions.begin()+k);
            		  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
                }
                //unknow object
                case 4:
                {
                  PredictUnknown Predict;
		              //5s short paths for calculating each path's probability
                  ivpredict::predictobj predictResult = Predict.run(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
            		  IAPredict_shortForProbal.objects.push_back(predictResult);	
            		  //2s short paths
            		  // for(int k=predictResult.positions.size()-1;k>=20;k--)
            		  // {
            		  //   predictResult.positions.erase(predictResult.positions.begin()+k);
            		  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
                }
                case 5:
                {
                  PredictPedestrian Predict;
		              //5s short paths for calculating each path's probability
                  ivpredict::predictobj predictResult = Predict.AbsShort(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
            		  IAPredict_shortForProbal.objects.push_back(predictResult);	
            		  //2s short paths
            		  //for(int k=predictResult.positions.size()-1;k>=20;k--)(change by lnh)
            		  // for(int k=predictResult.positions.size()-1;k>=10;k--)
            		  // {
            		  //   predictResult.positions.erase(predictResult.positions.begin()+k);
            		  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
                }
                default:
                {
                  PredictUnknown Predict;
                  //5s short paths for calculating each path's probability
                  ivpredict::predictobj predictResult = Predict.run(&Buffer.obj_info.objects[i],&iamap.ivlocpos,&iamap.ivvap);
                  IAPredict_shortForProbal.objects.push_back(predictResult);  
                  //2s short paths
                  // for(int k=predictResult.positions.size()-1;k>=20;k--)
                  // {
                  //   predictResult.positions.erase(predictResult.positions.begin()+k);
                  // }
                  IAPredict_shortAbs.objects.push_back(predictResult);
                  break;
            		  cout<<"exit no object is short term path"<<endl;
            		  break;
                }
            }
        }
    }
    return;
}
/**************************************************************
Function: YUYAN
Author: hui li
Date: 2017/08/23
Description: 
1. choose the type of the object and PredictManager using realposition
2. get the reference line in realposition
3. PredictManager short

Input: void
Output: void
Return: void
Others: set the IAPredict output
************************************************************/
void PredictManager::PredictYuyan()
{
    //predictuserfun userfunction;
    int num_buffer = Buffer.obj_info.objects.size();
    for (int i =0; i < num_buffer ;i++) //should be num_buffer
    {
        if (Buffer.obj_info.objects[i].id != INVALID_OBJECT_ID)
        {
          ivpredict::predictobj Predict_result = PredictAbsLineRelfunc(&Buffer.obj_info.objects[i],FREQUENCY,&Reference_path, iamap.ivlocpos, iamap.ivvap);
          IAPredict_LineRel.objects.push_back(Predict_result);
        }
    }
    return;
}

/**************************************************************
Function: PredictUserFunc
Author: hui li
Date: 17/06/06
Description: 
1. call the user function and push the PredictManager result

Input: void
Output: void
Return: void
Others: push back the object but may have same ID number
************************************************************/
void PredictManager::PredictUserFunc()
{
  //predictuserfun userfunction;
  int num_userfun = iamap.ivuserfun.obj.size();   

  for (int i =0; i < num_userfun ;i++) //should be num_buffer
  {
    switch (iamap.ivuserfun.obj[i].type)
    {
      case 6:
      {
        ivpredict::predictobj Predict_result = PredictUserFuncRel(iamap.ivuserfun.obj[i], FREQUENCY);
        IAPredict_omega.objects.push_back(Predict_result);
        break;
      }
      case 7:
      {
        ivpredict::predictobj Predict_result = PredictUserFuncGPS(iamap.ivuserfun.obj[i],iamap.ivlocpos, FREQUENCY);
        IAPredict_omega.objects.push_back(Predict_result);
        break;
      }            
      default:
      {
        break;
      }
    }
  }
  return;
}

/**************************************************************
Function: run
Author: hui li
Date: 17/06/06
Description: 
The main function of PredictManager
1. Map load

2. Buffer(container store) & check
4.

Input: void
Output: void
Return: void
Others: push back the object but may have same ID number
************************************************************/

void PredictManager::run()
{
  ros::Rate rate(FREQUENCY);

  //manage the map
  string cattype = "car", humantype = "human";
  // gLoadedMap = LoadMap(cattype);
  printf("------------ lihui -----------begin\n");
  while(ros::ok())
  {
    printf(" lihui ivpredict---------------time-------------start\n");
    RefPathRaw.LocalRefpathBuild(&car_pose, &iamap.ivlocpos);
    //debug check
    if (gDebug) 
    {
      Check.Inputcheck(&iamap.ivobj);
    }
    ros::spinOnce();
    double startTime=ros::Time::now().toSec() * 1000;
    MonitorCheck(startTime);
    //buffer check
    int buffer_success = Buffer.Update(iamap);
    if (gDebug)
    {     
      cout << "ivpredict-debug: lidar methods: " << predict_lidar << endl;
    }
    int buffercheck = Check.BuffermsgCheck(&Buffer.obj_info);
    
    if ((buffer_success == 0 && buffercheck == 0))
    {
      cout << "ivpredict-omega: buffer OK! " << endl;
    }
    
    //clear all objects
    IAPredict_LineRel.objects.clear();
    IAPredict_omega.objects.clear();
    IAPredict_shortAbs.objects.clear();
    IAPredict_shortForProbal.objects.clear();
    IAPredict_shortLong.objects.clear();

    //car type method check
    int outputcheck = 0;
    ros::NodeHandle nh;
    std::string avos("lsav");
    nh.param("avos", avos, avos);

    if ("lsav" == avos)
    {
      predictmethod = eOMEGA;
    }
    else if ("hsav" == avos)
    {
      predictmethod = eYUYAN;
    }
    predictmethod = eAbs;
    //choose the predict method
    switch(predictmethod)
    {
    	case 0:
    	{
    	  PredictOmega();
        // PredictAbsoluteShort();
       //  if (gDebug)
       //  {         
       //    cout << "ivpredict-debug: rel-short obj output: "  << IAPredict_omega.objects.size() << endl;
       //  }
        // IAPredict_omega.objects.clear();
        // IAPredict_omega.objects.push_back(car_pose);
        // cout << " lihui car size: " << car_pose.positions.size() << endl;
        outputcheck = Check.PredictmsgCheck(&IAPredict_omega);
    	  pub1_.publish(IAPredict_omega);
        cout << "ivpredict-omega: predictmethod rel-short!" << endl;
    	  break;
    	}
    	case 1:
    	{
        cout << "ivpredict-yuyan: pathplanner size = " << Reference_path.points.size() << endl;
        if (Reference_path.points.size() == 0)
        {
          continue;
          pub1_.publish(IAPredict_omega);
        }
        else
        {
          PredictYuyan();
          pub1_.publish(IAPredict_LineRel);
        }       
        cout << "ivpredict-yuyan: predictmethod relpredict-absline-long!" << endl;
    	  break;
    	}
    	case 2:
    	{
    	  PredictAbsoluteShort();
    	  geo_function.PredictGCC2VCS(&IAPredict_shortForProbal,&iamap.ivlocpos);
        if (gDebug)
        {
          cout << "ivpredict-debug: abs-short obj output: "  << IAPredict_omega.objects.size() << endl;
        }
        outputcheck = Check.PredictmsgCheck(&IAPredict_shortForProbal);        
    	  pub1_.publish(IAPredict_shortForProbal);
        cout << "ivpredict-omega: predictmethod abs-short" << endl;
    	  break;
    	}
    	default:
      {
        cout << "ivpredict-omega: predictmethod too long!" << endl;        
      }         
    }

    time_pub = ros::Time::now().toSec() * 1000.0 - time_init;
    timeRecord << time_pub << "\t";
    for(int i=0;i<IAPredict_shortForProbal.objects.size();++i)
    {
	timeRecord<<(int)IAPredict_shortForProbal.objects[i].id <<"\t";
    }
    timeRecord<<endl;

    //output check
    if (gDebug)
    {
      if (outputcheck == 0)
      {
        cout << "ivpredict-omega: output context OK! " << endl;
      }
    }
    printf("ivpredict------------------------------time--------------------------over\n");
    rate.sleep();
    Reference_path.points.clear();
    iamap.ivvap.v = INVALID_SPEED_ID;
    //monitor
    time2 = time1;
    time1 = ros::Time::now().toSec() * 1000;
    // checkWarning(time1 - time2);
    //monitor
  }
}

void PredictManager::chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg)
{
    iamap.ivobj = *msg;
    timeObj = ros::Time::now().toSec() * 1000;
}

void PredictManager::chatterCallbackRoad(const ivmap::ivmapmsgroad::ConstPtr &msg)
{
    iamap.ivroad = *msg;
    timeRoad = ros::Time::now().toSec() * 1000;
}

void PredictManager::chatterCallbackVsp(const ivmap::ivmapmsgvsp::ConstPtr &msg)
{
    timeVsp = ros::Time::now().toSec() * 1000;
}

void PredictManager::chatterCallbackVap(const ivmap::ivmapmsgvap::ConstPtr &msg)
{
    iamap.ivvap = *msg;
    gEgoCarVg=fabs(iamap.ivvap.v);
    timeVap = ros::Time::now().toSec() * 1000;

}

void PredictManager::chatterCallbackApp(const ivmap::ivmapmsgapp::ConstPtr &msg)
{  
    timeApp = ros::Time::now().toSec() * 1000;
}

void PredictManager::chatterCallbackUser(const ivmap::ivmapmsguserfun::ConstPtr &msg)
{
    iamap.ivuserfun = *msg;
    timeUser = ros::Time::now().toSec() * 1000;   
}

void PredictManager::chatterCallbackTraffic(const ivmap::ivmapmsgtraffic::ConstPtr &msg)
{
    timeTraffic = ros::Time::now().toSec() * 1000;
}

void PredictManager::chatterCallbackLocpos(const ivmap::ivmapmsglocpos::ConstPtr &msg)
{    
    iamap.ivlocpos = *msg;
    gEgoCarPosXg=iamap.ivlocpos.xg;
    gEgoCarPosYg=iamap.ivlocpos.yg;   
    timeLocpos = ros::Time::now().toSec() * 1000;
}

void PredictManager::chatterCallbackLoctag(const ivmap::ivmapmsgloctag::ConstPtr &msg)
{
    timeLoctag = ros::Time::now().toSec() * 1000;
}

void PredictManager::MonitorCheck(double TimeNow)
{
  // checkWarningObj();
  // checkWarningRoad();
  // checkWarningVsp();
  mainMonitor.checkWarningVap(TimeNow, timeVap);
  // checkWarningApp();
  // checkWarningUser();
  //scheckWarningTraffic();
  // checkWarningLocpos();
 // checkWarningLoctag();
}

void PredictManager::subCallback_motionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg)
{
    // sPointMsgtest pointMsg;
    Reference_path = *msg;
    float disx,disy,dis,dismin = 9999;
    float nearx, neary;
    for (int i = 0; i < iamap.ivobj.obj.size(); ++i)
    {
      dismin = 9999;
      for(long j = 0; j < msg->points.size(); j = j + 1)
      {
        disx = abs(iamap.ivobj.obj[i].x - msg->points[j].x);
        disy = abs(iamap.ivobj.obj[i].y - msg->points[j].y);
        dis = sqrt(disx * disx + disy * disy);
          // pointMsg.length += calc_distance(pointMsg_last, pointMsg);
        if (dis < dismin)
        {
          dismin = dis;
          nearx = msg->points[j].x;
          neary = msg->points[j].y;
        }
      }
      // cout << "lihui 2 objID: " << (int)iamap.ivobj.obj[i].id << " dis-to-route: " << dismin << " car angle: " << iamap.ivlocpos.angle << endl;
      // cout << "lihui rel-dis-to-car: " << sqrt(iamap.ivobj.obj[i].x * iamap.ivobj.obj[i].x + iamap.ivobj.obj[i].y * iamap.ivobj.obj[i].y) << endl;
      // cout << "lihui nearpoint: x:" << nearx << " y: " << neary << endl;
      // printf("--------------------------------------------------\n");
    }

}
