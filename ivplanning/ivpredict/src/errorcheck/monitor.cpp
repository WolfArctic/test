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
#include "monitor.h"

monitor::monitor(){
	predictMonitor = new Monitor(20);
}
monitor::~monitor(){
	delete predictMonitor;
    predictMonitor = NULL;
}

void monitor::checkWarningVap(double runTime,double CallbackTime)
{
  double dif_time = runTime - CallbackTime;

  if(dif_time >= (periodeN + later_Time1) && dif_time < (periodeN + later_Time2))
    predictMonitor->sendWarnning(3, 0);
  else if(dif_time >= (periodeN + later_Time2) && dif_time < (periodeN + later_Time4))
    predictMonitor->sendWarnning(3, 1);
  else if(dif_time >= (periodeN + later_Time4) && dif_time < (periodeN + later_Time5))
    predictMonitor->sendWarnning(3, 2);
  else if(dif_time >= (periodeN + later_Time5) && dif_time < (periodeN + later_Time6))
    predictMonitor->sendWarnning(3, 3); 
  else if(dif_time >= (periodeN + later_Time6))
  	predictMonitor->sendWarnning(2, 3); 
  
  // if(((dif_time - periodeN) / periodeN) * 100 >= later_Time1 && ((dif_time - periodeN) / periodeN) * 100 < later_Time3)
  //   predictMonitor->sendWarnning(4, 0);   
  // else if(((dif_time - periodeN) / periodeN) * 100 >= later_Time3 && ((dif_time - periodeN) / periodeN) * 100 < later_Time4)
  //   predictMonitor->sendWarnning(4, 1);
  // else if(((dif_time - periodeN) / periodeN) * 100 >= later_Time4 && ((dif_time - periodeN) / periodeN) * 100 < later_Time5)
  //   predictMonitor->sendWarnning(4, 2);
  // else if(((dif_time - periodeN) / periodeN) * 100 >= later_Time5 && ((dif_time - periodeN) / periodeN) * 100 < later_Time6)
  //   predictMonitor->sendWarnning(4, 3);
  // else if(((dif_time - periodeN) / periodeN) * 100 >= later_Time6)
  //   predictMonitor->sendWarnning(2, 3);
}
