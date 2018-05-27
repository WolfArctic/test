//CMakeList.txt
/* add_message_files(
 *	...
 *	carstateshow.msg
 *)
 */
/////////////////////////////////////////////////////////////////////////////////
//include headfile
#include "iv***/carstateshow.h"
/////////////////////////////////////////////////////////////////////////////////

//define pub_msg
ros::Publisher pub_carstateshow;
/////////////////////////////////////////////////////////////////////////////////

//define execute function
int sendCarStateShowToIVACTUTER(const char *cmdString,bool showCmd,int ledcmd);
/////////////////////////////////////////////////////////////////////////////////

//declare msg
pub_carstateshow =  nh.advertise<ivapp::carstateshow>("carstateshow",10);
/////////////////////////////////////////////////////////////////////////////////
/* Func     :sendCarStateShowToIVACTUTER
 * Author   :suhaotian@idriverplus.com
 * Data     :17/12/05
 * Info     :for horn of car and some others.
*/
int app::sendCarStateShowToIVACTUTER(const char *cmdString,bool showCmd,int ledcmd)
{
    const char *cmdArray[9] = {"restart","TurnLight_Left","TurnLight_Right","HeadLight","BrakeLight","ReversingLight"
                            ,"DoubleFlashLight","TailLight","Horn"};
    static bool cmdValue[9] = {0,0,0,0,0,0,0,0,0};
    static int leddisplayvalue = 0;
    if(!strcmp(cmdString,"ledcmd"))
    {    
        leddisplayvalue = ledCmd;
        return 0;
    }
    for(int i=0;i<9;i++)
    {
        if(!strcmp(cmdString,cmdArray[i]))
        {    
            cmdValue[i] = showCmd;
            break;
        }
    }
    if(cmdValue[0] == true)
        for(int i=0;i<9;i++)
            cmdValue[i] = 0;
    if(!strcmp(cmdString,"send") && showCmd == true)
    {
        ivapp::carstateshow pubcarshowstatemsg;
     //   pubcarshowstatemsg.openLockcmd = cmdValue[0];//turn left:1.....tern right:2....
        pubcarshowstatemsg.turnlightleft = cmdValue[1];
        pubcarshowstatemsg.turnlightright = cmdValue[2];
        pubcarshowstatemsg.headlight = cmdValue[3];
        pubcarshowstatemsg.brakelight = cmdValue[4];
        pubcarshowstatemsg.reversinglight = cmdValue[5];
        pubcarshowstatemsg.doubleflashlight = cmdValue[6];
        pubcarshowstatemsg.taillight = cmdValue[7];
        pubcarshowstatemsg.horn = cmdValue[8];
        pubcarshowstatemsg.leddisplaycmd = leddisplayvalue;
        pub_carstateshow.publish(pubcarshowstatemsg);
    }
    return 0;
}
/////////////////////////////////////////////////////////////////////////////////
//usage
    const char *cmdArray[9] = {"restart","TurnLight_Left","TurnLight_Right","HeadLight","BrakeLight","ReversingLight"
                            ,"DoubleFlashLight","TailLight","Horn"};
    sendCarStateShowToIVACTUTER(cmdArray[1],true,0);
    sendCarStateShowToIVACTUTER(cmdArray[2],true,0);
    sendCarStateShowToIVACTUTER("ledcmd",true,1);//1左转，2右转，3取货中，4配送中,请让行，5左换道，6右换道，7智行者,蜗必达
	//....
    sendCarStateShowToIVACTUTER("send",true,0);
/////////////////////////////////////////////////////////////////////////////////
//PS:"***",like ivapp,ivcloud....
