/*
 * Copyright (c) 2015-2020 idriverplus(Beijing ZhiXingZhe Inc.)
 * website: www.idriverplus.com
 * Distributed under the IVPT Software License
 * Author: zhangbaofeng
 * This node is used to read serial data and publish the data content, and subscription data content into a serial port.
 * * *************************************************************************
 * */
#include "actuator.h"

actuator::actuator(ros::NodeHandle handle)
{
        m_handle = handle;
     // handle.param("odom_data_read_mode",can_serial_data_flag,can_serial_data_flag);
     // if("can" == can_serial_data_flag)
     // {
        memset(&t_lightHornControl[0],0,sizeof(t_lightHornControl));
        sockaddr_can addr;
        ifreq ifr;
        socketcan_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(socketcan_fd < 0)
        {
            perror("ivactuator--> socket:");
            exit(-1);
        }
        strcpy(ifr.ifr_name, "can1" );
        ioctl(socketcan_fd, SIOCGIFINDEX, &ifr);
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if( bind(socketcan_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        {
            perror("ivactuator--> bind:");
            exit(-1);
	    }
        struct can_filter rfilter[4];
        rfilter[0].can_id = AVCU_XEP_86;
        rfilter[0].can_mask = CAN_SFF_MASK;
        rfilter[1].can_id = BCM_201;
        rfilter[1].can_mask = CAN_SFF_MASK;
        rfilter[2].can_id = BCM_202;
        rfilter[2].can_mask = CAN_SFF_MASK;
        rfilter[3].can_id = EPS_401;
        rfilter[3].can_mask = CAN_SFF_MASK;
        setsockopt(socketcan_fd,SOL_CAN_RAW,CAN_RAW_FILTER,&rfilter,sizeof(rfilter));
    // }

    // else
    // {
    //     m_baudrate = 9600;
    //     m_serialport = "/dev/ttyUSB0";
    //     steerTimeFlag = false;
    //     driverTimeFlag = false;
    //     mcuCount = 0;
    //     steerControlDataCount = 0;
    //     driverControlDataCount = 0;
    //     receiverCurrentByteCount = 0;
    //     memset(tempDataArray,0,sizeof(tempDataArray));
    //     memset(&t_driverControlMsg,0,sizeof(t_driverControlMsg));
    //     memset(&t_steerControlMsg,0,sizeof(t_steerControlMsg));
    //     mcuMonitor = new Monitor(24);
    //     underMcuMonitor = new Monitor(29);
    //     //paramters
    //     handle.param("mcubaudrate",m_baudrate,m_baudrate);
    //     handle.param("mcuserialport",m_serialport,m_serialport);
    //     handle.param("mcudevice",m_deviceName,m_deviceName);
    //     std::cout<<FRED("Copyright©2016-2020 idriverplus. All rights reserved ")<<std::endl;
    //     std::cout<<FYEL("*****ivactuator:parameters*******************")<<std::endl;
    //     std::cout<<FGRN("mcu_baudrate: ")<<m_baudrate<<std::endl;
    //     std::cout<<FGRN("mcu_serialport: ")<<m_serialport<<std::endl;
    //     std::cout<<FGRN("mcu_device: ")<<m_deviceName<<std::endl;
    //     std::cout<<FYEL("*****ivactuator:parameters end***************")<<std::endl; 
    //     //set and open serial
    //     try
    //     {
    //       	std::cout<<FYEL("[ivactuator-->]")<<FGRN("Serial initialize start!")<<std::endl;
    //       	ser.setPort(m_serialport.c_str());// virtual com address  /dev/pts/3
    //       	ser.setBaudrate(m_baudrate);
    //       	serial::Timeout to = serial::Timeout::simpleTimeout(500);
    //       	ser.setTimeout(to);
    //       	ser.open();
    //     }
    //     catch(serial::IOException& e)
    //     {
    //         sendWarnnigToMonitor(PERIPHERAL_FAULT,0);
    //         std::cout<<FYEL("[ivactuator-->]")<<FRED("Unable to open port!")<<std::endl;
    //     }
    //     if(ser.isOpen())
    //     {
    //         std::cout<<FYEL("[ivactuator-->]")<<FGRN("Serial initialize successfully!")<<std::endl;
    //     }
    //     else
    //     {
    //         sendWarnnigToMonitor(PERIPHERAL_FAULT,0);
    //         std::cout<<FYEL("[ivactuator-->]")<<FRED("Serial port failed!")<<std::endl;
    //     } 
    // }
}

void actuator::run()
{
    // if("can" == can_serial_data_flag)
    // {
        sub_steer   = m_handle.subscribe("ivsteercontrol",1000,&actuator::callback_ivsteercontrol,this);
        sub_driver  = m_handle.subscribe("ivdrivercontrol",1000,&actuator::callback_ivdrivercontrol,this);
        sub_carstateshow  = m_handle.subscribe("carstateshow",1000,&actuator::callback_carstateshow,this);
        sub_appopenbox = m_handle.subscribe("ivappopenbox",1000,&actuator::callback_ivappopenbox,this);
	    sub_monitofaultreaction = m_handle.subscribe("monitofaultreaction",1000,&actuator::callback_monitofaultreaction,this);
        pub_control = m_handle.advertise<ivactuator::ivactuator>("ivactuator",1000);
        pub_wheelspeed = m_handle.advertise<ivactuator::wheelspeed>("wheelspeed",1000);
        pub_lockstate = m_handle.advertise<ivactuator::lockstate>("lockstate",1000);
        boost::thread thread(boost::bind(&actuator::callback_sendthread, this) );  
        thread.detach();
        int run_rate = 200; //5ms
        ros::Rate rate(run_rate);
        while(ros::ok())
        {
            ros::spinOnce();
            recvCarInfoKernel();   // read message from serial and pub message to contro
            rate.sleep();
        }
    // }
    // else
    // {
    //     std::cout<<FRED("serialport mode is not suit for omega")<<std::endl;
    //     std::cout<<FRED("please choose can_serial_data_flag == can")<<std::endl;
    //     sub_steer   = m_handle.subscribe("ivsteercontrol",1000,&actuator::callback_serialivsteercontrol,this);
    //     sub_driver  = m_handle.subscribe("ivdrivercontrol",1000,&actuator::callback_serialivdrivercontrol,this);
    //     sub_appremote  = m_handle.subscribe("remotecontrol",1000,&actuator::callback_serialivapp,this);
    //     pub_control = m_handle.advertise<ivactuator::ivactuator>("ivactuator",1000);
    //     boost::thread thread(boost::bind(&actuator::callback_serialsendthread, this) );  
    //     thread.detach();
    //     int run_rate = 20; //50ms
    //     bool timeFlag = false;
    //     ros::Time timeTemp;
    //     ros::Time timeTempOld;
    //     ros::Rate rate(run_rate);
    //     while(ros::ok())
    //     {
    //         timeTemp = ros::Time::now();
    //         if(timeFlag)
    //         {
    //             float absoluteCycle = timeTemp.toSec() - timeTempOld.toSec();
    //             float relativeCycle = (absoluteCycle-0.05)/0.05;
    //             monitorNodeAbsoluteCycle(absoluteCycle,THIS_NODE_ABSOLUTE_CYCLE,0.05,0);
    //             monitorNodeRelativeCycle(relativeCycle,THIS_NODE_RELATIVE_CYCLE,0);
    //         }
    //         timeTempOld = timeTemp;
    //         timeFlag = true;
    //         ros::spinOnce();
    //         serialRecvCarInfoKernel();   // read message from serial and pub message to contro
    //         rate.sleep();
    //     }
    // }
}

/**
 *callback_sendthread()
 *detail:therd send message
 */
void actuator::callback_sendthread()
{
    bool hornFlag = false;
    while(1){
        for(int i=0; i<sizeof(enum lightHornName); i++){
            if(i == HORN){
                if(t_lightHornControl[HORN].status)
                    hornFlag = true;
                if(hornFlag){
                    t_lightHornControl[HORN].counter += 20;
                    if(1 == (t_lightHornControl[HORN].counter/1000)%2){
                        t_lightHornControl[HORN].status = false;
                    }else{
                        t_lightHornControl[HORN].status = true;
                    }
                    if(t_lightHornControl[HORN].counter >= t_lightHornControl[HORN].delaytime){
                        t_lightHornControl[HORN].status = false;
                        hornFlag = false;
                    }
                }
            }else if(t_lightHornControl[i].status){
                t_lightHornControl[i].counter += 20;
                if(t_lightHornControl[i].counter >= t_lightHornControl[i].delaytime){
                     t_lightHornControl[i].status = false;
                }
            }
        }
        sendCarInfoKernel();//send message 
        usleep(10*1000); //10ms
    }
}

void actuator::callback_ivsteercontrol(const ivsteercontrol::ivsteercontrol::ConstPtr &msg)
{
    t_steerControlMsg.msg     = *msg;
    t_steerControlMsg.isvalid = true;
}

void actuator::callback_ivdrivercontrol(const ivdrivercontrol::ivdrivercontrol::ConstPtr &msg)
{
    t_driverControlMsg.msg     = *msg;
    t_driverControlMsg.isvalid = true;
}

void actuator::callback_carstateshow(const ivpathplanner::carstateshow::ConstPtr &msg)
{
    t_carstateshowMsg.msg = *msg;
    t_carstateshowMsg.isvalid = true;
    if(t_carstateshowMsg.msg.turnlightleft){
        t_lightHornControl[TURNLIGHTLEFT].status    = true;
        t_lightHornControl[TURNLIGHTLEFT].counter   = 0;
        t_lightHornControl[TURNLIGHTLEFT].delaytime = LIGHT_HORN_DELAY_TIME;
    }
    if(t_carstateshowMsg.msg.turnlightright){
        t_lightHornControl[TURNLIGHTRIGHT].status    = true;
        t_lightHornControl[TURNLIGHTRIGHT].counter   = 0;
        t_lightHornControl[TURNLIGHTRIGHT].delaytime = LIGHT_HORN_DELAY_TIME;
    }
    if(t_carstateshowMsg.msg.brakelight){
        t_lightHornControl[BRAKELIGHT].status    = true;
        t_lightHornControl[BRAKELIGHT].counter   = 0;
        t_lightHornControl[BRAKELIGHT].delaytime = LIGHT_HORN_DELAY_TIME;
    }
    if(t_carstateshowMsg.msg.horn){
        t_lightHornControl[HORN].status    = true;
        t_lightHornControl[HORN].counter   = 0;
        t_lightHornControl[HORN].delaytime = LIGHT_HORN_DELAY_TIME;
    }	
}

void actuator::callback_ivappopenbox(const ivapp::ivappopenbox::ConstPtr &msg)
{
    t_ivappopenboxMsg.msg = *msg;
    t_ivappopenboxMsg.isvalid = true;
}

void actuator::callback_monitofaultreaction(const monitor_msgs::monitofaultreaction::ConstPtr &msg)
{
    t_monitofaultreactionMsg.msg = *msg;
    t_monitofaultreactionMsg.isvalid = true;
}

/**
 *RecvCarInfoKernel()
 *detail:Read serial data, publish after parsing
 */
void actuator::recvCarInfoKernel()
{
    int nbytes = 0;
    unsigned int tmp = 0;
    can_frame frame;
    memset(&frame,0,sizeof(frame));
    nbytes = read(socketcan_fd, &frame, sizeof(frame));
    if (nbytes < 0){
        perror("ivactuator--> can raw socket read");
    }
    /* paranoid check ... */
    if (nbytes < sizeof(struct can_frame)) {
        fprintf(stderr, "ivactuator--> read: incomplete CAN frame\n");
    }
    /*the content can be handled independently from the received MTU size*/
    /*printf("can_id: %X data length: %d \n", frame.can_id, frame.can_dlc);*/
    if(AVCU_XEP_86 == frame.can_id){
        ivactuatorMsg.shiftlvlposition  = frame.data[0] & 0x3;
        ivactuatorMsg.sysstatus  		= (frame.data[0]>>2) & 0x7;
        ivactuatorMsg.uispeed           = (float)((frame.data[1]<<8)|frame.data[2])/10.0/3.6;//m/s 
        ivactuatorMsg.batteryvoltage    = (unsigned short)((frame.data[3]<<8)|frame.data[4]);
        ivactuatorMsg.reserve2          = (float)((frame.data[5]<<8)|(frame.data[6]));
    }else if(BCM_201 == frame.can_id){
        tmp = (unsigned int)(frame.data[0]<<24 | frame.data[1]<<16 | frame.data[2]<<8 | frame.data[3]);
        wheelspeedMsg.wheelspeed_lr = (float)(tmp/10000.0-30.0);
        tmp = (unsigned int)(frame.data[4]<<24 | frame.data[5]<<16 | frame.data[6]<<8 | frame.data[7]);
        wheelspeedMsg.wheelspeed_rr = (float)(tmp/10000.0-30.0);
    }else if(BCM_202 == frame.can_id){
        lockstateMsg.lockstate = (unsigned int)(frame.data[0]<<16 | frame.data[1]<<8 | frame.data[2]);
    }else if(EPS_401 == frame.can_id){
        ivactuatorMsg.realstrtorque     = (unsigned short)(frame.data[1]);
        ivactuatorMsg.uisteerangle      = (short)((frame.data[3]<<8)|frame.data[4])-1024;		
    }
    pub_control.publish(ivactuatorMsg);
    pub_wheelspeed.publish(wheelspeedMsg);
    pub_lockstate.publish(lockstateMsg);
}

/**
 *SendCarInfoKernel()
 *detail:Write data to serial port
 */
void actuator::sendCarInfoKernel()
{
    can_frame frame;
    int nbytes = 0;
    frame.can_id = AVCU_TX2_F5;
    frame.can_dlc = 0x2;
    // frame.data[0] = t_ivappopenboxMsg.msg.openboxnum & 0x1F;
    // frame.data[0] |= t_lightHornControl[TURNLIGHTLEFT].status ? (0x01<<5) : 0x00;
    // frame.data[0] |= t_lightHornControl[TURNLIGHTRIGHT].status ? (0x01<<6) : 0x00;
    // frame.data[0] |= t_carstateshowMsg.msg.headlight ? (0x01<<7) : 0x00;
    // frame.data[1] = t_lightHornControl[BRAKELIGHT].status ? 0x01 : 0x00;
    // frame.data[1] |= t_carstateshowMsg.msg.reversinglight ? (0x01<<1) : 0x00;
    // frame.data[1] |= t_monitofaultreactionMsg.msg.doubleflash ? (0x01<<2) : 0x00;
    // frame.data[1] |= t_carstateshowMsg.msg.taillight ? (0x01<<3) : 0x00;
    // frame.data[1] |= t_lightHornControl[HORN].status ? (0x01<<4) : 0x00;
    nbytes = write(socketcan_fd, &frame, sizeof(frame));
    usleep(10*1000);
    frame.can_id = AVCU_TX2_85;
    frame.can_dlc = 0x8;
    frame.data[0] = (unsigned short)(t_steerControlMsg.msg.targetangle + 1024) / 256;
    frame.data[1] = (unsigned short)(t_steerControlMsg.msg.targetangle + 1024) % 256;
    frame.data[2] = (unsigned short)t_steerControlMsg.msg.torque / 256;
    frame.data[3] = (unsigned short)t_steerControlMsg.msg.torque % 256;
    frame.data[4] = (unsigned short)t_driverControlMsg.msg.targettorque / 256;
    frame.data[5] = (unsigned short)t_driverControlMsg.msg.targettorque % 256;
	if(t_monitofaultreactionMsg.msg.breakcmd){
		t_driverControlMsg.msg.actuatormode = 0x2;
	}
    frame.data[6] = t_driverControlMsg.msg.actuatormode | (t_driverControlMsg.msg.shiftposition << 2);
    // frame.data[7] = 0xF & t_carstateshowMsg.msg.leddisplaycmd;
    nbytes = write(socketcan_fd, &frame, sizeof(frame));
}


/**
 *callback_sendthread()
 *detail:therd send message to serial
 */
//hide on 0521 zt
// void actuator::callback_serialsendthread()
// {
//     while(1){
//         serialSendCarInfoKernel();//send message to serial
//         usleep(50*1000); //50ms
//     }
// }

/**
 *callback_ivsteercontrol()
 *detail:ivsteercontrol Msg callback function
 */
//hide on 0521 zt
// void actuator::callback_serialivsteercontrol(const ivsteercontrol::ivsteercontrol::ConstPtr &msg)
// {
//     ros::Time now =  ros::Time::now();
//     if(steerTimeFlag){
//         float absoluteCycle = steerTimeTemp.toSec() - now.toSec();
//         float relativeCycle = (absoluteCycle-0.025)/0.025;
//         monitorNodeAbsoluteCycle(absoluteCycle,FRONT_NODE_ABSOLUTE_CYCLE,0.025,4);
//         monitorNodeRelativeCycle(relativeCycle,FRONT_NODE_RELATIVE_CYCLE,4);
//     }else{
//         steerTimeFlag = true;
//     }
//     steerTimeTemp = now;
//     t_steerControlMsg.msg     = *msg;
//     t_steerControlMsg.isvalid = true;
// }

/**
 *callback_ivdrivercontrol()
 *detail:ivdrivercontrol Msg callback function
 */
//hide on 0521 zt
// void actuator::callback_serialivdrivercontrol(const ivdrivercontrol::ivdrivercontrol::ConstPtr &msg)
// {
//     ros::Time now =  ros::Time::now();
//     if(driverTimeFlag){
//         float absoluteCycle = driverTimeTemp.toSec() - now.toSec();
//         float relativeCycle = (absoluteCycle-0.025)/0.025;
//         monitorNodeAbsoluteCycle(absoluteCycle,FRONT_NODE_ABSOLUTE_CYCLE,0.05,0);
//         monitorNodeRelativeCycle(relativeCycle,FRONT_NODE_RELATIVE_CYCLE,0);
//     }else{
//         driverTimeFlag = true;
//     }
//     driverTimeTemp = now;
//     t_driverControlMsg.msg     = *msg;
//     t_driverControlMsg.isvalid = true;
// }

/**
 *subCallback_ivapp
 *author:zhangbaofeng
 *date:2016.12.02
 *detail:callback function
 */
//hide on 0521 zt
// void actuator::callback_serialivapp(const ivapp::remotecontrol::ConstPtr &msg)
// {
//     t_remoteControlMsg.msg     = *msg;
//     t_remoteControlMsg.isvalid = true;
//     if(0x11 == t_remoteControlMsg.msg.data_type){
//         t_remoteControlInfo.steerAngle = t_remoteControlMsg.msg.data_i32;
//     }else if(0x12 == t_remoteControlMsg.msg.data_type){
//         t_remoteControlInfo.targettorque = t_remoteControlMsg.msg.data_f32*10;
//         if(t_remoteControlInfo.targettorque <= 0){
//             t_remoteControlInfo.actuatormode = 0;
//             t_remoteControlInfo.targetacc = 5;
//         }
//         else{
//             t_remoteControlInfo.actuatormode = 1;
//             t_remoteControlInfo.targetacc = 0;
//         }
//     }else if(0x13 == t_remoteControlMsg.msg.data_type){
//         if(t_remoteControlMsg.msg.data_i32 < 0x4){
//             t_remoteControlInfo.actuatormode = 1;
//             t_remoteControlInfo.targetshiftposition = t_remoteControlMsg.msg.data_i32;
//             t_remoteControlInfo.targetacc = 0;
//         }
//         else{
//             t_remoteControlInfo.actuatormode = 0;
//             t_remoteControlInfo.targetacc = 15;
//         }
//     }else if(0x14 == t_remoteControlMsg.msg.data_type){
//         t_remoteControlInfo.start = (0==t_remoteControlMsg.msg.data_i32)?true:false;
//     }
//     if(!t_remoteControlInfo.start){
//         memset(&t_remoteControlInfo,0,sizeof(t_remoteControlInfo));
//     }
// } 

/**
 *RecvCarInfoKernel()
 *detail:Read serial data, publish after parsing
 */
// void actuator::serialRecvCarInfoKernel()
// {
//     unsigned char str[512];
//     memset(&str,0,sizeof(str));
//     std::string recvstr = ser.read(ser.available());
//     int lenrecv= recvstr.size(); //here read serialport data
//     if(lenrecv <= 0){
//     	mcuCount++;
//     	if(mcuCount >= 5){
//     		mcuCount = 5;
//     		sendWarnnigToMonitor(PERIPHERAL_FAULT,0);
//     	}
//     	return;
//     }else{
//     	mcuCount = 0;
//         if( (lenrecv+receiverCurrentByteCount) > sizeof(tempDataArray) )
//             return;
//     }
//     //printf("ivactuator-->lenrecv = %d\n",lenrecv);
//     for(int i=0; i<lenrecv; i++){
//         tempDataArray[i+receiverCurrentByteCount] = recvstr[i];
//     }
//     receiverCurrentByteCount+=lenrecv;
//     if(receiverCurrentByteCount < RECV_MSG_LEN){
//         return;
//     }
//     int headStartPosition = 0;
//     bool key = false;
//     for(int i=0; i<receiverCurrentByteCount; i++){
//         //printf("ivactuator-->recvstr[%d] = 0x%x\n",i,tempDataArray[i]);
//         str[i] = tempDataArray[i];
//         if((tempDataArray[i]==0XFF)&&(tempDataArray[i+1] == 0XA5)&&(tempDataArray[i+2]==0X5A) && !key){
//             headStartPosition= i;
//             if((headStartPosition+RECV_MSG_LEN)>(receiverCurrentByteCount))
//                 return;
//             else
//                 key = true;
//         }
//     } 
//     int count=0;
//     for(int i = headStartPosition;i<receiverCurrentByteCount;i+=RECV_MSG_LEN){ 
//         if(i+RECV_MSG_LEN<=receiverCurrentByteCount){
//             count++;
//             unsigned char sum = 0;
//             for(int j=3; j<RECV_MSG_LEN-1;j++){
//                 sum += str[i + j];
//             }
//             if(sum ==  str[i+RECV_MSG_LEN-1]) {//check sum
//                 memset(&ivactuatorMsg,0,sizeof(ivactuatorMsg));
//                 ivactuatorMsg.uisteerangle      = (short)( (str[i+5]<<8)|str[i+6] );
//                 ivactuatorMsg.realstrtorque     = (unsigned short)( (str[i+7]<<8)|str[i+8] );
//                 ivactuatorMsg.reserve1          = (signed)str[i+9];
//                 ivactuatorMsg.espaccel          = (unsigned short)( (str[i+10]<<8)|str[i+11] );
//                 ivactuatorMsg.shiftlvlposition  = (unsigned char)(str[i+12] ); 
//                 ivactuatorMsg.currentdrvmode    = (unsigned char)(str[i+13] );
//                 ivactuatorMsg.sysstatus         = (unsigned char)( str[i+14] );
//                 ivactuatorMsg.vehiclesysfault   = (unsigned char)( str[i+15] );
//                 ivactuatorMsg.batteryvoltage    = (unsigned short)( (str[i+16]<<8)|str[i+17] );
//                 ivactuatorMsg.reserve2          = (float)( (str[i+18]<<8)|(str[i+19]) );
//                 ivactuatorMsg.uispeed           = (float)((str[i+20]<<8)|str[i+21])/10.0/3.6;//m/s  
//                 ivactuatorMsg.evbpwmduty        = (unsigned char)(str[i+22]);
//                 ivactuatorMsg.evbbrkpressure    = (unsigned char)(str[i+23]);
//                 underMcuMonitor->sendWarnning(str[i+18], str[i+19]);
//                 pub_control.publish(ivactuatorMsg);
//             }
//         }
//     }
//     headStartPosition = headStartPosition+RECV_MSG_LEN*count;
//     int lenthTemp = receiverCurrentByteCount;
//     receiverCurrentByteCount = 0;
//     memset(tempDataArray,0,sizeof(tempDataArray));
//     if( headStartPosition < lenthTemp){
//         int k=0;
//         for(int j=headStartPosition; j<lenthTemp; j++,k++){
//             tempDataArray[k]=str[j];
//         }
//         receiverCurrentByteCount = k;
//     }
// }

/**
 *SendCarInfoKernel()
 *detail:Write data to serial port
 */
//hide on 0521 zt
// void actuator::serialSendCarInfoKernel()
// {
//     monitorFrontNodeData();
//     bool startRemote = false;//t_remoteControlInfo.start;
//     unsigned char sendBuffer[23] = {0xFF,0XA5,0X5A,0x12,0x81};
//     memset(&sendBuffer[5],0,sizeof(sendBuffer)-5);
//     unsigned short tmp = 0;
//     short tmp2 = startRemote?t_remoteControlInfo.steerAngle:t_steerControlMsg.msg.targetangle;
//     //printf("ivactuator--->%d\n",tmp2);    
// 	tmp  = tmp2; 
//     sendBuffer[5]  = tmp / 256;
//     sendBuffer[6]  = tmp % 256;
//     tmp2 = startRemote?200:t_steerControlMsg.msg.torque;
//     tmp  = tmp2;
//     sendBuffer[7]  = tmp / 256;
//     sendBuffer[8]  = tmp % 256;
//     tmp2 = startRemote?t_remoteControlInfo.targettorque:t_driverControlMsg.msg.targettorque;
//     tmp  = tmp2;
//     sendBuffer[9]  = tmp / 256;
//     sendBuffer[10] = tmp % 256;
//     tmp2 = startRemote?t_remoteControlInfo.targetacc:t_driverControlMsg.msg.targetacc;
//     tmp  = tmp2;
//     sendBuffer[11] = tmp / 256;
//     sendBuffer[12] = tmp % 256;
//     sendBuffer[13] = startRemote?t_remoteControlInfo.actuatormode:t_driverControlMsg.msg.actuatormode; //targetdriverstatue
//     sendBuffer[14] = 1;
//     sendBuffer[15] = startRemote?t_remoteControlInfo.targetshiftposition:t_driverControlMsg.msg.shiftposition;

//     unsigned char sum = 0;
//     for(int i = 3; i < 22; ++i)//clac sum
//         sum += sendBuffer[i];
//     sendBuffer[22] = (unsigned char)(sum);
//     ser.write(sendBuffer,23);
// }

/**
 *monitorFrontNodeData()
 *detail:monitor data quality of front node
 *if no data at 1s send warnning
 */
//hide on 0521 zt
// void actuator::monitorFrontNodeData()
// {
//     if(!t_steerControlMsg.isvalid && !t_driverControlMsg.isvalid){
//         steerControlDataCount++;
//         driverControlDataCount++;
//     }else{
//         if(!t_steerControlMsg.isvalid){
//           steerControlDataCount++;
//         }else{
//           steerControlDataCount = 0;
//           t_steerControlMsg.isvalid  = false;
//         }
//         if(!t_driverControlMsg.isvalid){
//           driverControlDataCount++;
//         }else{
//           driverControlDataCount = 0;
//           t_driverControlMsg.isvalid = false;
//         }
//     }
//     if(steerControlDataCount >= 20){
//       steerControlDataCount = 20;
//       sendWarnnigToMonitor(FRONT_NODE_DATA_QUALITY,0);
//       //sendWarnnigToMonitor(FRONT_NODE_ABSOLUTE_CYCLE,7);
//     }
//     if(driverControlDataCount >= 20){
//       driverControlDataCount = 20;
//       sendWarnnigToMonitor(FRONT_NODE_DATA_QUALITY,1);
//       //sendWarnnigToMonitor(FRONT_NODE_ABSOLUTE_CYCLE,3);
//     }
// }

/**
 *monitorNodeAbsoluteCycle()
 *
 *@absoluteCycle: Absolute cycle of front node
 *@relativeCycle:Relative cycle of front node
 *@N:      normal cycle of front node
 *@offset: offset of front nodes
 *detail:monitor front node absolute cycle
 */
//hide on 0521 zt
// void actuator::monitorNodeAbsoluteCycle(float absoluteCycle,int warnningType,float N,float offset)
// {
//     if(absoluteCycle>N+0.1){
//         sendWarnnigToMonitor(warnningType,3+offset);
//     }else if(absoluteCycle>N+0.05){
//         sendWarnnigToMonitor(warnningType,2+offset);
//     }else if(absoluteCycle>N+0.02){
//         sendWarnnigToMonitor(warnningType,1+offset);
//     }else if(absoluteCycle>N+0.01){
//         sendWarnnigToMonitor(warnningType,0+offset);
//     }

// }

/**
 *monitorNodeRelativeCycle()
 *
 *@absoluteCycle: Absolute cycle of front node
 *@relativeCycle:Relative cycle of front node
 *@offset: offset of front nodes
 *detail:monitor front node relative cycle
 */
//hide on 0521 zt
// void actuator::monitorNodeRelativeCycle(float relativeCycle,int warnningType,float offset)
// {
//     if(relativeCycle>1){
//         sendWarnnigToMonitor(warnningType,3+offset);
//     }else if(relativeCycle>0.5){
//         sendWarnnigToMonitor(warnningType,2+offset);
//     }else if(relativeCycle>0.3){
//         sendWarnnigToMonitor(warnningType,1+offset);
//     }else if(relativeCycle>0.1){
//         sendWarnnigToMonitor(warnningType,0+offset);
//     }
// }

/**
 *sendWarnnigToMonitor()
 *
 *@monitorWarnningType:  warnning type
 *@monitorWarnningValue: warnning value
 *detail:send Warnnig to Monitor node
 */
//hide on 0521 zt
// void actuator::sendWarnnigToMonitor(int mcuWarnningType,int mcuWarnningValue)
// {
//     if(mcuWarnningType || mcuWarnningValue)
//         mcuMonitor->sendWarnning(mcuWarnningType, mcuWarnningValue); //send failure to monitor node
// }
