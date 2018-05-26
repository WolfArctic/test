#include "gpsparse.h"

gpsparse::gpsparse(ros::NodeHandle nh)
{
  gpsDeviceName = 1;
  baudrate = 115200;
  serialport = "/dev/ttyS0";
  runningmode = 0;
  loopFrep = 10;
  fp = NULL;       //add by wangxiao

  //checksum
  checksum_gga = false;
  checksum_rmc = false;
  checksum_hpr = false;
  checksum_ksxt = false;
  checksum_vtg = false;

  is_debug = false;

#if 1
  //import paramters from lauch file
  nh.param("gpsbaudrate", baudrate, baudrate);
  nh.param("gpsserialport", serialport, serialport);
  nh.param("gpsdevice", gpsDeviceName, gpsDeviceName);
  nh.param("gpsloopfrep", loopFrep, loopFrep);
#endif
  //paramters
  std::cout << FRED("Copyright©2016-2020 idriverplus. All rights reserved ") << std::endl;
  std::cout << FYEL("*****ivsensorgps:parameters*******************") << std::endl;
  std::cout << FGRN("gps_baudrate: ") << baudrate << std::endl;
  std::cout << FGRN("gps_serialport: ") << serialport << std::endl;
  std::cout << FGRN("gps_device: ") << gpsDeviceName << std::endl;
  std::cout << FGRN("gps_runningmode:(0-dpcar, 1-simualtion) ") << runningmode << std::endl;
  std::cout << FGRN("gps_loopfrep: ") << loopFrep << std::endl;
  std::cout << FYEL("*****ivsensorgps:parameters end***************") << std::endl;
  //data initialize
  ReceiverCurrentByteCount = 0;
  memset(TempDataArray, 0, sizeof(TempDataArray));
  memset(&addattr, 0, sizeof(addattr));

  pub_ = nh.advertise<ivlocmsg::ivsensorgps>("ivsensorgps", 10);

  if (DPCAR == runningmode)
  {
    try
    {
      std::cout << FYEL(DEBUGHEAD) << "Serial star initialize" << std::endl;
      ser.setPort(serialport);
      ser.setBaudrate(baudrate);
      serial::Timeout to = serial::Timeout::simpleTimeout(1000);
      ser.setTimeout(to);
      ser.open();
    }
    catch (serial::IOException& e)
    {
      std::cout << FRED(DEBUGHEAD) << "Unable to open port" << std::endl;
      ROS_ERROR_STREAM(baudrate);
      ROS_ERROR_STREAM(serialport);
      ROS_ERROR_STREAM(gpsDeviceName);
    }
    if (ser.isOpen())
    {
      std::cout << FYEL(DEBUGHEAD) << "Serial port initialized successfully" << std::endl;
    }
    else
    {
      std::cout << FRED(DEBUGHEAD) << "Serial port failed" << std::endl;
    }
  }
  else
  {
    sub_ = nh.subscribe("ivvrep_gps", 10, &gpsparse::callback_vrep, this);
  }
}

/*run
 *author:wangxiao
 *date:2016.07.30
 *detail:receive data from serial
*/
void gpsparse::run()
{
  ros::Rate rate(loopFrep); //loop at 10 hz until ros is shut down
  while (ros::ok())
  {
    ros::NodeHandle nh;
    nh.getParam("road_attr", addattr.roadattr);
    if (DPCAR == runningmode)
    {
      naviColKernel();
    }
    else
    {
      ros::spinOnce();
    }
    //ros::spinOnce(); //there is no subscriber
    rate.sleep();
  }
}

/*callback_vrep
 *author:zhuxuekui
 *date:2016.07.27
 *detail:get data from vrep
 */
void gpsparse::callback_vrep(const std_msgs::Float64MultiArray::ConstPtr msg)
{
  // ROS_INFO_STREAM("Serial information:");
  sGpsPoint info;

  info.lon = msg->data[0]; //
  info.lat = msg->data[1];
  info.heading = msg->data[2];
  publishMsg(info);
}

/*naviColKernel
 *author:zhuxuekui
 *date:2016.07.19
 *detail:publish message
 */
void gpsparse::naviColKernel()
{
  std::string recvstr;
  recvstr = ser.read(ser.available());
  //here read serialport data
  const int bufsize = 4096;
  unsigned char revbuf[bufsize];
  memset(&revbuf, 0, sizeof(revbuf));
  int len = recvstr.size();
  for (int i = 0; i < len; ++i)
  {
    revbuf[i] = recvstr[i];
  }
  if (len > 0 && len < bufsize)
  {
    receiveData(revbuf, len);
  }
}

/*publishMsg
 *author:zhuxuekui
 *date:2016.07.19
 *detail:publish message
 */

void gpsparse::publishMsg(sGpsPoint data)
{
  ivlocmsg::ivsensorgps msg;
  msg.lon = data.lon;
  msg.lat = data.lat;
  msg.mode = data.mode;
  msg.heading = data.heading;
  msg.velocity = data.velocity/3.6;  //km/h-->m/s
  msg.status = data.status;
  msg.satenum = data.satenum;
  msg.utctime = data.utctime;

  msg.height = data.height;
  msg.track_angle = data.track_angle;
  msg.hdop = data.hdop;

  msg.ve = data.ve;
  msg.vn = data.vn;
  msg.vu = data.vu;
  msg.pos_status = data.pos_status;
  msg.att_status = data.att_status;
  msg.header.stamp = ros::Time::now();

  msg.isvalid = 1;
  
  pub_.publish(msg);
}
/*parseGpybm
 *author:zhuxuekui
 *date:2016.07.19 *detail:parse gpybm protocol */
void gpsparse::receiveData(unsigned char* str, int len)
{
  int receiver_type = 0;   // 1-hexinxingtong  2-lianshi

  checksum_gga = false;
  checksum_rmc = false;
  checksum_hpr = false;
  checksum_ksxt = false;
  checksum_vtg = false;  

  for (int i = 0; i < len; ++i)
  {
    TempDataArray[ReceiverCurrentByteCount] = str[i];
    ReceiverCurrentByteCount++;
    if (ReceiverCurrentByteCount > 2)
    {
      if (TempDataArray[ReceiverCurrentByteCount - 2] == 0X0D
          && TempDataArray[ReceiverCurrentByteCount - 1] == 0X0A )
      {
        if (ReceiverCurrentByteCount > 6)
        {
          if (TempDataArray[0] == '$' && TempDataArray[1] == 'G' && TempDataArray[2] == 'P'&& 
              TempDataArray[3] == 'G'&& TempDataArray[4] == 'G'&& TempDataArray[5] == 'A')
           {
              if(CheckSum(TempDataArray))
              {
                checksum_gga = true;
                parseGpgga();
              }
          }
          else if (TempDataArray[0] == '$' && TempDataArray[1] == 'G' && TempDataArray[2] == 'P' && 
                   TempDataArray[3] == 'R' && TempDataArray[4] == 'M' && TempDataArray[5] == 'C')
          {
            if(CheckSum(TempDataArray))
            {
               checksum_rmc = true;
               parseGprmc();
            }
          }
          else if (TempDataArray[0] == '$' && TempDataArray[1] == 'K' &&
               TempDataArray[2] == 'S' && TempDataArray[3] == 'X' && TempDataArray[4] == 'T')
          {
            if(CheckSum(TempDataArray))
            {
              checksum_ksxt = true;
              parseKSXT();
            }
          }
          else if (TempDataArray[0] == '$' && TempDataArray[1] == 'P' &&
                   TempDataArray[2] == 'S' && TempDataArray[3] == 'A')
          {
            if(CheckSum(TempDataArray))
            {
              checksum_hpr = true;
              parseGphpr();
            }
            receiver_type = 1; 
          }
          else if (TempDataArray[0] == '$' && TempDataArray[1] == 'G' && TempDataArray[2] == 'P' && 
                   TempDataArray[3] == 'V' && TempDataArray[4] == 'T' && TempDataArray[5] == 'G')
          {
            if(CheckSum(TempDataArray))
            {
              checksum_vtg = true;
              parseGpvtg();
            }
          }

          if (receiver_type == 1 && checksum_vtg && checksum_gga && checksum_rmc  && checksum_hpr)
          {
            rp.mode = addattr.roadattr;

            receiver_type = 0;

            double time = ros::Time::now().toSec();
            ROS_INFO_STREAM("PubGPSData:" << setprecision(12) << time);

            publishMsg(rp); //parse finish

            if (is_debug)
            {
              std::cout<<"checksum_gga:  "<<checksum_gga
                       <<"checksum_rmc:  "<<checksum_rmc
                       <<"checksum_hpr:  "<<checksum_hpr
                       <<"checksum_ksxt: "<<checksum_ksxt
                       <<"checksum_vtg:  "<<checksum_vtg
                       <<std::endl;
            }
          }
        }
        ReceiverCurrentByteCount = 0; //clear zero
      }
    }
  }
}

bool gpsparse::CheckSum(unsigned char *pstr)
{
  int index,result,checksum_value;
  for(result = pstr[1],index = 2;pstr[index] != '*';index++)
  {
    result ^= pstr[index];
  }
  std::stringstream stream;
  stream << std::hex << pstr[index+1] << pstr[index+2] << '\0';
  stream >> checksum_value;
  if(checksum_value == result)
    return true;
  else
    return false;
}

/*parseGphpr
 *author:zhuxuekui
 *date:2016.07.19
 *detail:parse gphpr protocol
 */
void gpsparse::parseGphpr()
{
  int strnum = 0;
  int DouNum = 0;
  string temp;
  for (int i = 0; i < ReceiverCurrentByteCount; ++i)
  {
    if (TempDataArray[i] == ',' ) {
      strnum ++;
      stringstream stream;
      stream << temp;
      switch (strnum) {
      case 4:
        stream >> rp.heading;
        stream.str(""); temp = ""; break;
      default: stream.str(""); temp = ""; break;
      }
    }
    else {
      temp += TempDataArray[i];
    }
  }
}

/*parseGprmc
 *author:zhuxuekui
 *date:2016.07.19
 *detail:parse gprmc protocol
 */
void gpsparse::parseGprmc()
{
  int strnum = 0;
  int DouNum = 0;
  string temp;
  for (int i = 0; i < ReceiverCurrentByteCount; ++i) {
    if (TempDataArray[i] == ',' ) {
      strnum ++;
      stringstream stream;
      stream << temp;
      switch (strnum)
      {
        case 8:
          stream >> rp.velocity;
          rp.velocity *= 1.852;       //knots
          stream.str(""); temp = ""; break;
        case 9:
          stream >> rp.track_angle;
          stream.str(""); temp = ""; break;
        default: stream.str(""); temp = ""; break;
      }
    }
    else {
      temp += TempDataArray[i];
    }
  }
}

/*parseKSXT
 *date:2017.12.20
 *detail:parse $KSXT protocol
 */
void gpsparse::parseKSXT()
{
  int strnum = 0;
  int DouNum = 0;
  string temp;
  for (int i = 0; i < ReceiverCurrentByteCount; ++i) {
    if (TempDataArray[i] == ',' ) {
      strnum ++;
      stringstream stream;
      stream << temp;
      switch (strnum)
      {
        case 11:
          stream >> rp.pos_status;
          stream.str(""); temp = ""; break;
        case 12:
          stream >> rp.att_status;
          stream.str(""); temp = ""; break;
        case 18:
          stream >> rp.ve;
          rp.ve = rp.ve/3.6;
          stream.str(""); temp = ""; break;
        case 19:
          stream >> rp.vn;
          rp.vn = rp.vn/3.6;
          stream.str(""); temp = ""; break;
        case 20:
          stream >> rp.vu;
          rp.vu = rp.vu/3.6;
          stream.str(""); temp = ""; break;
        default: stream.str(""); temp = ""; break;
      }
    }
    else {
      temp += TempDataArray[i];
    }
  }
}
/*parseGpvtg
 *author:zhuxuekui
 *date:2016.07.19
 *detail:parse gpvtg protocol
 */
void gpsparse::parseGpvtg()
{
  int strnum = 0;
  int DouNum = 0;
  string temp;
  for (int i = 0; i < ReceiverCurrentByteCount; ++i) {
    if (TempDataArray[i] == ',' ) {
      strnum ++;
      stringstream stream;
      stream << temp;
      switch (strnum) {
      case 2:  stream >> rp.track_angle;  stream.str(""); temp = ""; break;
      case 8:  stream >> rp.velocity;  stream.str(""); temp = ""; break;
      default: stream.str(""); temp = ""; break;
      }
    }
    else {
      temp += TempDataArray[i];
    }
  }
}

/*parseGpgga
 *author:zhuxuekui
 *date:2016.07.19
 *detail:parse gpgga protocol
 */
void gpsparse::parseGpgga()
{
  int strnum = 0;
  int DouNum = 0;
  string temp;
  double lon = 0;
  double lat = 0;
  int inter = 0;
  double utc = 0;
  for (int i = 0; i < ReceiverCurrentByteCount; ++i)
  {
    if (TempDataArray[i] == ',' )
    {
      strnum ++;
      stringstream stream;
      stream << temp;

      switch (strnum)
      {
      case 2:
        stream >> utc;
        rp.utctime = GetUtcTime(utc);;
        stream.str(""); temp = ""; break;
      case 3:
        stream >> lat;
        inter = (int)(lat) / 100;
        lat = lat - inter * 100;
        lat = lat / 60.0 + inter;
        rp.lat = lat;
        stream.str(""); temp = ""; break;
      case 5:
        stream >> lon;
        inter = (int)(lon) / 100;
        lon = lon - inter * 100;
        lon = lon / 60.0 + inter;
        rp.lon = lon;
        stream.str(""); temp = ""; break;
      case 7:
        stream >> rp.status;
        stream.str(""); temp = ""; break;
      case 8:
        stream >> rp.satenum;
        stream.str(""); temp = ""; break;
      case 9:
        stream >> rp.hdop;
        stream.str(""); temp = ""; break;
      case 10:
        stream >> rp.height;
        stream.str(""); temp = ""; break;
      default: stream.str(""); temp = ""; break;
      }//end switch
    }//end if
    else
    {
      temp += TempDataArray[i];
    }
  }//end for
}

double gpsparse::GetUtcTime(double time)
{
  int hour = 0;
  int mimute = 0;
  float second = 0.0;
  double utc;

  hour = (int)(time/10000);
  mimute = (int)((time - hour*10000)/100);
  second = time - hour*10000 - mimute*100;

  utc = hour*3600+mimute*60+second;
  return utc;
}

double gpsparse::getDist2(Point2D p1, Point2D p2)
{
  return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}


