void loclib::loctool::init(double input){
  gps_to_frame = input;
}

sGps loclib::loctool::getGps(sGps oringinGps, sPose2Dd poseInput, double height){
  const double E = 1.0/298.257;
  const double Re = 6378137.0;

  double originLatRad = degreeToRadian(oringinGps.lat);
  double originLonRad = degreeToRadian(oringinGps.lon);
  double antenna_distance = sqrt(pow(poseInput.x, 2) + pow(poseInput.y, 2));
  double bearingDeg = oringinGps.heading + poseInput.angle - radianToDegree(atan2(- poseInput.y, - poseInput.x));
  if(bearingDeg > 180)
  	bearingDeg -= 360;    
  else if(bearingDeg < -180)
  	bearingDeg += 360;
  double bearingRad = degreeToRadian(bearingDeg);

  double Rm = 0.0,Rn = 0.0;
  Rm = Re*(1-2*E+3*E*sin(originLatRad)*sin(originLatRad));
  Rn = Re*(1+E*sin(originLatRad)*sin(originLatRad));

  double Rx = 0.0,Ry = 0.0;
  Rx = antenna_distance * sin(bearingRad);
  Ry = antenna_distance * cos(bearingRad);

  ivlocmsg::ivsensorgps targetGps;
  targetGps.lon = radianToDegree(originLonRad + Rx/((Rn+height)*cos(originLatRad)));
  targetGps.lat = radianToDegree(originLatRad + Ry/(Rm+height));
  targetGps.heading = oringinGps.heading;
  return targetGps;
}

sPose3Dd loclib::loctool::get_delta_pose(sGps gps_source, sGps gps_input, double gps_start_angle){
  double dis = getDisBetweenGpsPoints(gps_source, gps_input);
  double bearing = getBearingOfGpsPts(gps_source, gps_input);
  //heading is anticlockwise, odom is clockwise
  double angle = gps_source.heading - bearing + gps_start_angle;
  if(angle > 180)
    angle -= 360;
  else if(angle < -180)
    angle += 360;

  double delta_angle = gps_source.heading - gps_input.heading;
  if(delta_angle > 180)
    delta_angle -= 360;
  else if(delta_angle < -180)
    delta_angle += 360;

  sPose3Dd temp_delta_pose;
  temp_delta_pose.x = dis * cos(degreeToRadian(angle)) +
    gps_to_frame * cos(degreeToRadian(gps_start_angle + delta_angle))
    - gps_to_frame * cos(degreeToRadian(gps_start_angle));
  temp_delta_pose.y = dis * sin(degreeToRadian(angle)) +
    gps_to_frame * sin(degreeToRadian(gps_start_angle + delta_angle))
    - gps_to_frame * sin(degreeToRadian(gps_start_angle));
  temp_delta_pose.yaw = degreeToRadian(delta_angle);
  return temp_delta_pose;
}

double loclib::loctool::degreeToRadian(double v){
    const double degToRadfactor = MPI/180; 
    return v * degToRadfactor;
} 
double loclib::loctool::radianToDegree(double v){
    const double radToDegfactor = 180/MPI;
    return v * radToDegfactor;
}

double loclib::loctool::getBearingOfGpsPts(sGps source, sGps target){
     double lon1 = source.lon;
     double lat1 = source.lat;
     double lon2 = target.lon;
     double lat2 = target.lat;
     double angle,angle1;
     double averageLat = (lat1 + lat2) / 2; 
     if(fabs(lat1 - lat2) <= 1e-20){
      angle = 90;
      if (lon1 > lon2)
        angle = angle + 180;
    }else{
      angle = atan((lon1 - lon2) * cos(degreeToRadian(averageLat)) / (lat1 - lat2)) * 180 / M_PI;
      if (lat1 > lat2)
        angle = angle + 180;
      if (angle < 0)
        angle = 360 + angle;
    }
    return angle;
}

double loclib::loctool::getDisBetweenGpsPoints(sGps pt1, sGps pt2){
   sPoint2Dd pt2d_1 = BLH2XYZ(pt1.lat,pt1.lon,0);
   sPoint2Dd pt2d_2 = BLH2XYZ(pt2.lat,pt2.lon,0);
   return sqrt((pt2d_1.x - pt2d_2.x) * (pt2d_1.x - pt2d_2.x) + (pt2d_1.y - pt2d_2.y) * (pt2d_1.y - pt2d_2.y));
}

double loclib::loctool::GetL0InDegree(double dLIn){
  double ZoneNumber = (int)((dLIn - 1.5) / 3.0) + 1;
  double L0 = ZoneNumber * 3.0;//degree
  return L0;
}

sPoint2Dd loclib::loctool::BLH2XYZ(double B, double L, double H){
  double N, E, h;
  double L0 = GetL0InDegree(L);
  sPoint2Dd pt2d;
  double a = 6378137.0;
  double F = 298.257223563;
  double iPI = 0.0174532925199433;
  double f = 1 / F;
  double b = a * (1 - f);
  double ee = (a * a - b * b) / (a * a);
  double e2 = (a * a - b * b) / (b * b);
  double n = (a - b) / (a + b), n2 = (n * n), n3 = (n2 * n), n4 = (n2 * n2), n5 = (n4 * n);
  double al = (a + b) * (1 + n2 / 4 + n4 / 64) / 2;
  double bt = -3 * n / 2 + 9 * n3 / 16 - 3 * n5 / 32;
  double gm = 15 * n2 / 16 - 15 * n4 / 32;
  double dt = -35 * n3 / 48 + 105 * n5 / 256;
  double ep = 315 * n4 / 512;
  B = B * iPI;
  L = L * iPI;
  L0 = L0 * iPI;
  double l = L - L0, cl = (cos(B) * l), cl2 = (cl * cl), cl3 = (cl2 * cl), cl4 = (cl2 * cl2), cl5 = (cl4 * cl), cl6 = (cl5 * cl), cl7 = (cl6 * cl), cl8 = (cl4 * cl4);
  double lB = al * (B + bt * sin(2 * B) + gm * sin(4 * B) + dt * sin(6 * B) + ep * sin(8 * B));
  double t = tan(B), t2 = (t * t), t4 = (t2 * t2), t6 = (t4 * t2);
  double Nn = a / sqrt(1 - ee * sin(B) * sin(B));
  double yt = e2 * cos(B) * cos(B);
  N = lB;
  N += t * Nn * cl2 / 2;
  N += t * Nn * cl4 * (5 - t2 + 9 * yt + 4 * yt * yt) / 24;
  N += t * Nn * cl6 * (61 - 58 * t2 + t4 + 270 * yt - 330 * t2 * yt) / 720;
  N += t * Nn * cl8 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;
  E = Nn * cl;
  E += Nn * cl3 * (1 - t2 + yt) / 6;
  E += Nn * cl5 * (5 - 18 * t2 + t4 + 14 * yt - 58 * t2 * yt) / 120;
  E += Nn * cl7 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;
  E += 500000;
  N = 0.9999 * N;
  E = 0.9999 * (E - 500000.0) + 250000.0;//Get y
  pt2d.x = E;
  pt2d.y = N;
  h = H;
  return pt2d;
}
