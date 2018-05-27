//
// Created by idriver on 17-3-14.
//

#include "motionplannerWBD.h"

motionplannerWBD::motionplannerWBD() { Init_motionplanner(); }

void motionplannerWBD::Init_motionplanner() {
  velocity_limit = 100.0;
  velocity_suggest = 100.0;
  T_crossin_car = 0.0;
  Aplus_max = 0.2;  // 0.1 TODO @yanbo
  Aminus_max = -0.2;
  PathID = 0;
  cnt_test = 0;
  speed_reset_obs = 0;
  speed_reset_obs_last = 0;
  speed_reset_staticmap = 0;
  speed_reset_staticmap_last = 0;
  guidespeed_min = 1.5 / 3.6;  // 3
  sbjspeed = 0.0;
  Dsafe = DIST_PREOBS;
  Distl_obs_min = 88.0;
  Distw_obs_min = 88.0;
  objsteps_predict = 50.0;
  refpath_source = 1;
  lidar_stop_pathid = 888;
  lidar_seg_update = 0;
  emergency_flag = 0;
  isspeedlimit = 0;
  static_obs_flag = 0;
  speedupdate = 1;
  emergency_decision = 0;
  avoid_path = false;
  cautious_status = false;
  index_start = 1;
  dist_car = 0.1;
  length_last = 0;
  speedlimit = 0;
}

motionplannerWBD::~motionplannerWBD() {}

void motionplannerWBD::Velocitygenbasedroad(
    ivpathplanner::ivmsgpathplanner option_paths_in) {
  if (option_paths_in.paths.size() == 0) {
    return;
  }
  pathprocess(option_paths_in);

  curvefitting(option_pathpoints);  // option_paths_in;c
  // curvature_cal(option_pathpoints);//failed;
  velocitygenetation(option_pathpoints);
}

void motionplannerWBD::Velocitygenbasedobs(
    std::vector<std::vector<sPoint>> &option_pathpoints,
    ivpredict::ivmsgpredict obs, ivmap::ivmapmsgbdcell obs_map) {
  if (option_pathpoints.size() == 0) {
    return;
  }

  // Velocity_staticobs(option_pathpoints,obs_map);
  ros::Time pre_run = ros::Time::now();
  Velocity_dynamicobs(option_pathpoints, obs);
}

void motionplannerWBD::PathGeneration(
    std::vector<std::vector<sPoint>> option_pathpoints)  // road roadsource???
{
  finalpath.points.clear();
  if (option_pathpoints.size() == 0) {
    return;
  }

  for (int i = 0; i < option_pathpoints[PathID].size(); i++) {
    motionpoint.x = option_pathpoints[PathID][i].pathpoints_in.x;
    motionpoint.y = option_pathpoints[PathID][i].pathpoints_in.y;
    motionpoint.xc = option_pathpoints[PathID][i].pathpoints_in.xc;
    motionpoint.yc = option_pathpoints[PathID][i].pathpoints_in.yc;
    motionpoint.xg = option_pathpoints[PathID][i].pathpoints_in.xg;
    motionpoint.yg = option_pathpoints[PathID][i].pathpoints_in.yg;
    motionpoint.angle = option_pathpoints[PathID][i].pathpoints_in.angle;
    motionpoint.velocity = option_pathpoints[PathID][i].pathpoints_in.velocity;
    // if (isspeedlimit == 1 && motionpoint.velocity <= 1/3.6 &&
    // motionpoint.velocity >=0.0){
    //   motionpoint.velocity = 1/3.6;
    // }
    if (i < index_start && motionpoint.velocity < 0.1) {
      motionpoint.velocity = 0.1;
    }
    lidar_seg_update = isspeedlimit;
    motionpoint.horn = 0;
    motionpoint.turnlight = 0;
    motionpoint.alertlight = 0;
    motionpoint.emergency_flag = emergency_flag;
    if (fabs(option_pathpoints[PathID][i].y_fitting) < 20000) {
      motionpoint.y_fit = option_pathpoints[PathID][i].y_fitting;
    } else {
      motionpoint.y_fit = option_pathpoints[PathID][i - 1].y_fitting;
    }

    if (i < 50) {
    }
    motionvars.speed =
        option_pathpoints[PathID][index_start + 1].pathpoints_in.velocity;
    // motionvars.relspeed_x =
    // option_pathpoints[PathID][5].pathpoints_in.velocity;
    //
    finalpath.points.push_back(motionpoint);
  }
}

void motionplannerWBD::pathprocess(
    ivpathplanner::ivmsgpathplanner option_paths_in) {
  pathpoints.clear();
  option_pathpoints.clear();
  Stopflag.clear();
  for (int i = 0; i < option_paths_in.paths.size(); i++) {
    float32 dis2ps = 0.0;
    float32 length_segment = 0.0;
    pathpoints.clear();
    double stopval = option_paths_in.paths[i].stopflag;
    Stopflag.push_back(stopval);
    for (int j = 0; j < option_paths_in.paths[i].points.size(); j++) {
      point.curvature = CURVATURE_INIT;
      point.pathpoints_in.x = option_paths_in.paths[i].points[j].x;
      point.pathpoints_in.y = option_paths_in.paths[i].points[j].y;
      point.pathpoints_in.xc = option_paths_in.paths[i].points[j].xc;
      point.pathpoints_in.yc = option_paths_in.paths[i].points[j].yc;
      point.pathpoints_in.xg = option_paths_in.paths[i].points[j].xg;
      point.pathpoints_in.yg = option_paths_in.paths[i].points[j].yg;
      point.pathpoints_in.angle = option_paths_in.paths[i].points[j].angle;
      point.pathpoints_in.velocity =
          option_paths_in.paths[i].points[j].velocity;
      if (j == 0) {
        // point.length = sqrt(option_paths_in.paths[i].points[j].x *
        // option_paths_in.paths[i].points[j].x +
        // option_paths_in.paths[i].points[j].y *
        // option_paths_in.paths[i].points[j].y);
        point.length = 0;
      } else {
        dis2ps = sqrt((option_paths_in.paths[i].points[j].x -
                       option_paths_in.paths[i].points[j - 1].x) *
                          (option_paths_in.paths[i].points[j].x -
                           option_paths_in.paths[i].points[j - 1].x) +
                      (option_paths_in.paths[i].points[j].y -
                       option_paths_in.paths[i].points[j - 1].y) *
                          (option_paths_in.paths[i].points[j].y -
                           option_paths_in.paths[i].points[j - 1].y));
        point.length = length_segment + dis2ps;
      }
      length_segment = point.length;
      if (length_segment > dist_car && length_last <= dist_car) {
        index_start = j;
      }
      // index_start = 22;
      length_last = length_segment;
      pathpoints.push_back(point);
    }
    option_pathpoints.push_back(pathpoints);
  }
  // motionvars.speed_test =
  // option_pathpoints[PathID][5].pathpoints_in.velocity;
}

void motionplannerWBD::curvefitting(
    std::vector<std::vector<sPoint>> &option_pathpoints) {
  // Least square method for curve fitting or ...;
  int fitting_order = 2;
  int fittingpoints_step = 30;
  int fitting_flag = 0;

  std::vector<pt> sample;
  sample.clear();
  pt temp;
  for (int i = 0; i < option_pathpoints.size(); i++) {
    float64 fitting_length = 8;  // according to the roadtype;
    float64 fitting_length_init = 0;
    sample.clear();
    for (int j = 0; j < option_pathpoints[i].size(); j++) {
      temp.x = option_pathpoints[i][j].pathpoints_in.x;
      temp.y = option_pathpoints[i][j].pathpoints_in.y;
      sample.push_back(temp);
      if (sample.size() == 1) {
        fitting_length_init = option_pathpoints[i][j].length;
      }
      if (option_pathpoints[i][j].length - fitting_length_init >=
          fitting_length) {
        fitting_flag = 1;
      }
      if (fitting_flag == 1) {
        doubleVector Coef;
        Coef = getCoeff(sample, fitting_order);
        for (int n = 0; n < sample.size(); n++) {
          double y_fit = 0;
          double dy_fit = 0;
          double ddy_fit = 0;
          for (int m = 0; m <= fitting_order; m++) {
            y_fit += Coef[m] * pow(sample[n].x, m);
          }
          for (int m = 1; m <= fitting_order; m++) {
            dy_fit += m * Coef[m] * pow(sample[n].x, m - 1);
          }
          for (int m = 2; m <= fitting_order; m++) {
            ddy_fit += (m - 1) * m * Coef[m] * pow(sample[n].x, m - 2);
          }

          option_pathpoints[i][j - sample.size() + n + 1].y_fitting = y_fit;
          option_pathpoints[i][j - sample.size() + n + 1].curvature =
              fabs(ddy_fit) /
              sqrt((1 + dy_fit * dy_fit) * (1 + dy_fit * dy_fit) *
                   (1 + dy_fit * dy_fit));
        }
        fitting_flag = 0;
        sample.clear();
      } else {
        if (option_pathpoints[i][option_pathpoints[i].size() - 1].length <
                fitting_length &&
            option_pathpoints[i][j].curvature < 0) {
          option_pathpoints[i][j].curvature = 0.2;
        }
      }
    }
  }
}
void motionplannerWBD::velocitygenetation(
    std::vector<std::vector<sPoint>> &option_pathpoints) {
  float32 velocity_temp = 100.0;
  float32 Diff_curvature_max = 0.002;

  bool speedlim_change = false;
  if (fabs(speedlimit - decisions.vellimit) > 0.1) {
    speedlim_change = true;
  }

  speedlimit = decisions.vellimit;
// emergency_decision = emergency
#if 1
  for (int i = 0; i < option_pathpoints.size(); i++) {
    for (int j = 1; j < option_pathpoints[i].size() - 1; j++) {
      if (fabs(option_pathpoints[i][j].curvature -
               option_pathpoints[i][j - 1].curvature) >= Diff_curvature_max &&
          fabs(option_pathpoints[i][j].curvature -
               option_pathpoints[i][j + 1].curvature) >= Diff_curvature_max) {
        option_pathpoints[i][j].curvature =
            option_pathpoints[i][j - 1].curvature;
      }
    }
  }
#endif

  for (int i = 0; i < option_pathpoints.size(); i++) {
    double cnt = 0;

    for (int j = 0; j < option_pathpoints[i].size(); j++) {
      if (true == speedlim_change || speedupdate == 1 ||
          (option_pathpoints[i][j].pathpoints_in.velocity == VELOCITY_ISVALID &&
           option_pathpoints[i][j].curvature >= 0) ||
          speed_reset_obs == 1 || (sbjspeed <= 0.6 && Distl_obs_min >= 5))

      {
        if (decisions.refpathsource == 1) {
          lidar_stop_pathid = 888;
          if (option_pathpoints[i][j].curvature == CURVATURE_INIT) {
            velocity_temp = -88;
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.002) {
            velocity_temp = 0.8;  // 1.2;
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.004) {
            velocity_temp = 0.8;  // 1.2;
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.006) {
            velocity_temp = 0.8;  // 1.2;//1.2
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.008) {
            velocity_temp = 0.8;  // 1.2
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.01) {
            velocity_temp = 0.5;  // 1.2
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.02) {
            velocity_temp = 0.5;  // 1.2
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.05) {
            velocity_temp = 0.5;  // 1.0
          } else if (fabs(option_pathpoints[i][j].curvature) < 0.2) {
            velocity_temp = 0.3;  // 0.8
          } else {
            velocity_temp = 0.3;  // 0.5
          }
          // velocity_temp = 1.2;
          if (velocity_temp >=
              decisions.vellimit)  // when ,where to limitspeed?
          {
            velocity_temp = decisions.vellimit;
          }
          // if(emergency_decision == 1){
          //    velocity_temp = 0.0;
          // }
        } else {
          lidar_stop_pathid = 888;
          velocity_temp = VELOCITY_CURVE_MIN;
        }
// velocity_temp = 10/3.6;
#if 1
        if (option_pathpoints[i].size() - j <= 2 &&
            option_pathpoints[i][j].length <= 10) {
          option_pathpoints[i][j].pathpoints_in.velocity = SPEED_STOP;  // 0;
        } else {
          if (decisions.refpathsource == 1) {
            int start_index = 0;
            /***********ADD 0924 @yanbo ********/
            for (int k = 0; k <= option_pathpoints[i].size(); k++) {
              /* if (option_pathpoints[i][k].pathpoints_in.x >= 0){
                 start_index = k;
                 break;
               }*/

              if (j > start_index) {
                option_pathpoints[i][j].pathpoints_in.velocity = velocity_temp;
              } else {
                option_pathpoints[i][j].pathpoints_in.velocity =
                    sbjspeed + 0.1;  // TO BE TESTED
                if (sbjspeed - VELOCITY_STRAIGHT_MAX >= 0.01 / 3.6) {
                  option_pathpoints[i][j].pathpoints_in.velocity =
                      VELOCITY_STRAIGHT_MAX + 0.01 / 3.6;
                }
              }
            }
          } else {
          }
        }
#endif
      }
    }
    speed_reset_obs = 0;
    speed_reset_staticmap = 0;
    speedupdate = 0;
  }
  refpath_source = decisions.refpathsource;
  if (decisions.refpathsource == 1) {
    for (int i = 0; i < option_pathpoints.size(); i++) {
      for (int j = 1; j < option_pathpoints[i].size(); j++) {
        float32 diff_max = 0.05;
        float32 diff_velocity =
            option_pathpoints[i][j].pathpoints_in.velocity -
            option_pathpoints[i][j - 1].pathpoints_in.velocity;
        if (diff_velocity > diff_max) {
          float32 velocity_temp =
              sqrt(option_pathpoints[i][j - 1].pathpoints_in.velocity *
                       option_pathpoints[i][j - 1].pathpoints_in.velocity +
                   2 * Aplus_max * (option_pathpoints[i][j].length -
                                    option_pathpoints[i][j - 1].length));
          if (option_pathpoints[i][j].pathpoints_in.velocity > velocity_temp) {
            option_pathpoints[i][j].pathpoints_in.velocity = velocity_temp;
          }
        }
      }
    }
    for (int i = 0; i < option_pathpoints.size(); i++) {
      for (int j = option_pathpoints[i].size() - 1; j > 0; j--) {
        float32 diff_max = 0.05;
        float32 diff_velocity =
            option_pathpoints[i][j - 1].pathpoints_in.velocity -
            option_pathpoints[i][j].pathpoints_in.velocity;
        if (diff_velocity > diff_max) {
          float32 velocity_temp =
              sqrt(option_pathpoints[i][j].pathpoints_in.velocity *
                       option_pathpoints[i][j].pathpoints_in.velocity -
                   2 * Aminus_max * (option_pathpoints[i][j].length -
                                     option_pathpoints[i][j - 1].length));
          if (option_pathpoints[i][j - 1].pathpoints_in.velocity >
              velocity_temp) {
            option_pathpoints[i][j - 1].pathpoints_in.velocity = velocity_temp;
          }
        }
      }
    }
  } else {
  }

  //	for(int i = 0;i<option_pathpoints.size();i++)
  //     {
  //         for(int j = option_pathpoints[i].size()-1;j>0;j--)
  //         {
  //		option_pathpoints[i][j].pathpoints_in.velocity = 0.8;
  //	}
  //}
  // motionvars.speed = option_pathpoints[0][22].pathpoints_in.velocity;
  // motionvars.lidar_seg_update = (double)lidar_seg_update;
  // motionvars.source_last = refpath_source;
}
void motionplannerWBD::Velocity_staticobs(
    std::vector<std::vector<sPoint>> &option_pathpoints,
    ivmap::ivmapmsgbdcell obs_map) {
  if (obs_map.rightcell.size() == 0) {
    return;
  }
  for (int i = 0; i < option_pathpoints.size(); i++) {
    double dist_obsmap_min = 88.0;
    double length_obsmap_min = 10.0;
    for (int j = 0; j < obs_map.rightcell.size(); j++) {
      objpos pos_temp;
      double x = obs_map.rightcell[j].xc / 10;
      double y = obs_map.rightcell[j].yc / 10;
      pos_temp = objPosCal(x, y, option_pathpoints[i]);
      if (dist_obsmap_min >= pos_temp.dist &&
          option_pathpoints[i][pos_temp.posid].length < length_obsmap_min) {
        dist_obsmap_min = pos_temp.dist;
      }
      float32 velocity_obs = velocity_sobj(
          pos_temp.dist,
          option_pathpoints[i][pos_temp.posid].pathpoints_in.velocity);
      option_pathpoints[i][pos_temp.posid].pathpoints_in.velocity =
          velocity_obs;
      int posid_preobs = 0;
      for (int h = pos_temp.posid; h > 0; h--) {
        double dist =
            dist_get(option_pathpoints[i][pos_temp.posid].pathpoints_in.x,
                     option_pathpoints[i][pos_temp.posid].pathpoints_in.y,
                     option_pathpoints[i][h].pathpoints_in.x,
                     option_pathpoints[i][h].pathpoints_in.y);
        if (dist >= DIST_PREOBS_S) {
          posid_preobs = h;
          break;
        }
      }
      for (int n = posid_preobs; n < pos_temp.posid; n++) {
        if (option_pathpoints[i][n].pathpoints_in.velocity >
            option_pathpoints[i][pos_temp.posid].pathpoints_in.velocity)
          option_pathpoints[i][n].pathpoints_in.velocity =
              option_pathpoints[i][pos_temp.posid].pathpoints_in.velocity;
      }
      if (option_pathpoints[i][pos_temp.posid].pathpoints_in.velocity ==
          (float32)SPEED_STOP) {
        for (int k = posid_preobs; k < option_pathpoints[i].size(); k++) {
          option_pathpoints[i][k].pathpoints_in.velocity = SPEED_STOP;
        }
      }
      if (speed_reset_staticmap_last == 1 &&
          dist_obsmap_min > PASSWIDTH_LOWER &&
          option_pathpoints[i][1].pathpoints_in.velocity <= guidespeed_min) {
        speed_reset_staticmap = 1;
        speed_reset_staticmap_last = 0;
      }
      if (dist_obsmap_min >= PASSWIDTH_LOWER_DY) {
        speed_reset_staticmap_last = 0;
        // speed_reset_obs = 0;
      } else {
        speed_reset_staticmap = 0;
        speed_reset_staticmap_last = 1;
      }
    }
  }
#if 1
  for (int i = 0; i < option_pathpoints.size(); i++) {
    for (int j = 1; j < option_pathpoints[i].size(); j++) {
      float32 diff_max = 0.1;
      float32 diff_velocity =
          option_pathpoints[i][j].pathpoints_in.velocity -
          option_pathpoints[i][j - 1].pathpoints_in.velocity;
      if (diff_velocity > diff_max) {
        float32 velocity_temp =
            sqrt(option_pathpoints[i][j - 1].pathpoints_in.velocity *
                     option_pathpoints[i][j - 1].pathpoints_in.velocity +
                 2 * Aplus_max * (option_pathpoints[i][j].length -
                                  option_pathpoints[i][j - 1].length));
        if (option_pathpoints[i][j].pathpoints_in.velocity > velocity_temp) {
          option_pathpoints[i][j].pathpoints_in.velocity = velocity_temp;
        }
      }
    }
  }
  for (int i = 0; i < option_pathpoints.size(); i++) {
    for (int j = option_pathpoints[i].size() - 1; j > 0; j--) {
      float32 diff_max = 0.1;
      float32 diff_velocity =
          option_pathpoints[i][j - 1].pathpoints_in.velocity -
          option_pathpoints[i][j].pathpoints_in.velocity;
      if (diff_velocity > diff_max) {
        float32 velocity_temp =
            sqrt(option_pathpoints[i][j].pathpoints_in.velocity *
                     option_pathpoints[i][j].pathpoints_in.velocity -
                 2 * Aminus_max * (option_pathpoints[i][j].length -
                                   option_pathpoints[i][j - 1].length));
        if (option_pathpoints[i][j - 1].pathpoints_in.velocity >
            velocity_temp) {
          option_pathpoints[i][j - 1].pathpoints_in.velocity = velocity_temp;
        }
      }
    }
  }
#endif
}

void motionplannerWBD::Velocity_dynamicobs(
    std::vector<std::vector<sPoint>> &option_pathpoints,
    ivpredict::ivmsgpredict obs) {
  dynamic_obs.objects.clear();
  ivpredict::predictobj object;
  object.positions.clear();
  float32 Dist_thresthold = 0.5;
  doubleVector dynamic_obs_disp;
  Distl_obs_min = 88.0;
  Distw_obs_min = 88.0;
  dynamic_obs_disp.clear();
  if (obs.objects.size() == 0) {
    return;
  }
  for (int i = 0; i < obs.objects.size(); i++) {
    objsteps_predict = obs.objects[i].steps;
    if (1) {
      object = obs.objects[i];
      dynamic_obs.objects.push_back(object);
      dynamic_obs_disp.push_back(0);
    }
  }
  for (int i = 0; i < option_pathpoints.size(); i++) {
    double checksum = 0;
    for (int j = 0; j < dynamic_obs.objects.size(); j++) {
      dynamic_obs_disp[j] =
          Velocity_dynamicobj(option_pathpoints[i], dynamic_obs.objects[j]);
      checksum = checksum + dynamic_obs_disp[j];
    }
    if (speed_reset_obs_last == 1 && checksum == dynamic_obs.objects.size()) {
      speed_reset_obs = 1;
      speed_reset_obs_last = 0;
    }
    if (checksum == dynamic_obs.objects.size())  //||sbjspeed<0.1)
    {
      speed_reset_obs_last = 0;
      // speed_reset_obs = 0;
    } else {
      speed_reset_obs = 0;
      speed_reset_obs_last = 1;
    }
    if (sbjspeed < 0.3 &&
        Distw_obs_min > PASSWIDTH_LOWER)  ////TODO @YANBO 0924 0.1M/S
    {
      speed_reset_obs = 1;
    }
  }
  // motionvars.Distl_obs_min = Distl_obs_min;
  // motionvars.speed_reset_obs = (double)isspeedlimit;
}

objpos motionplannerWBD::objPosCal(float32 x, float32 y,
                                   std::vector<sPoint> pathpoints) {
  objpos poscal;
  float64 disThre = INVALIDDOUBLE;
  int32 indexStamp = 0;
  for (int i = 0; i < pathpoints.size(); i++) {
    float64 ptX = pathpoints[i].pathpoints_in.x;
    float64 ptY = pathpoints[i].pathpoints_in.y;
    float64 dis = sqrt((x - ptX) * (x - ptX) + (y - ptY) * (y - ptY));

    if (disThre > dis) {
      disThre = dis;
      indexStamp = i;
    }
    poscal.dist = disThre;
    poscal.posid = indexStamp;
  }
  return poscal;
}
float32 motionplannerWBD::velocity_sobj(float32 dist, float32 velocity_origin) {
  // According the obj's classification and  Power field Gaussian distribution
  float32 velocity_temp;
  if (dist < PASSWIDTH_LOWER) {
    if (isspeedlimit == 1) {
      velocity_temp = guidespeed_min;
    } else {
      velocity_temp = SPEED_STOP;  // 0
    }
  } else if (dist < PASSWIDTH_LOWER_DY) {
    velocity_temp = guidespeed_min +
                    (dist - PASSWIDTH_LOWER) /
                        (PASSWIDTH_LOWER_DY - PASSWIDTH_LOWER) *
                        (VELOCITY_CURVE_MIN - guidespeed_min);
  } else if (dist <= PASSWIDTH_UPPER_DY)  // PASSWIDTH_UPPER_DY
  {
    double speed_origin = VELOCITY_STRAIGHT_MAX;
    if (speed_origin < velocity_origin) {
      speed_origin = velocity_origin;
    }
    velocity_temp = VELOCITY_CURVE_MIN +
                    (dist - PASSWIDTH_LOWER_DY) /
                        (PASSWIDTH_UPPER_DY - PASSWIDTH_LOWER_DY) *
                        (speed_origin - VELOCITY_CURVE_MIN);
  } else {
    velocity_temp = VELOCITY_STRAIGHT_MAX;
  }
  if (velocity_temp > velocity_origin) {
    velocity_temp = velocity_origin;
  }

  return velocity_temp;
}

double motionplannerWBD::Velocity_dynamicobj(std::vector<sPoint> &pathpoints,
                                             ivpredict::predictobj objpred) {
  objpos pos_temp, pos_in, pos_out, pos_center, pos_temp_last, pos_pre;
  pos_center.dist = INVALIDDOUBLE;
  pos_center.posid = 0;
  pos_center.time_inter = 88.0;
  pos_pre.dist = INVALIDDOUBLE;
  pos_pre.posid = 0;
  pos_pre.time_inter = 88.0;
  pos_out.dist = INVALIDDOUBLE;
  pos_out.posid = 0;
  pos_out.time_inter = 88.0;
  pos_in.dist = INVALIDDOUBLE;
  pos_in.posid = 0;
  pos_in.time_inter = 88.0;
  pos_temp_last.dist = INVALIDDOUBLE;
  pos_temp_last.posid = 0;
  pos_temp_last.time_inter = 88.0;
  std::vector<objpos> pos_inside;
  pos_inside.clear();
  float32 Tsafe_bef_c = 88.0;
  float32 Tsafe_aft_c = 88.0;
  float32 Tsafe_inside_c = 88.0;
#if 1
  double objpos_dist_min = 88.0;
  double objpos_dist_min0 = 88.0;
  double objpos_length_min = 88.0;
  double objpos_length_thrd = 8.0;  // Modify TBD
  double objpos_length_min0 = 88.0;

  double pos_x_nearside = 0.0;
  double pos_y_nearside = 0.0;
  // Dsafe set;
  double relspeed_x = objpred.xvrel;

  Dsafe = DIST_PREOBS - 6 * relspeed_x;  // 8,6,7
  if (Dsafe >= DIST_PREOBS) {
    Dsafe = DIST_PREOBS;
  }
  if (Dsafe <= DIST_PREOBS_MIN) {
    Dsafe = DIST_PREOBS_MIN;
  }
  double Dsafe_temp = Dsafe;
  if (sbjspeed < 0.05) {
    Dsafe = Dsafe_temp + 0.5;
  }
  // Dsafe = DIST_PREOBS;

  /******* TODO @yanbo 0924 add for path&motion collision ****/
  static_obs_flag = 0;
  if (objpred.v_abs < 0.01) {
    static_obs_flag = 1;
  }

  // TODO @yanbo 20171127 for obj backward;

  cautious_status = cautious_path(avoid_path, pathpoints);
  // motionvars.test_x_obj = (double)cautious_status;

  // if(objpred.positions[0].xmean < 1 && fabs(objpred.positions[0].ymean) <=
  // PASSWIDTH_LOWER)//1
  /* if(objpred.positions[0].xmean < (0.1 + 1.2) &&
   fabs(objpred.positions[0].ymean) < 0.5){ //TODO @yanbo 1124
      objsteps_predict = 0;
   }
   else if(objpred.positions[0].xmean < (1 + 1.2) && objpred.xvrel < 0)//1
   {
     objsteps_predict = 1;
   }
   else if(objpred.positions[0].xmean < (1 + 1.2) && objpred.xvrel > 0)
   {
     if(sqrt(objpred.yvrel*objpred.yvrel + objpred.xvrel*objpred.xvrel) >=
   3/3.6)
     {
       objsteps_predict = 5 + 2 * int((sqrt(objpred.yvrel*objpred.yvrel +
   objpred.xvrel*objpred.xvrel)*3.6 - 3));
       if(objsteps_predict >= 10)
       {
         objsteps_predict = 10;
       }
     }
     else
     {
       objsteps_predict = 5;
     }
     if (cautious_status == false) {
        objsteps_predict = 1;
     }
     objsteps_predict = 1;
   }
   else if(objpred.positions[0].xmean >=(1 + 1.2) && fabs(objpred.yvrel)>=
   4/3.6)//3
   {
      objsteps_predict = 10 + 2 * int((fabs(objpred.yvrel)*3.6 - 3));
      if(objsteps_predict >= 20)
      {
        objsteps_predict = 20;
      }
   }
   else if(objpred.positions[0].xmean >=(1 + 1.2) &&
   fabs(objpred.positions[0].ymean)<= PASSWIDTH_LOWER_DY && fabs(objpred.yvrel)<
   4/3.6)
   {
     objsteps_predict = 1;//1 + 2 * int((fabs(objpred.yvrel)*3.6 - 1));
   }
   else
   {
     objsteps_predict = 10;
   }
   if (objpred.positions[0].xmean >= 15) {
     objsteps_predict = 0;
   }

   if (objpred.xvrel + sbjspeed <= -0.3 && objpred.v_abs > 0.2 ) {
       objsteps_predict = 0;
   }
   if(objsteps_predict >= objpred.steps){
     objsteps_predict = objpred.steps;
   }
   if (objsteps_predict > 1) {
      objsteps_predict = 1;
    }*/
  if (objpred.positions[0].xmean < 1.3) {  // TODO @yanbo 1124
    objsteps_predict = 0;
  } else if (objpred.positions[0].xmean >= 15) {
    objsteps_predict = 0;
  }

  else if (objpred.xvrel + sbjspeed <= -0.3 && objpred.v_abs > 0.2) {
    objsteps_predict = 0;
  } else {
    objsteps_predict = 1;
  }
  // objsteps_predict = 1;

  if (objsteps_predict >= 1) {
    for (int32 j = 0; j < objsteps_predict; j++)  // objpred.steps
    {
#if 0
      objpred.width = 0.0;
      objpred.length = 0.0;
#endif
      objpred.length = 0.0;
      double nearside_y = 88.88;
      double nearside_x = 88.88;
      double width_calu = 0.5 * objpred.width;
      double xcell = 88.88;
      double ycell = 88.88;
      if (objpred.cell.size() >= 1) {
        for (int i = 0; i < objpred.cell.size(); i++) {
          double y = objpred.cell[i].y;
          double x = objpred.cell[i].x;
          pos_temp = objPosCal(x, y, pathpoints);
          if (nearside_y > pos_temp.dist) {
            nearside_y = pos_temp.dist;
            ycell = y;
            // width_calu = fabs(objpred.positions[j].ymean - y);
          }
          if (nearside_x > pathpoints[pos_temp.posid].length) {
            nearside_x = pathpoints[pos_temp.posid].length;
            xcell = x;
          }
        }
      } else {
        // width_calu = 0.5 * objpred.width;
      }
      if (static_obs_flag == 1 && isspeedlimit == 0) {
        // width_calu += 0.3;
      }

      /*if(objpred.positions[j].ymean >=0)
      {
        pos_y_nearside = objpred.positions[j].ymean - width_calu;
      }
      else
      {
         pos_y_nearside = objpred.positions[j].ymean + width_calu;
      }*/

      // pos_x_nearside = objpred.positions[j].xmean - 0.5*objpred.length;
      // pos_x_nearside = pos_x_nearside + sbjspeed*j*objpred.time_interval;

      pos_temp = objPosCal(xcell, ycell, pathpoints);
      double length_obj =
          pathpoints[pos_temp.posid]
              .length;  // dist_get(pos_x_nearside,pos_y_nearside,0.0,0.0);
      if (objpos_dist_min > pos_temp.dist) {
        objpos_dist_min = pos_temp.dist;
      }
      if (objpos_dist_min0 > pos_temp.dist &&
          pathpoints[pos_temp.posid].length <= objpos_length_thrd) {
        objpos_dist_min0 = pos_temp.dist;
      }
      if (objpos_length_min0 > pathpoints[pos_temp.posid].length) {
        objpos_length_min0 = pathpoints[pos_temp.posid].length;
      }
      if (pos_temp.dist <= PASSWIDTH_LOWER && pos_temp.dist < INVALIDDOUBLE) {
        pos_temp.time_inter = (j + 1) * objpred.time_interval;
        pos_inside.push_back(pos_temp);
      }
      if (pos_center.dist > pos_temp.dist && pos_temp.dist <= PASSWIDTH_LOWER &&
          pos_temp.dist != INVALIDDOUBLE) {
        pos_center.dist = pos_temp.dist;
        pos_center.posid = pos_temp.posid;
        pos_center.time_inter = (j + 1) * objpred.time_interval;
      }
      // if(pos_pre.dist >
      // pos_temp.dist&&pos_temp.dist>PASSWIDTH_LOWER&&pos_temp.dist<=PASSWIDTH_UPPER_DY)
      if (pos_pre.dist > pos_temp.dist && pos_temp.dist > PASSWIDTH_LOWER &&
          pos_temp.dist <= PASSWIDTH_UPPER_DY &&
          pathpoints[pos_temp.posid].length <= 20) {
        pos_pre.dist = pos_temp.dist;
        pos_pre.posid = pos_temp.posid;
        pos_pre.time_inter = 0;
      }
      if (pos_temp.dist <= PASSWIDTH_LOWER_DY &&
          pos_temp_last.dist > PASSWIDTH_LOWER_DY &&
          pos_temp.dist != INVALIDDOUBLE)  // TBD  or use PASSWIDTH_LOWER
      {
        pos_in.dist = pos_temp.dist;
        pos_in.posid = pos_temp.posid;
        pos_in.time_inter = (j + 1) * objpred.time_interval;
      }
      if (pos_temp.dist > PASSWIDTH_LOWER_DY &&
          pos_temp_last.dist <=
              PASSWIDTH_LOWER_DY)  //&&pos_temp.dist!=INVALIDDOUBLE)//TBD  or
                                   // use PASSWIDTH_LOWER
      {
        pos_out.dist = pos_temp_last.dist;
        pos_out.posid = pos_temp_last.posid;
        pos_out.time_inter = (j + 1) * objpred.time_interval;
      }
      pos_temp_last = pos_temp;
    }
  }
#endif
#if 0
  if(pathpoints[pos_pre.posid].length<=15)
  {
      float32 velocity_obs = velocity_sobj(pos_pre.dist,pathpoints[pos_pre.posid].pathpoints_in.velocity);
      pathpoints[pos_pre.posid].pathpoints_in.velocity = velocity_obs;
         int posid_preobs = 0;
         for(int h = pos_temp.posid;h>0;h--)
         {
            double dist = pathpoints[pos_pre.posid].length - pathpoints[h].length;
            if(dist>=DIST_PREOBS_S)
            {
              posid_preobs = h;
              break;
            }
         }

        for(int k = posid_preobs;k<pos_pre.posid;k++)
        {
           if(pathpoints[k].pathpoints_in.velocity > pathpoints[pos_pre.posid].pathpoints_in.velocity)
           {
               pathpoints[k].pathpoints_in.velocity = pathpoints[pos_pre.posid].pathpoints_in.velocity;
           }
        }
        for(int j = 1;j<pathpoints.size();j++)
         {
             float32 diff_max = 0.01;
             float32 diff_velocity = pathpoints[j].pathpoints_in.velocity - pathpoints[j-1].pathpoints_in.velocity;
             if(diff_velocity > diff_max)
             {
               float32 velocity_temp = sqrt(pathpoints[j-1].pathpoints_in.velocity*pathpoints[j-1].pathpoints_in.velocity+2*Aplus_max*(pathpoints[j].length-pathpoints[j-1].length));
               if(pathpoints[j].pathpoints_in.velocity>velocity_temp)
               {
                  pathpoints[j].pathpoints_in.velocity = velocity_temp;
               }
             }
         }
         for(int j = pathpoints.size();j>0;j--)
         {
             float32 diff_max = 0.01;
             float32 diff_velocity = pathpoints[j-1].pathpoints_in.velocity - pathpoints[j].pathpoints_in.velocity;
             if(diff_velocity > diff_max)
             {
               float32 velocity_temp = sqrt(pathpoints[j].pathpoints_in.velocity*pathpoints[j].pathpoints_in.velocity-2*Aminus_max*(pathpoints[j].length-pathpoints[j-1].length));
               if(pathpoints[j-1].pathpoints_in.velocity>velocity_temp)
               {
                  pathpoints[j-1].pathpoints_in.velocity = velocity_temp;
               }
             }
         }
      
  }
#endif
// doubleVector T_collision_car;
// T_collision_car.clear();
#if 1
  float32 A_target_inside_min = ACCELERATION_INIT_WBD;
  float32 A_target = 0.0;
  int index_inside = 0;
  for (int32 i = 0; i < pos_inside.size(); i++) {
    double T_collision_car[200] = {0};
    for (int32 j = 1; j < pos_inside[i].posid; j++) {
      if (pathpoints[pos_inside[i].posid].length - pathpoints[j].length >
              Dsafe &&
          pathpoints[j].pathpoints_in.velocity > 0.1)  // 0.2
      {
        T_collision_car[i] = T_collision_car[i] +
                             (pathpoints[j].length - pathpoints[j - 1].length) /
                                 pathpoints[j].pathpoints_in.velocity;
      } else {
        // collision_car[i] = T_collision_car[i];
      }
    }
    // if(T_collision_car[i]<pos_inside[i].time_inter +
    // TSAFE_LANE_COLLISATION_WBD)
    if ((T_collision_car[i] <
         pos_inside[i].time_inter + TSAFE_LANE_COLLISATION_WBD) ||
        0) {
      float32 T_safe_inside =
          pos_inside[i].time_inter + TSAFE_LANE_COLLISATION_WBD;
      if (fabs(pathpoints[2].pathpoints_in.velocity - sbjspeed) >= 0.1 &&
          REALVELOCITY == 1 && 1) {
        A_target = 2 * (pathpoints[pos_inside[i].posid].length - Dsafe -
                        sbjspeed * T_safe_inside) /
                   (T_safe_inside * T_safe_inside);
        if (0 || (pathpoints[pos_inside[i].posid].length - Dsafe -
                          sbjspeed * T_safe_inside <
                      0 &&
                  (A_target > 0 || sbjspeed + A_target * T_safe_inside < 0))) {
          A_target = -0.5 * sbjspeed * sbjspeed /
                     (pathpoints[pos_inside[i].posid].length - Dsafe);
        }
      } else {
        A_target = 2 * (pathpoints[pos_inside[i].posid].length - Dsafe -
                        pathpoints[2].pathpoints_in.velocity * T_safe_inside) /
                   (T_safe_inside * T_safe_inside);
      }

      if (pathpoints[pos_inside[i].posid].length - Dsafe <= 0.0) {
        A_target = -2.0;
      }
      if (A_target_inside_min > A_target && A_target < 0)  //&&A_target<0)//0.7
      {
        A_target_inside_min = A_target;
        index_inside = pos_inside[i].posid;
      }
    }

    motionvars.Acceltarget = A_target_inside_min;
  }
  if (A_target_inside_min == ACCELERATION_INIT_WBD) {
    A_target_inside_min = 0.0;
  }
  if (index_inside > 1) {
    for (int32 k = 1; k < index_inside; k++) {
      double velocity_temp = 0.01;
      if (pathpoints[k - 1].pathpoints_in.velocity *
                  pathpoints[k - 1].pathpoints_in.velocity +
              2 * A_target_inside_min *
                  (pathpoints[k].length - pathpoints[k - 1].length) <
          0)  // pathpoints[k-1].pathpoints_in.velocity<=0.1||
      {
        velocity_temp = SPEED_STOP;
      } else {
        velocity_temp =
            sqrt(pathpoints[k - 1].pathpoints_in.velocity *
                     pathpoints[k - 1].pathpoints_in.velocity +
                 2 * A_target_inside_min *
                     (pathpoints[k].length - pathpoints[k - 1].length));
      }
      if (velocity_temp < 0.01) {
        velocity_temp = SPEED_STOP;
      }
      if (pathpoints[k].pathpoints_in.velocity > velocity_temp) {
        pathpoints[k].pathpoints_in.velocity = velocity_temp;
      }

      if (pathpoints[k].pathpoints_in.velocity <= 0.01 ||
          A_target_inside_min == -2) {
        pathpoints[k].pathpoints_in.velocity = SPEED_STOP;
        for (int i = k; i < 10; i++)  // TODO @yanbo ?????
        {
          pathpoints[i].pathpoints_in.velocity = SPEED_STOP;
        }
      }
      if (1 == static_obs_flag && 1 == isspeedlimit &&
          pathpoints[k].pathpoints_in.velocity < guidespeed_min &&
          pathpoints[k].pathpoints_in.velocity >= 0) {
        pathpoints[k].pathpoints_in.velocity = guidespeed_min;
      }
    }
    for (int32 m = index_inside; m < pathpoints.size() - 1; m++) {
      if (m <= (index_inside + 10))
        pathpoints[m + 1].pathpoints_in.velocity = 0;
      else {
        if (pathpoints[m + 1].pathpoints_in.velocity -
                pathpoints[m].pathpoints_in.velocity >
            0.01) {
          pathpoints[m + 1].pathpoints_in.velocity =
              sqrt(pathpoints[m].pathpoints_in.velocity *
                       pathpoints[m].pathpoints_in.velocity +
                   2 * Aplus_max *
                       (pathpoints[m + 1].length - pathpoints[m].length));
        }
      }
    }
  }
#endif

#if 1
  float32 T_crossin_car = 0.0;
  float32 T_crossout_car = 0.0;
  for (int32 i = 1; i < pos_in.posid; i++) {
    T_crossin_car = T_crossin_car +
                    (pathpoints[i].length - pathpoints[i - 1].length) /
                        pathpoints[i].pathpoints_in.velocity;
  }

  for (int32 j = 1; j < pos_out.posid; j++) {
    if (pathpoints[pos_out.posid].length - pathpoints[j].length > Dsafe &&
        pathpoints[j].pathpoints_in.velocity > 0.1) {
      T_crossout_car = T_crossout_car +
                       (pathpoints[j].length - pathpoints[j - 1].length) /
                           pathpoints[j].pathpoints_in.velocity;
    }
  }

  if (pos_in.time_inter != 88.0 && (T_crossin_car < pos_in.time_inter - 2.0) &&
      pos_in.time_inter > TSAFE_BEF_COLLISATION_WBD) {
    if ((T_crossin_car > pos_in.time_inter - TSAFE_BEF_COLLISATION_WBD) &&
        T_crossin_car > 2.0) {
      Tsafe_bef_c = pos_in.time_inter - TSAFE_BEF_COLLISATION_WBD;
    } else {
      Tsafe_bef_c = 88.0;
    }
  }
  // if(pos_out.time_inter!=88&&T_crossout_car >
  // pos_out.time_inter&&T_crossout_car>0)
  if (pos_out.time_inter != 88.0 && Tsafe_bef_c == 88.0 &&
      T_crossout_car >= 0.0) {
    if (T_crossout_car < pos_out.time_inter + TSAFE_AFT_COLLISATION_WBD) {
      Tsafe_aft_c = pos_out.time_inter + TSAFE_AFT_COLLISATION_WBD;
    } else {
      Tsafe_aft_c = 88.0;
    }
  }
  float Aminus_target = 0;
  float Aplus_target = 0;
  Tsafe_aft_c = 88.0;
  if (Tsafe_bef_c != 88.0) {
    if (fabs(pathpoints[2].pathpoints_in.velocity - sbjspeed) >= 0.1 &&
        REALVELOCITY == 1) {
      Aplus_target = 2 * (pathpoints[pos_in.posid].length + objpred.length -
                          sbjspeed * Tsafe_bef_c) /
                     (Tsafe_bef_c * Tsafe_bef_c);
    } else {
      Aplus_target = 2 * (pathpoints[pos_in.posid].length + objpred.length -
                          pathpoints[2].pathpoints_in.velocity * Tsafe_bef_c) /
                     (Tsafe_bef_c * Tsafe_bef_c);
    }
    if (Aplus_target > 0 && Aplus_target < 0.2)  // param_Set;
    {
      for (int32 i = 1; i < pos_in.posid; i++) {
        double velocity_temp =
            sqrt(pathpoints[i - 1].pathpoints_in.velocity *
                     pathpoints[i - 1].pathpoints_in.velocity +
                 2 * Aplus_target *
                     (pathpoints[i].length - pathpoints[i - 1].length));

        if (pathpoints[i].pathpoints_in.velocity < velocity_temp) {
          pathpoints[i].pathpoints_in.velocity = velocity_temp;
        }
        // pathpoints[i].pathpoints_in.velocity = (1 +
        // 0.25)*pathpoints[i].pathpoints_in.velocity;
      }
      for (int32 j = pos_in.posid; j < pathpoints.size() - 1; j++) {
        if (pathpoints[j].pathpoints_in.velocity -
                pathpoints[j + 1].pathpoints_in.velocity >
            0.01) {
          pathpoints[j + 1].pathpoints_in.velocity =
              sqrt(pathpoints[j].pathpoints_in.velocity *
                       pathpoints[j].pathpoints_in.velocity +
                   2 * Aminus_max *
                       (pathpoints[j + 1].length - pathpoints[j].length));
        }
      }
    }
  }
  if (Tsafe_aft_c != 88.0) {
    if (fabs(pathpoints[2].pathpoints_in.velocity - sbjspeed) >= 0.1 &&
            REALVELOCITY == 1 ||
        1) {
      Aminus_target = 2 * (pathpoints[pos_out.posid].length - Dsafe -
                           sbjspeed * Tsafe_aft_c) /
                      (Tsafe_aft_c * Tsafe_aft_c);
      if (0 ||
          (pathpoints[pos_out.posid].length - Dsafe - sbjspeed * Tsafe_aft_c <
               0 &&
           (Aminus_target > 0 || sbjspeed + Aminus_target * Tsafe_aft_c < 0))) {
        Aminus_target = -0.5 * sbjspeed * sbjspeed /
                        (pathpoints[pos_out.posid].length - Dsafe);
      }
    } else {
      Aminus_target = 2 * (pathpoints[pos_out.posid].length - Dsafe -
                           pathpoints[2].pathpoints_in.velocity * Tsafe_aft_c) /
                      (Tsafe_aft_c * Tsafe_aft_c);  // sbjspeed
    }
    if (pathpoints[pos_out.posid].length - Dsafe < 0.0) {
      Aminus_target = -2.0;
    }

    if (Aminus_target < 0) {
      for (int32 i = 1; i < pos_out.posid; i++) {
        double velocity_temp =
            sqrt(pathpoints[i - 1].pathpoints_in.velocity *
                     pathpoints[i - 1].pathpoints_in.velocity +
                 2 * Aminus_target *
                     (pathpoints[i].length - pathpoints[i - 1].length));
        if (pathpoints[i].pathpoints_in.velocity > velocity_temp) {
          pathpoints[i].pathpoints_in.velocity = velocity_temp;
        }
        // pathpoints[i].pathpoints_in.velocity = (1 +
        // (T_crossout_car-Tsafe_aft_c)/T_crossout_car)*pathpoints[i].pathpoints_in.velocity;
        if (pathpoints[i].pathpoints_in.velocity <= 0.01 ||
            Aminus_target == -2) {
          pathpoints[i].pathpoints_in.velocity = SPEED_STOP;
        }

        if (1 == static_obs_flag && 1 == isspeedlimit &&
            pathpoints[i].pathpoints_in.velocity < guidespeed_min &&
            pathpoints[i].pathpoints_in.velocity >= 0.0) {
          pathpoints[i].pathpoints_in.velocity = guidespeed_min;
        }
      }
      for (int32 j = pos_out.posid; j < pathpoints.size() - 1; j++) {
        if (pathpoints[j].pathpoints_in.velocity == SPEED_STOP) {
          for (int i = 0; i < 10; i++) {
            pathpoints[j + i].pathpoints_in.velocity = SPEED_STOP;
          }
        }
        if (pathpoints[j + 1].pathpoints_in.velocity -
                pathpoints[j].pathpoints_in.velocity >
            0.01) {
          pathpoints[j + 1].pathpoints_in.velocity =
              sqrt(pathpoints[j].pathpoints_in.velocity *
                       pathpoints[j].pathpoints_in.velocity +
                   2 * Aplus_max *
                       (pathpoints[j + 1].length - pathpoints[j].length));
        }
      }

      // for(int32 n = 1;n<pathpoints.size()-1;n++)
      // {
      //   if(pathpoints[n].pathpoints_in.velocity ==SPEED_STOP)
      //   {
      //     pathpoints[n+1].pathpoints_in.velocity = SPEED_STOP;
      //   }
      // }
    }
  }

  // double objpos_dist_thrd = 2;
  double dynamic_obs_disp = 0.0;
  // if((objpos_dist_min>PASSWIDTH_UPPER||(objpos_dist_min0>PASSWIDTH_LOWER_DY&&pathpoints[1].pathpoints_in.velocity
  // <= guidespeed_min))&&Tsafe_aft_c==88&&Tsafe_bef_c==88)//
  if ((objpos_dist_min > PASSWIDTH_LOWER ||
       ((objpos_dist_min0 >= PASSWIDTH_LOWER ||
         (relspeed_x > 0.3 && objpos_length_min0 >= Dsafe)) &&
        pathpoints[1].pathpoints_in.velocity <= guidespeed_min)))  //
  {
    dynamic_obs_disp = 1;
  } else {
    dynamic_obs_disp = 0;
  }
  Distl_obs_min = objpos_length_min0;
  Distw_obs_min = objpos_dist_min0;

  if (Distl_obs_min <= 3.5) {
    // emergency_flag = 1;
  } else if (Distl_obs_min > 4.5 && emergency_flag == 1) {
    // emergency_flag = 0;
  } else {
  }
  if (emergency_decision == 1) {
    emergency_flag = 1;
  } else {
    emergency_flag = 0;
  }
// motionvars.relspeed_x = relspeed_x;
// motionvars.Dsafe = Dsafe;
// motionvars.Distw_obs_min = objpos_dist_min0;
#endif
  return dynamic_obs_disp;
}

doubleVector motionplannerWBD::getCoeff(vector<pt> sample, int n) {
  vector<doubleVector> matFunX;  //矩阵方程
  vector<doubleVector> matFunY;  //矩阵方程
  doubleVector temp;
  double sum;
  int i, j, k;

  //正规方程X
  for (i = 0; i <= n; i++) {
    temp.clear();
    for (j = 0; j <= n; j++) {
      sum = 0;
      for (k = 0; k < sample.size(); k++) sum += pow(sample[k].x, j + i);
      temp.push_back(sum);
    }
    matFunX.push_back(temp);
  }

  //正规方程Y
  for (i = 0; i <= n; i++) {
    temp.clear();
    sum = 0;
    for (k = 0; k < sample.size(); k++)
      sum += sample[k].y * pow(sample[k].x, i);
    temp.push_back(sum);
    matFunY.push_back(temp);
  }

  //矩阵行列式变换
  double num1, num2, ratio;
  for (i = 0; i < matFunX.size() - 1; i++) {
    num1 = matFunX[i][i];
    for (j = i + 1; j < matFunX.size(); j++) {
      num2 = matFunX[j][i];
      ratio = num2 / num1;
      for (k = 0; k < matFunX.size(); k++)
        matFunX[j][k] = matFunX[j][k] - matFunX[i][k] * ratio;
      matFunY[j][0] = matFunY[j][0] - matFunY[i][0] * ratio;
    }
  }

  //计算拟合曲线的系数
  doubleVector coeff(matFunX.size(), 0);
  for (i = matFunX.size() - 1; i >= 0; i--) {
    if (i == matFunX.size() - 1)
      coeff[i] = matFunY[i][0] / matFunX[i][i];
    else {
      for (j = i + 1; j < matFunX.size(); j++)
        matFunY[i][0] = matFunY[i][0] - coeff[j] * matFunX[i][j];
      coeff[i] = matFunY[i][0] / matFunX[i][i];
    }
  }
  return coeff;
}

int32 motionplannerWBD::PathSelect(
    std::vector<std::vector<sPoint>> option_pathpoints) {
  if (option_pathpoints.size() == 0) {
    return 1;
  } else {
#if 1
    doubleVector Priority_rating = {0};
    doubleVector Length_cost = {0};
    doubleVector T_cost = {0};
    doubleVector Curvature_cost = {0};
    doubleVector Curvature_max = {88.88};
    doubleVector Velocity_cost = {0};  //
    double Gain_L = 1;
    double Gain_T = 1;
    double Gain_Curv = 2;
    double Gain_V = 3;
    double Curv_max = 0.5;
    double Priority_rating_max = 0;

    for (int i = 0; i < option_pathpoints.size(); i++) {
      for (int j = 1; j < option_pathpoints[i].size(); j++) {
        T_cost[i] += (option_pathpoints[i][j].length -
                      option_pathpoints[i][j - 1].length) /
                     option_pathpoints[i][j].pathpoints_in.velocity;
        Curvature_cost[i] += option_pathpoints[i][j].curvature;
        if (Curvature_max[i] > option_pathpoints[i][j].curvature) {
          Curvature_max[i] = option_pathpoints[i][j].curvature;
        }
        Velocity_cost[i] +=
            fabs(option_pathpoints[i][j].pathpoints_in.velocity -
                 option_pathpoints[i][j - 1].pathpoints_in.velocity);
        Length_cost[i] = option_pathpoints[i][j].length;
      }
      Priority_rating[i] = Gain_L * Length_cost[i] + Gain_T * T_cost[i] +
                           Gain_Curv * Curvature_cost[i] +
                           Gain_V * Velocity_cost[i];
      if (Curvature_max[i] >= Curv_max) {
        Priority_rating[i] = 0;
      }
      if (Priority_rating_max < Priority_rating[i]) {
        Priority_rating_max = Priority_rating[i];
        PathID = i;
      }
      PathID = i;
    }
#endif
    return PathID;
  }
}

void motionplannerWBD::curvature_cal(
    std::vector<std::vector<sPoint>> &option_pathpoints) {
  for (int i = 0; i < option_pathpoints.size(); i++) {
    for (int j = 1; j < option_pathpoints[i].size() - 1; j++) {
      double k1 = 0;
      double k2 = 0;
      double angle = 0;
      double arc = 0;
      if (option_pathpoints[i].size() < 10) {
        option_pathpoints[i][j].curvature = 0;
      } else {
        k1 = (option_pathpoints[i][j - 1].pathpoints_in.x -
              option_pathpoints[i][j].pathpoints_in.x) /
             (option_pathpoints[i][j].pathpoints_in.y -
              option_pathpoints[i][j - 1].pathpoints_in.y);
        k2 = (option_pathpoints[i][j].pathpoints_in.x -
              option_pathpoints[i][j + 1].pathpoints_in.x) /
             (option_pathpoints[i][j + 1].pathpoints_in.y -
              option_pathpoints[i][j].pathpoints_in.y);
        angle = atan(fabs(k2 - k1) / (1 + k1 * k2));
        arc = 0.5 * (option_pathpoints[i][j + 1].length -
                     option_pathpoints[i][j - 1].length);
        option_pathpoints[i][j].curvature = sin(angle) / arc;
      }
    }
    option_pathpoints[i][0].curvature = option_pathpoints[i][1].curvature;
    // option_pathpoints[i][option_pathpoints[i].size()].curvature =
    // option_pathpoints[i][option_pathpoints[i].size()-1].curvature;
  }
}

double motionplannerWBD::dist_get(double x0, double y0, double x1, double y1) {
  double dist_temp = sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));

  return dist_temp;
}

void motionplannerWBD::Basesignals_get(ivmap::ivmapmsgvap car_status) {
  sbjspeed = car_status.v;
  // motionvars.relspeed_x = relspeed_x;
  // TODO @yanbo test 1121
  /* if(sbjspeed > 1.99) {
     sbjspeed = 1.99;
   }
   if(sbjspeed < -0.99) {
     sbjspeed = -0.99;
   }
 motionvars.speed_test = sbjspeed;*/
}

void motionplannerWBD::Decision_get(ivdecision::ivmsgdecision decision,
                                    bool path_passable, bool emergency_f,
                                    bool avoid_flag) {
  decisions = decision;
  // lidar_seg_update = lidarP_change;
  isspeedlimit = path_passable;
  if (emergency_f != emergency_decision) {
    speedupdate = 1;
  }
  emergency_decision = emergency_f;
  avoid_path = avoid_flag;
  // if(lidar_seg_update==1)
  // {

  // }
}

bool motionplannerWBD::cautious_path(bool avoid_status,
                                     std::vector<sPoint> pathpoints) {
  bool caution_path_status = 0;
  int path_length = 8;
  int Index_max = 1;
  double curvature_mean = 0;
  double curvature_max_thred = 0.2;
  double curvature_mean_thred_lower = 0.05;
  double curvature_mean_thred_upper = 0.1;
  double curvature_max = 0;
  if (pathpoints.size() > 1) {
    for (int i = 0; pathpoints[i].length < path_length; i++) {
      Index_max = i + 1;
      curvature_mean += pathpoints[i].curvature;
      if (curvature_max < pathpoints[i].curvature) {
        curvature_max = pathpoints[i].curvature;
      }
    }
    curvature_mean /= Index_max;
    if (curvature_max >= curvature_max_thred) {
      caution_path_status = true;
    } else if (curvature_mean >= curvature_mean_thred_upper) {
      caution_path_status = true;
    } else if (avoid_status && curvature_mean >= curvature_mean_thred_lower) {
      caution_path_status = true;
    } else {
      caution_path_status = false;
    }
  } else {
    caution_path_status = false;
  }

  return caution_path_status;
}
