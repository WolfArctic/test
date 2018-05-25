#include "wxbsteercontrol.h"

 wxbsteercontrol::wxbsteercontrol(ros::NodeHandle mh)
 {
	sub_ = mh.subscribe("ivmotionplanner", 1000, &wxbsteercontrol::SubMotionplanner,this);
	sub2_ = mh.subscribe("ivmapvap", 1000, &wxbsteercontrol::SubVAP,this);
	pub_ = mh.advertise<ivsteercontrol::ivsteercontrol>("ivsteercontrol", 1000);
	Initial();    
 }

wxbsteercontrol::~wxbsteercontrol()
{
    fclose(fp1);
}

void wxbsteercontrol::Init(ros::NodeHandle mh) 
{
    InitFilter1();
    InitFilter2();
}

void wxbsteercontrol::Initial()
{
	ros::NodeHandle mh;
	amount_motion_path_point = 0;
	sc_loop_freq = 0;
	i_speed_count = 0;
	i_steer_count = 0;
	car_speed_last = 0;
	eps_angle_last = 0;

	actuator_steerangle = 0;
	actuator_speed = 0;
	
	fp1=fopen("/home/nvidia/test_data.txt","w");
	memset(&target_angle_record,0,sizeof(target_angle_record));
	mh.param("scloopfrep",sc_loop_freq,sc_loop_freq);
	mh.param("lengthWheelBase",length_vehicle,length_vehicle);
	mh.param("widthVehicle",width_vehicle,width_vehicle);
	mh.param("reduceRatioLift",k_ratio_l,k_ratio_l);
	mh.param("reduceRatioRight",k_ratio_r,k_ratio_r);
	mh.param("steerAngleMaxLift", steerangle_max_l,steerangle_max_l);
	mh.param("steerAngleMaxRight",steerangle_max_r,steerangle_max_r);
	mh.param("wheelCenter",vehicle_angle_center,vehicle_angle_center);
	mh.param("gpsAngleCenter",gps_angle_center,gps_angle_center);
	mh.param("torqueMax",torque_max,torque_max);
	mh.param("xFrontAxelMid",x_middle_front,x_middle_front);
	mh.param("yFrontAxelMid",y_middle_front,y_middle_front);

	memset(&resultcontrol, 0, sizeof(resultcontrol));	
}

void wxbsteercontrol::InitFilter1() 
{
    double window_size = 5;
    mean_filter_init(&actuator_steerangle_filter_param,window_size); 
 
 }

void wxbsteercontrol::InitFilter2() 
{
    double window_size = 5;
    mean_filter_init(&actuator_speed_filter_param,window_size); 

}

void wxbsteercontrol::run()
{
	ros::Rate rate(sc_loop_freq);
	while (ros::ok())
	{	
		ros::spinOnce();
		StartDataProcess();
		rate.sleep();
	}
}

void wxbsteercontrol::SubMotionplanner(const ivpathplanner::ivmsgpath::ConstPtr &msg)
{
	motionpath = *msg;
}

void wxbsteercontrol::SubVAP(const ivmap::ivmapmsgvap::ConstPtr &msg)
{
	actuator_steerangle = msg->uisteerangle;
	actuator_speed = msg->v;   
}

void wxbsteercontrol::StartDataProcess()
{
	amount_motion_path_point = motionpath.points.size();
	if (amount_motion_path_point > 3 && !motionpath.points[0].emergency_flag || 1)
	{	
	
		CalculateMain();
	}
	else
	{
		resultcontrol.targetangle = 0;
		resultcontrol.torque = torque_max;
		std::cout<<"Warnning : No road!!! "<<std::endl;
	}

	pub_.publish(resultcontrol);

}

void wxbsteercontrol::CalculateMain()
{
	//Get target road
	GetTargetPath();
	amount_target_path_point = target_path.x.size();
	//Get car position
	double mid_front_x  = x_middle_front;
	double mid_front_y  = y_middle_front;
	double left_rear_x  = x_middle_front - length_vehicle;
	double left_rear_y  = y_middle_front + width_vehicle*0.5;
	double right_rear_x = x_middle_front - length_vehicle;
	double right_rear_y = y_middle_front - width_vehicle*0.5;	
	double car_pos_x[3] = {mid_front_x, left_rear_x, right_rear_x};
	double car_pos_y[3] = {mid_front_y, left_rear_y, right_rear_y};
	double angle_center = vehicle_angle_center + gps_angle_center;
	//Get feedback of actuator
    double eps_deg = mean_filter_apply(&actuator_steerangle_filter_param, actuator_steerangle);
    double car_speed = mean_filter_apply(&actuator_speed_filter_param, actuator_speed);
	double wheel_rad = eps2wheel(eps_deg - vehicle_angle_center);
	car_speed_last = car_speed;
	eps_angle_last = eps_deg; 
    // Get the relative position of the car and the road
	double id_nearest_point = GetNearestPointNum(mid_front_x, mid_front_y);
	sTrackError track_err = GetTrackError(id_nearest_point, car_pos_x, car_pos_y);
	double heading_err = track_err.heading_err * M_PI / 180;
    double lateral_err = track_err.lateral_err;
    double k_coe = 0.1;
	double target_wheel_angle = GetTargetAngle(lateral_err, heading_err,
													 car_speed, k_coe);
    double target_eps_angle1 = wheel2eps(target_wheel_angle);
    double target_eps_angle2 = LimiteTargetAngle(target_eps_angle1);
    double target_eps_angle3 = MeanFilter(target_eps_angle2);
    double target_eps_angle  = LimiteTargetAngle(target_eps_angle3);
    resultcontrol.targetangle = target_eps_angle + angle_center;
	resultcontrol.torque = torque_max;
    
    if (NULL==fp1)
	{
	   printf("open file error\n");
	}
     else
     {
       fprintf(fp1,"heading_err :%.4lf, lateral_err :%.4lf,resultcontrol.targetangle :%.4lf,actuator_speed :%.4lf,actuator_steerangle :%.4lf\n",heading_err,lateral_err,resultcontrol.targetangle,actuator_speed,actuator_steerangle);
     } 
}

double wxbsteercontrol::LimiteTargetAngle(double original_angle)
{
	if (original_angle > steerangle_max_r)
	{
		return (steerangle_max_r);
	}
	else if (original_angle < -steerangle_max_l)
	{
		return (-steerangle_max_l);
	}
	else 
	{
		return (original_angle);
	}
}

double wxbsteercontrol::MeanFilter(double original_angle)
{
	double angle_mean = 0;
	int mean_window = 10;
	for(int i=mean_window-1; i>0; --i)
	{
		target_angle_record[i] = target_angle_record[i-1];
		angle_mean += target_angle_record[i];
	}
	target_angle_record[0] = original_angle;
	angle_mean += target_angle_record[0];
	angle_mean = angle_mean / mean_window;
	return angle_mean;
}
double wxbsteercontrol::GetTargetAngle(double lateral_error, 
											 double heading_error, 
											 double car_speed, 
											 double k_coe)
{
	double target_wheel_angle = 0;
	double head_part = heading_error;
	double late_part = 0; 
   if (fabs(lateral_error) < MIN)
	{
		late_part = 0;
    }
	else if (car_speed*MAX < fabs(k_coe*lateral_error))
	{
		late_part = Signn(lateral_error) * M_PI * 0.5;
    } 
	else if (fabs(k_coe*lateral_error)*MAX < car_speed)
	{
		late_part = 0;
    }
	else
	{
		late_part = atan2(k_coe*lateral_error, car_speed);
	}

	return (head_part + late_part);
}

void wxbsteercontrol::GetTargetPath()
{
	target_path = GetOriginalPath();
	target_path = GetSmoothPath(target_path);
}

sPath wxbsteercontrol::GetOriginalPath() const
{
	sPath original_path;
	int amount_original_path_point = amount_motion_path_point;
	original_path.x        = VectorXd(amount_original_path_point);
	original_path.y        = VectorXd(amount_original_path_point);
	original_path.angle    = VectorXd(amount_original_path_point);
	original_path.distance = VectorXd(amount_original_path_point);

	for (int i=0; i<amount_original_path_point; ++i)
	{
		original_path.x(i)        = motionpath.points[i].x;
		original_path.y(i)        = motionpath.points[i].y;
		original_path.angle(i)    = motionpath.points[i].angle;
		original_path.distance(i) = 0;
	}
	return original_path;
}

sPath wxbsteercontrol::GetSmoothPath(sPath &original_path) const
{
	sPath smooth_path;
	int amount_original_path_point = original_path.x.size();
	int smooth_window = 8;

	if (amount_original_path_point <= smooth_window)
	{
		return original_path;
	}

	smooth_path.x         = VectorXd(amount_original_path_point);
	smooth_path.y         = VectorXd(amount_original_path_point);
	smooth_path.angle     = VectorXd(amount_original_path_point);
	smooth_path.distance  = VectorXd(amount_original_path_point);

	double xpointsum = 0;
	double ypointsum = 0;
	double roaddirectionmin = 0;
	double roaddirectionmax = 0;
	double roaddirectionsum = 0;
	for (int ismooth=0; ismooth<amount_original_path_point; ++ismooth)
	{
		if (ismooth <= smooth_window)
		{
			xpointsum = 0;
			ypointsum = 0;
			for (int i=0; i<=2*ismooth; ++i)
			{
				xpointsum = xpointsum + original_path.x(i);
				ypointsum = ypointsum + original_path.y(i);
			}
			smooth_path.x(ismooth) = xpointsum / (2*ismooth+1);
			smooth_path.y(ismooth) = ypointsum / (2*ismooth+1);
			smooth_path.angle(ismooth) = original_path.angle(ismooth);
		}

		else if (ismooth >= amount_original_path_point - smooth_window)
		{
			smooth_path.x(ismooth) = original_path.x(ismooth);
			smooth_path.y(ismooth) = original_path.y(ismooth);
			smooth_path.angle(ismooth) = original_path.angle(ismooth);			
		}

		else
		{
			xpointsum = 0;
			ypointsum = 0;
			for (int i = 0; i <= 2 * smooth_window; ++i)
			{
				xpointsum = xpointsum + original_path.x(ismooth+smooth_window-i);
				ypointsum = ypointsum + original_path.y(ismooth+smooth_window-i);
			}
			smooth_path.x(ismooth) = xpointsum / (2*smooth_window+1);
			smooth_path.y(ismooth) = ypointsum / (2*smooth_window+1);

			roaddirectionmin = original_path.angle(ismooth);
			roaddirectionmax = original_path.angle(ismooth);
			for (int i = 0; i <= 2 * smooth_window; ++i)
			{
				if (original_path.angle[ismooth+smooth_window-i] < roaddirectionmin)
				{
					roaddirectionmin = original_path.angle[ismooth+smooth_window-i];
				}
				if (original_path.angle[ismooth+smooth_window-i] > roaddirectionmax)
				{
					roaddirectionmax = original_path.angle[ismooth+smooth_window-i];
				}
			}

			if ((roaddirectionmax - roaddirectionmin) > 180)
			{
				smooth_path.angle(ismooth) = original_path.angle(ismooth);
			}
			else
			{
				roaddirectionsum = 0;
				for (int i=0; i<=2*smooth_window; ++i)
				{
					roaddirectionsum = roaddirectionsum + original_path.angle(ismooth+smooth_window-i);
				}
				smooth_path.angle(ismooth) = roaddirectionsum / (2*smooth_window+1);
			}
		}
		smooth_path.distance(ismooth) = 0;
	}

	return smooth_path;	
}

int wxbsteercontrol::Signn(double value)
{
    int value_sign = 1;
    if (value >= 0)
    {
        value_sign = 1;
    }
    else
    {
        value_sign = -1;
    }
    return value_sign;
}

int wxbsteercontrol::GetNearestPointNum(double x_point, double y_point)
{
	double distance_to_car_min = 1000;
	double distance_point_to_vehicle = 100;
	int id_nearest_point = 1;
	for (int i=0; i<amount_target_path_point; i++)
	{
		distance_point_to_vehicle = sqrt(pow((target_path.x(i) - x_point),2) + 
										 pow((target_path.y(i) - y_point),2));
		if (distance_point_to_vehicle > distance_to_car_min + 5)
		{
			break;
		}
		if (distance_point_to_vehicle < distance_to_car_min)
		{
			distance_to_car_min = distance_point_to_vehicle;
			id_nearest_point = i;
		}
	}
	return id_nearest_point;
}

sTrackError wxbsteercontrol::GetTrackError(int id_nearest_point, double car_x[], double car_y[])
{
	double lateral_err = 0;
	double heading_err = 0; 
	double lateral_dis = 0;

	double xfind = target_path.x(id_nearest_point);
	double yfind = target_path.y(id_nearest_point);
	sTrackError track_error;
	if (0 == id_nearest_point)
	{
		lateral_err = GetDist2Seg(car_x[0], car_y[0], 
								  target_path.x(0), target_path.y(0), 
								  target_path.x(1), target_path.y(1));
		heading_err = target_path.angle(0);
	}
	else if (amount_target_path_point-1 == id_nearest_point)
	{
		lateral_err = GetDist2Seg(car_x[0], car_y[0], 
							      target_path.x(id_nearest_point), 
							      target_path.y(id_nearest_point), 
							      target_path.x(id_nearest_point-1), 
							      target_path.y(id_nearest_point-1));
		heading_err = target_path.angle(amount_target_path_point-1);
	} 
	else
	{
		lateral_dis = GetDist2Seg(car_x[0], car_y[0], 
							      target_path.x(id_nearest_point), 
							      target_path.y(id_nearest_point), 
							      target_path.x(id_nearest_point-1), 
							      target_path.y(id_nearest_point-1));

		lateral_err = lateral_dis;
		heading_err = target_path.angle(id_nearest_point);

		lateral_dis = GetDist2Seg(car_x[0], car_y[0], 
							      target_path.x(id_nearest_point), 
							      target_path.y(id_nearest_point), 
							      target_path.x(id_nearest_point+1), 
							      target_path.y(id_nearest_point+1));	
		if (lateral_dis < lateral_err)
		{
			lateral_err = lateral_dis;
			heading_err = target_path.angle(id_nearest_point+1);
		}				 			   	
	}
	double left_err =  GetLength(car_x[1], car_y[1], 
								 target_path.x(id_nearest_point), 
								 target_path.y(id_nearest_point));
	double right_err = GetLength(car_x[2], car_y[2], 
		                         target_path.x(id_nearest_point), 
		                         target_path.y(id_nearest_point));

	if (left_err > right_err)
	{
		lateral_err = -lateral_err;
	}

	track_error.lateral_err = lateral_err;
	track_error.heading_err = heading_err;

	return track_error;
}


double wxbsteercontrol::wheel2eps(double wheel_angle)
{	
	if (wheel_angle < 0)
	{
		return (-1 * wheel_angle * k_ratio_r * 180.0 / M_PI);
	}
	else
	{
		return (-1 * wheel_angle * k_ratio_l * 180.0 / M_PI);
	}
}

double wxbsteercontrol::eps2wheel(double eps_angle)
{
	double wheel_rad = 0;
	if (eps_angle > 0)
	{
		wheel_rad = - (eps_angle-vehicle_angle_center) / k_ratio_r * M_PI / 180;		
	}
	else
	{
		wheel_rad = - (eps_angle-vehicle_angle_center) / k_ratio_l * M_PI / 180;		
	}
	return wheel_rad;
}

double wxbsteercontrol::GetLength(double x_point1, double y_point1, double x_point2, double y_point2)
{
	return (sqrt(pow(x_point1 - x_point2, 2) + pow(y_point1 - y_point2, 2)));
}

double wxbsteercontrol::GetDist2Seg(double x, double y, double x1, double y1, double x2, double y2)
{
	double cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
	if (cross <= 0) 
	{
		return (GetLength(x, y, x1, y1));
	}	

	double d2 = GetLength(x1, y1, x2, y2);
	if (cross >= d2 * d2) 
	{
		return (GetLength(x, y, x2, y2));
	}
	
	double r = cross / d2;
	double px = x1 + (x2 - x1) * r;
	double py = y1 + (y2 - y1) * r;
	double dist2seg = GetLength(x, y, px, py);
	return dist2seg;
}

