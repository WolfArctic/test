#ifndef _VEHICLE_H
#define _VEHICLE_H
#pragma once
typedef enum
{
	STOP = 0,
	STABLE,
    ACCELERATE,
    DECELERATION,
} VehicleState_s;


typedef enum
{
	CONSPEED = 0,
    FOLLOW,
    FiXPARK,
} ControlMode_s;
typedef enum 
{
	BRAKING = 0,
	DRIVERING,
}DrivingMode_s;
struct LookUp
{
    //map<double,double> section;
    pair<double,double> speed_sec;
    pair<double,double> acc_sec_low;
    pair<double,double> acc_sec_high;

    bool is_lookup;
    bool speed_up_limit;
    bool speed_low_limit;
    bool acc_up_limit_low;
    bool acc_low_limit_low;
    bool acc_up_limit_high;
    bool acc_low_limit_high;

};
#endif 