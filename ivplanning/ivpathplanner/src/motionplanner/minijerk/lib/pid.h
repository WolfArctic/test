/
#ifndef _PID_H_
#define _PID_H_

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/

/* PID½á¹¹Ìå */
typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
	float last_error;
	float integ;
	float integ_max;
	float deriv;
	float d_lpf_fcut;
	float alpha;
    float output;
 
}PID_Typedef;
typedef struct 
{
	double inte;
	double dt;
}inte_param;
/*
*********************************************************************************************************
*                                        	FUNCTION DEFINITION
*********************************************************************************************************
*/

float pid_get_p(PID_Typedef* pid,float error);

float pid_get_i(PID_Typedef* pid,float error,float dt);

float get_integrator(PID_Typedef* pid);

void set_integrator(PID_Typedef* pid, float value);

float pid_get_d(PID_Typedef* pid,float error,float dt);

void pid_reset_i(PID_Typedef* pid);

void set_d_lpf_alpha(PID_Typedef* pid,float fcut, float dt);
void inte_param_init(inte_param* integrator_param,double dt);
double Integrator(inte_param* integrator_param,double new_data);

#endif
/*
*********************************************************************************************************
--------------------------------------------- END OF FILE ------------------ ahrs.c ----------------------
*********************************************************************************************************
*/
