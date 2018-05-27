
#include "pid.h"
#include "mathLib.h"


float pid_get_p(PID_Typedef* pid,float error)
{
    return (pid->kp * error);
}

float pid_get_i(PID_Typedef* pid,float error,float dt)
{
    if(pid->ki>0 && dt>0)
        pid->integ += (pid->ki * error * dt);
    
    if(pid->integ > pid->integ_max) pid->integ = pid->integ_max;
    if(pid->integ < -pid->integ_max)pid->integ = -pid->integ_max;
    
    return (pid->integ);
}

float pid_get_d(PID_Typedef* pid,float error,float dt)
{
    float new_deriv = 0;
    pid->error = error;
    if(pid->kd>0 && dt>0+1e-6)
	{
		if(isnan(pid->deriv))
		{
			pid->deriv = 0;
			new_deriv = 0;
		}
		else
		{
			new_deriv = (pid->error - pid->last_error) / dt;
		}
		pid->deriv = pid->deriv + pid->alpha * (new_deriv - pid->deriv);	

        pid->last_error = pid->error;
        
        return (pid->kd * pid->deriv);
	}
    return 0;
}

void pid_reset_i(PID_Typedef* pid)
{
	pid->integ = 0;
}

float get_integrator(PID_Typedef* pid) 
{ 
    return pid->integ; 
}

void set_integrator(PID_Typedef* pid, float value) 
{ 
	pid->integ = value; 
}

void set_d_lpf_alpha(PID_Typedef* pid, float fcut, float dt)
{
	float rc_d = 1 / (2 * M_PI * fcut);
    
	pid->alpha = dt / (rc_d + dt);
    
    pid->d_lpf_fcut = fcut;
}
void inte_param_init(inte_param* integrator_param,double dt)
{
	integrator_param->inte = 0;
	integrator_param->dt = dt;
}
double Integrator(inte_param* integrator_param,double new_data)
{
	integrator_param->inte = integrator_param->inte + new_data*integrator_param->dt;
	return integrator_param->inte;
}
/*
*********************************************************************************************************
--------------------------------------------- END OF FILE ------------------ pid.c ----------------------
*********************************************************************************************************
*/
