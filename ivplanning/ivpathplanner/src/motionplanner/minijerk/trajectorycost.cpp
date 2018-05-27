#include "trajectorycost.h"

double TimeDiffCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	double t = traj.t;
	double unperturbed_t = traj.unperturbed_t;

	return logistic(float(abs(t - unperturbed_t)) / unperturbed_t);

}

double VDiffCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	vector<double> s_dot_coeffs =differentiate(traj.s_coeff);
	vector<double> unperturbed_s_dot_coeffs =differentiate(traj.unperturb_s_coeff);

	double t = traj.t;
	double dt = t/100.0;
	double cur_t = 0;
	double total_diff_v = 0;

	for(int i = 0; i< 100; i++)
	{
		cur_t = dt * i;
		double actual_v = to_equation(s_dot_coeffs, cur_t);
		double expected_v = to_equation(unperturbed_s_dot_coeffs,cur_t);
		total_diff_v += float(abs(actual_v - expected_v));
	}

	double diff_v_per_second = total_diff_v / cur_t;

	//cout<<"cost diff_v_per_second = "<<diff_v_per_second<<endl;
	return logistic(diff_v_per_second / EXPECTED_DIFF_V_IN_ONE_SEC);
}


double TotalAccCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{

    vector<double> s_dot_coeffs =differentiate(traj.s_coeff);
    vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);

    double t = traj.t;
    double dt = t/100.0;
    double cur_t = 0;
    double total_acc = 0;
    
    for(int i = 0; i< 100; i++)
    {
    	cur_t = dt * i ;
    	double acc = to_equation(s_dot_dot_coeffs, cur_t);
    	total_acc += abs(acc*dt);
    }

    double acc_pre_second = total_acc / cur_t;

    //in order to make 
    // cout<<"cost acc_pre_second = "<<acc_pre_second<<endl;
    int index_follow_car = 1;
    auto behavior_state = std::get<0>(prediction);
    if(behavior_state == FOLLOW_CAR)
    	index_follow_car = 5;
    return index_follow_car * logistic(acc_pre_second / EXPECTED_ACC_IN_ONE_SEC);
}

double MaxAccCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);
	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
	double t = traj.t;

	vector<double> all_as = {};

	for(int i = 0; i<100; i++)
	{
		double cur_t = double(t)/100 *i;
		all_as.push_back(to_equation(s_dot_dot_coeffs,cur_t));
	}

    auto max_acc = std::max_element(std::begin(all_as), std::end(all_as));

    if(abs(*max_acc) > MAX_ACCEL)
    {
    	return 1;
    }

    else
    {
    	return 0;
    }
}

double TotalJerkCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{

    vector<double> s_dot_coeffs =differentiate(traj.s_coeff);
    vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
    vector<double> s_dot_dot_dot_coeffs = differentiate(s_dot_dot_dot_coeffs);

    double t = traj.t;
    double dt = t / 100.0;
    double cur_t = 0;
    double total_jerks = 0;
    
    for(int i = 0; i< 100; i++)
    {
    	cur_t = dt * i ;
    	double jerk = to_equation(s_dot_dot_dot_coeffs,cur_t);
    	total_jerks += abs(jerk*dt);
    }

    double jerk_pre_second = total_jerks / cur_t;
    return logistic(jerk_pre_second / EXPECTED_JERK_IN_ONE_SEC);
}

double MaxJerkCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	vector<double> s_dot_coeffs = differentiate(traj.s_coeff);
	vector<double> s_dot_dot_coeffs = differentiate(s_dot_coeffs);
	vector<double> s_dot_dot_dot_coeffs = differentiate(s_dot_dot_coeffs);

	double t = traj.t;

	vector<double> all_jerks = {};

	for(int i = 0; i<100; i++)
	{
		double cur_t = double(t)/100 *i;
		all_jerks.push_back(to_equation(s_dot_dot_dot_coeffs,cur_t));
	}

    auto max_jerk = std::max_element(std::begin(all_jerks), std::end(all_jerks));

    if(abs(*max_jerk) > MAX_JERK)
    {
    	return 1;
    }

    else
    {
    	return 0;
    }
}


double NearestApproachToVehicle(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	auto if_follow_car = std::get<0>(prediction);
	double closest = REALLY_BIG_NUMBER;

	if(if_follow_car)
	{

		auto front_car_speed = std::get<1>(prediction);
  		auto front_car_distance = std::get<2>(prediction);

		const vector<double> &s_coeffs = traj.s_coeff;
		double t = traj.t;

		for(int i =0; i< 100; i++)
		{
			double cur_t = float(i) / 100 * t;
			double cur_s = to_equation(s_coeffs,cur_t);

			double cur_other_vehicle_s = front_car_distance + front_car_speed * cur_t;

			double dist = sqrt(pow(cur_s - cur_other_vehicle_s,2));
			if(dist <closest)
			{
				closest = dist;
			}
		}
	}

	return closest;
}


double BufferCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	double nearest = NearestApproachToVehicle(traj,prediction,verbose);
	return logistic(2*VEHICLE_RADIUS/nearest);
}

double CollisionCost(const TrjObject &traj, const std::tuple<int,float,float> &prediction, bool verbose)
{
	double nearest = NearestApproachToVehicle(traj,prediction,verbose);

	if(nearest < 2*VEHICLE_RADIUS)
	{
		return 1;
	}
	else
		return 0;
}