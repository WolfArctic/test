#include "trajectory.h"

/*	all parameters should modified to change the weights of each elements. 
	TimeDiffCost, VDiffCost perfer the first trajectory.
	MaxAccCost, MaxJerkCost, TotalJerkCost, TotalAccCost perfer the smooth path
	BufferCost and CollisionCost perfer the large distance from the front car.
*/
Trajectory::Trajectory() 
{
	m_cost_map["TimeDiffCost"] = CostFuncWeight(&TimeDiffCost, 10);
	m_cost_map["MaxAccCost"] = CostFuncWeight(&MaxAccCost, 8);
	m_cost_map["MaxJerkCost"] = CostFuncWeight(&MaxJerkCost, 8);
	m_cost_map["TotalJerkCost"] = CostFuncWeight(&TotalJerkCost, 8);
	m_cost_map["CollisionCost"] = CostFuncWeight(&CollisionCost, 100);
	m_cost_map["BufferCost"] = CostFuncWeight(&BufferCost, 10);
	m_cost_map["TotalAccCost"] = CostFuncWeight(&TotalAccCost, 5);
	m_cost_map["VDiffCost"] = CostFuncWeight(&VDiffCost, 10);
}

Trajectory::~Trajectory() 
{
	// TODO Auto-generated destructor stub
}


vector<double> Trajectory::TrajectoryJMT(vector< double> start, vector <double> end, double T)
{


	MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			3*T*T, 4*T*T*T,5*T*T*T*T,
			6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			end[1]-(start[1]+start[2]*T),
			end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
		result.push_back(C.data()[i]);
	}

	return result;

}

/******************************************************
 * Function: PerturbGoals()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: change the goal length to get different trajectories;
 * Input: start_point, goal_point, delta
 * Output: 
 * Return: all_trajs
 * Others: None
 *******************************************************/
std::vector<TrjObject> Trajectory::PerturbGoals(const std::vector<double> &start_point, const std::vector<double> &goal_point, const double &delta, 
									const std::tuple<int,float,float> &prediction)
{
	//auto if_follow_car = std::get<0>(behavior);  // 0 no car, 1 front car
	std::vector<TrjObject> all_trajs = {};
	double count = 0;
	double gap_s = 0;
	double perturbed_goal_v = goal_point[1];
	double unperturbed_time = 2*goal_point[0]/(start_point[1] + goal_point[1]);
	vector<double> unperturb_s_coeff = TrajectoryJMT(start_point, goal_point,unperturbed_time);

	while(gap_s <= 20)
	{
		double perturbed_goal_s = goal_point[0] + gap_s;
		double perturbed_time = 2*perturbed_goal_s/(start_point[1] + perturbed_goal_v);
		all_trajs.push_back(TrjObject({perturbed_goal_s, perturbed_goal_v, 0.0}, goal_point,unperturb_s_coeff, perturbed_time, unperturbed_time));
		gap_s += delta;
	}

	return all_trajs;

}

/*
 Finds the best trajectory according to WEIGHTED_COST_FUNCTIONS (global).
 */


TrjObject Trajectory::PTG(const vector<double> &start_point, std::vector<TrjObject > &all_trajs, const std::tuple<int,float,float> &prediction)
{
	double min_cost = REALLY_BIG_NUMBER;
	TrjObject best;

	for(auto &trj: all_trajs)
	{
		trj.s_coeff = TrajectoryJMT(start_point, trj.goal, trj.t);
		double cost = CalculateCost(trj, prediction);

		if(cost < min_cost)
		{
			best = trj;
			min_cost = cost;
		}
	}

	return best;
}

double Trajectory::CalculateCost(const TrjObject &trajectory, const std::tuple<int,float,float> &prediction, bool verbose)
{
	double cost = 0;
	for(auto & kv : m_cost_map)
	{
		auto cost_func_pair = kv.second;
		double new_cost = cost_func_pair.weight * cost_func_pair.cost_func(trajectory, prediction, verbose);
		cost += new_cost;
		//cout<< "cost for "<<kv.first << " is "<< new_cost << endl;

	}
		//cout<<"overall cost="<< cost<<"\n cost trj="<<trajectory.goal[0]<<endl;  // goal[0] is the local distance
	return cost;
}

/******************************************************
 * Function: UpdateBehavior()
 * Author: Sheng Fu
 * Date: 2018-3-1
 * Description: Determine the current status;
 * Input: optionPathPoints, objDynamic, current_curve
 * Output: 
 * Return: behavior_state
 * Others: None
 *******************************************************/
TrjObject Trajectory::ChooseBestTrajectory(const std::vector<double> &start_point, const std::vector<double> &goal_point, const double &delta,
												const std::tuple<int,float,float> &prediction)
{
	std::vector<TrjObject> all_goals = PerturbGoals(start_point, goal_point, delta, prediction);
	return PTG(start_point, all_goals, prediction);
}