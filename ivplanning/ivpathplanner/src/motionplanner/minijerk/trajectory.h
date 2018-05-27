#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_


#include<vector>
#include "helper.h"
#include "jmt.h"
#include <map>
#include <math.h>
#include <functional>
#include <algorithm>
#include <random>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>
#include "trajectorycost.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


typedef double (*CostFunPtr)(const TrjObject &traj,const std::tuple<int,float,float> &prediction, bool verbose);

class CostFuncWeight
{
public:
	CostFuncWeight()
	{

	}
	CostFuncWeight(CostFunPtr cost_func_p, double weight_p)
	{
		cost_func = cost_func_p;
		weight = weight_p;
	}
	CostFunPtr cost_func;
	double weight;

};



class Trajectory
{
public:
	Trajectory();
	virtual ~Trajectory();
	double CalculateCost(const TrjObject &trajectory, const std::tuple<int,float,float> &prediction, bool verbose = false);
	TrjObject ChooseBestTrajectory(const std::vector<double> &start_point, const std::vector<double> &goal_point, const double &delta,
											const std::tuple<int,float,float> &prediction);

private:

	std::map<std::string, CostFuncWeight> m_cost_map;
	std::vector<double> TrajectoryJMT(std::vector< double> start, std::vector <double> end, double T);
	TrjObject PTG(const vector<double> &start_point, std::vector<TrjObject > &all_trajs, const std::tuple<int,float,float> &prediction);
	std::vector<TrjObject> PerturbGoals(const std::vector<double> &start_point, const std::vector<double> &goal_point, const double &delta, 
												const std::tuple<int,float,float> &prediction);
};

#endif