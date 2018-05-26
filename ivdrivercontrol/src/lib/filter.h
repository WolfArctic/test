#ifndef _FILTER_H
#define _FILTER_H
#pragma once
#include <vector>
typedef struct 
{
	double alpha;
	double last_value;
	
}lpf_1p_param;

typedef struct 
{
	std::vector<double> data;
	int window_size;	
}mean_filter_param;


void lpf_1p_set_cutoff_freq(lpf_1p_param* param, double dt, double fcut);
double lpf_1p_apply(lpf_1p_param* param, double sample);
void mean_filter_init(mean_filter_param *mf_param, int window_size);
double mean_filter_apply(mean_filter_param *mf_param, double in_data);
std::vector<double> double_sort(std::vector<double> data);


#endif