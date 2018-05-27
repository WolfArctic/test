#include "filter.h"
#include "mathLib.h"

void lpf_1p_set_cutoff_freq(lpf_1p_param* param, double dt, double fcut)
{
    double rc_d = 1 / (2 * M_PI * fcut);
    
    param->alpha = dt / (rc_d + dt);
    
    param->last_value = 0;
}
double lpf_1p_apply(lpf_1p_param* param, double sample)
{
    double output =  param->last_value + param->alpha * (sample - param->last_value);
    
    param->last_value = output;
    
    return output;
}
void mean_filter_init(mean_filter_param *mf_param, int window_size)
{
    mf_param->window_size = window_size;
    for(int i=0;i<window_size;i++)
        mf_param->data.push_back(0);
}
double mean_filter_apply(mean_filter_param *mf_param, double in_data)
{
    double output = 0;
    std::vector<double> v;
    for(int i=0; i<mf_param->window_size-1; i++)
        mf_param->data[i] = mf_param->data[i+1];

    mf_param->data[mf_param->window_size-1] = in_data;
    v = double_sort(mf_param->data);
    for(int i = 1; i<v.size()-1; i++)
        output = v[i];
    output = output/(mf_param->window_size - 2);
    return output;
}


double mean_filter_nosort_apply(mean_filter_param *mf_param, double in_data)
{
    double output = 0;
    for(int i=0; i<mf_param->window_size-1; i++)
        mf_param->data[i] = mf_param->data[i+1];
    mf_param->data[mf_param->window_size-1] = in_data;
    for(int i =0; i<mf_param->window_size;i++)
        output += mf_param->data[i];
    output = output/mf_param->window_size ;
    return output;
}

std::vector<double> double_sort(std::vector<double> data)
{
    for(int i=0; i<data.size(); i++)
        for(int j=data.size()-1; j>i; j--)
            if(data[j]<data[j-1])
            {
                double temp = data[j-1];
                data[j-1] = data[j];
                data[j] = temp;
            }
    return data;
}