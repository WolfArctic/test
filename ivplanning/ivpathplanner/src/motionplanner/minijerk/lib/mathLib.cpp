#include "mathLib.h"

float constrain_float(const float Max,const float Min,float value)
{
	if(Max<Min)
	{
		//cout<<"Oop:constrain_float is used under wrong way."<<endl;
		return value;
	}
	if(value>Max)
		value = Max;
	if(value<Min)
		value = Min;
	return value;
}

VectorXd polyfit(VectorXd xvals, VectorXd yvals, int order) 
{
  MatrixXd A(xvals.size(), order + 1);
  MatrixXd Q;
  VectorXd result;
  for (int i = 0; i < xvals.size(); i++) 
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) 
  {
    for (int i = 0; i < order; i++) 
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  result = A.householderQr().solve(yvals);;

  return  result;
}