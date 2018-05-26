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