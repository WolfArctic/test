#include "mathtool.h"

mathtool::mathtool()
{

}

mathtool::~mathtool()
{
	
}

void mathtool::initParam()
{
     //
}


/******************************************************
 * Function: GetDist()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Calculate the distance of two points;
 * Input: x0 ,y0, x1, y1
 * Output: None 
 * Return: distance 
 * Others: None
 *******************************************************/
double mathtool::GetDist(double x0, double y0, double x1, double y1) 
{
	double dist2Pts = sqrt((x0 - x1 ) * (x0 - x1) + (y0 - y1) * (y0 - y1)); 
	return dist2Pts;
}

/******************************************************
 * Function: GetCoeff()
 * Author: Bo Yan
 * Date: 2017-6-6
 * Description: Calculate the coefficient for the least square method ;
 * Input: sample points, order
 * Output: None
 * Return: velocity 
 * Others: None
 *******************************************************/
doubleVector mathtool::GetCoeff(const vector<sPt> &sample, int n) 
{
	vector<doubleVector> matFunX;  //矩阵方程
	vector<doubleVector> matFunY;  //矩阵方程
	doubleVector temp;
	double sum;
	int i, j, k;

	//正规方程X
	for (i=0; i<=n; i++) 
	{
		temp.clear();
		for (j=0; j<=n; j++) 
		{
			sum = 0;
			for(k=0; k<sample.size(); k++)
				sum += pow(sample[k].x, j+i);
			temp.push_back(sum);
		}
		matFunX.push_back(temp);
	}

	//正规方程Y
	for (i=0; i<=n; i++) 
	{
		temp.clear();
		sum = 0;
		for(k=0; k<sample.size(); k++)
			sum += sample[k].y*pow(sample[k].x, i);
		temp.push_back(sum);
		matFunY.push_back(temp);
	}

	//矩阵行列式变换
	double num1, num2, ratio;
	for (i=0; i<matFunX.size()-1; i++) 
	{
		num1 = matFunX[i][i];
		for (j=i+1; j<matFunX.size(); j++) 
		{
			num2 = matFunX[j][i];
			ratio = num2/num1;
			for (k=0; k<matFunX.size(); k++)
				matFunX[j][k] = matFunX[j][k]-matFunX[i][k]*ratio;
			matFunY[j][0] = matFunY[j][0]-matFunY[i][0]*ratio;
		}
	}

	//计算拟合曲线的系数
	doubleVector coeff(matFunX.size(), 0);
	for (i=matFunX.size()-1; i>=0; i--) 
	{
		if (i==matFunX.size()-1)
			coeff[i] = matFunY[i][0]/matFunX[i][i];
		else
		{
			for (j=i+1; j<matFunX.size(); j++)
				matFunY[i][0] = matFunY[i][0]-coeff[j]*matFunX[i][j];
			coeff[i] = matFunY[i][0]/matFunX[i][i];
		}
	}
	return coeff;
}

