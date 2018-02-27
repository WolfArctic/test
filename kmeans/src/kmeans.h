#ifndef _KMEANS_H
#define _KMEANS_H
#include <iostream>
#include <vector>
#include <map>
#include <cstdlib>
#include <algorithm>
#include <fstream>
#include <string.h>
#include <string>
#include <time.h>
#include <limits.h>

using namespace std;

template <typename T>
class KMEANS
{

  public:
	KMEANS(int k);
	void loadDataSet(char *filename);
	void randCent();
	void print();
	void kmeans();

  private:
	vector<vector<T> > dataSet;
	vector<T> mmin, mmax;
	int colLen, rowLen;
	int k;
	vector<vector<T> > centroids;
	typedef struct MinMax
	{
		T Min;
		T Max;
		MinMax(T min, T max) : Min(min), Max(max) {}
	} tMinMax;

	typedef struct Node
	{
		int minIndex;
		double minDist;
		Node(int idx, double dist) : minIndex(idx), minDist(dist) {}
	} tNode;

	vector<tNode> clusterAssment;

	void split(char *buffer, vector<T> &vec);
	tMinMax getMinMax(int idx);
	void setCentroids(tMinMax &tminmax, int idx);
	void initClusterAssment();
	double distEclud(vector<T> &v1, vector<T> &v2);
};

template <typename T>
void KMEANS<T>::initClusterAssment()
{
	tNode node(-1, -1);
	for (int i = 0; i < rowLen; i++)
	{
		clusterAssment.push_back(node);
	}
}

template <typename T>
void KMEANS<T>::kmeans()
{
	initClusterAssment();
	bool clusterChanged = true;
	while (clusterChanged)
	{
		clusterChanged = false;
		cout << "Find the nearest centroid of each point : " << endl;
		for (int i = 0; i < rowLen; i++)
		{
			int minIndex = -1;
			double minDist = INT_MAX;
			for (int j = 0; j < k; j++)
			{
				double distJI = distEclud(centroids[j], dataSet[i]);
				if (distJI < minDist)
				{
					minDist = distJI;
					minIndex = j;
				}
			}
			if (clusterAssment[i].minIndex != minIndex)
			{
				clusterChanged = true;
				clusterAssment[i].minIndex = minIndex;
				clusterAssment[i].minDist = minDist;
			}
		}
		cout << "Update the centroid : " << endl;
		for (int cent = 0; cent < k; cent++)
		{
			vector<T> vec(colLen, 0);
			int cnt = 0;
			for (int i = 0; i < rowLen; i++)
			{
				if (clusterAssment[i].minIndex == cent)
				{
					++cnt;
					for (int j = 0; j < colLen; j++)
					{
						vec[j] += dataSet[i].at(j);
					}
				}
			}
			for (int i = 0; i < colLen; i++)
			{
				if (cnt != 0)
				{
					vec[i] /= cnt;
				}
				centroids[cent].at(i) = vec[i];
			}
		}
		print();
	}
}

template <typename T>
KMEANS<T>::KMEANS(int k)
{
	this->k = k;
}

template <typename T>
void KMEANS<T>::setCentroids(tMinMax &tminmax, int idx)
{
	T rangeIdx = tminmax.Max - tminmax.Min;
	for (int i = 0; i < k; i++)
	{
		centroids[i].at(idx) = tminmax.Min + rangeIdx * (rand() / (double)RAND_MAX);
		cout << "centroids "
			 << "i "<<i
			 << " .at "<<idx<<" "<< centroids[i].at(idx) << endl;
	}
}

template <typename T>
typename KMEANS<T>::tMinMax KMEANS<T>::getMinMax(int idx)
{
	T min, max;
	dataSet[0].at(idx) > dataSet[1].at(idx) ? (max = dataSet[0].at(idx), min = dataSet[1].at(idx)) : (max = dataSet[1].at(idx), min = dataSet[0].at(idx));

	for (int i = 2; i < rowLen; i++)
	{
		if (dataSet[i].at(idx) < min)
		{
			min = dataSet[i].at(idx);
		}
		else if (dataSet[i].at(idx) > max)
		{
			max = dataSet[i].at(idx);
		}
		else
			continue;
	}
	tMinMax tminmax(min, max);
	return tminmax;
}

template <typename T>
void KMEANS<T>::randCent()
{
	vector<T> vec(colLen, 0);
	for (int i = 0; i < k; i++)
	{
		centroids.push_back(vec);
	}
	for (int i = 0; i < centroids.size(); i++)
	{
		cout << "centroids " << i << " .at 0 "
			 << "= " << centroids[i].at(0) << endl;
		cout << "centroids " << i << " .at 1 "
			 << "= " << centroids[i].at(1) << endl;
	}
	srand(time(NULL));//什么意思
	cout << "种子 " << rand() << endl;//这有什么用
	for (int j = 0; j < colLen; j++)
	{
		tMinMax tminmax = getMinMax(j);//找出这一列中最大最小值
		setCentroids(tminmax, j);//根据最大最小值设置该列的k个中心点
	}
}

template <typename T>
double KMEANS<T>::distEclud(vector<T> &v1, vector<T> &v2)
{
	T sum = 0;
	int size = v1.size();
	for (int i = 0; i < size; i++)
	{
		sum += (v1[i] - v2[i]) * (v1[i] - v2[i]);
	}
	return sum;
}

template <typename T>
void KMEANS<T>::split(char *buffer, vector<T> &vec)
{
	char *p = strtok(buffer, " \t");
	while (p != NULL)
	{
		vec.push_back(atof(p));
		p = strtok(NULL, " ");
	}
}

template <typename T>
void KMEANS<T>::print()
{
	ofstream fout;
	fout.open("res.txt");
	if (!fout)
	{
		cout << "File res.txt open filed " << endl;
		exit(0);
	}

	typename vector<vector<T> >::iterator it = dataSet.begin();
	typename vector<tNode>::iterator itt = clusterAssment.begin();
	for (int i = 0; i < rowLen; i++)
	{
		typename vector<T>::iterator it2 = (*it).begin();
		while (it2 != (*it).end())
		{
			fout << *it2 << "\t";
			it2++;
		}
		fout << (*itt).minIndex << endl;
		itt++;
		it++;
	}
	fout.close();
}

template <typename T>
void KMEANS<T>::loadDataSet(char *filename)
{
	FILE *pFile;
	pFile = fopen(filename, "r");
	if (!pFile)
	{
		printf("open file %s failed...\n", filename);
		exit(0);
	}

	char *buffer = new char[100];
	vector<T> temp;
	while (fgets(buffer, 100, pFile))
	{
		temp.clear();
		split(buffer, temp);
		dataSet.push_back(temp);
	}
	colLen = dataSet[0].size();
	rowLen = dataSet.size();
	delete[] buffer;
}

#endif //_KMEANS_H
