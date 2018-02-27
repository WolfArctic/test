#include "kmeans.h"
int main()
{

	cout << "Test" << endl;
	char *filename = "testSet.txt";
	int k = 4;
	KMEANS<double> kms(k);
	kms.loadDataSet(filename);
	kms.randCent();
	kms.kmeans();

	return 0;
}