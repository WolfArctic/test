#ifndef MATRIX_DEVICE_H_
#define MATRIX_DEVICE_H_

#include "Matrix.h"
#include "debug.h"

namespace gpu {
class MatrixDevice : public Matrix {
public:
	CUDAH MatrixDevice() {
		rows_ = cols_ = offset_ = 0;
		buffer_ = NULL;
		fr_ = true;
	}

	MatrixDevice(int rows, int cols){
		rows_ = rows;
		cols_ = cols;
		offset_ = 1;
		fr_ = true;

		checkCudaErrors(cudaMalloc(&buffer_, sizeof(double) * rows_ * cols_ * offset_));
		checkCudaErrors(cudaMemset(buffer_, 0, sizeof(double) * rows_ * cols_ * offset_));
		checkCudaErrors(cudaDeviceSynchronize());
	}

	void memFree()
	{
		if (fr_) {
			if (buffer_ != NULL)
				checkCudaErrors(cudaFree(buffer_));
		}
	}


	CUDAH MatrixDevice(int rows, int cols, int offset, double *buffer) {
		rows_ = rows;
		cols_ = cols;
		offset_ = offset;
		buffer_ = buffer;
		fr_ = false;
	}



	CUDAH bool isEmpty() {
		return (rows_ == 0 && cols_ == 0);
	}

	CUDAH MatrixDevice col(int index) {
		return MatrixDevice(rows_, 1, offset_ * cols_, buffer_ + index * offset_);
	}

	CUDAH MatrixDevice row(int index) {
		return MatrixDevice(1, cols_, offset_, buffer_ + index * cols_ * offset_);
	}

	CUDAH void setBuffer(double *buffer) {
		buffer_ = buffer;
	}


private:
	bool fr_;
};

class SquareMatrixDevice : public MatrixDevice {
public:
	SquareMatrixDevice(int size) : MatrixDevice(size, size) {};
};
}

#endif
