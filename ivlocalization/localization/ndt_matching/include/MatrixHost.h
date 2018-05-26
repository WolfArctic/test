#ifndef MATRIX_HOST_H_
#define MATRIX_HOST_H_

#include "Matrix.h"
#include "MatrixDevice.h"
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace gpu {
class MatrixHost : public Matrix {
public:
	MatrixHost() : Matrix() { fr_ = false; }


	MatrixHost(int rows, int cols) {
		rows_ = rows;
		cols_ = cols;
		offset_ = 1;

		buffer_ = (double*)malloc(sizeof(double) * rows_ * cols_ * offset_);
		memset(buffer_, 0, sizeof(double) * rows_ * cols_ * offset_);
		fr_ = true;
	}


	MatrixHost(const MatrixHost& other){
		rows_ = other.rows_;
		cols_ = other.cols_;
		offset_ = other.offset_;
		fr_ = other.fr_;

		if (fr_) {
			buffer_ = (double*)malloc(sizeof(double) * rows_ * cols_ * offset_);
			memcpy(buffer_, other.buffer_, sizeof(double) * rows_ * cols_ * offset_);
		} else {
			buffer_ = other.buffer_;
		}
}


	MatrixHost(int rows, int cols, int offset, double *buffer) {
		rows_ = rows;
		cols_ = cols;
		offset_ = offset;
		buffer_ = buffer;
		fr_ = false;
	}

	bool moveToGpu(MatrixDevice output);

	bool moveToHost(MatrixDevice input);

	MatrixHost &operator=(const MatrixHost &other){
		rows_ = other.rows_;
		cols_ = other.cols_;
		offset_ = other.offset_;
		fr_ = other.fr_;

		if (fr_) {
			buffer_ = (double*)malloc(sizeof(double) * rows_ * cols_ * offset_);
			memcpy(buffer_, other.buffer_, sizeof(double) * rows_ * cols_ * offset_);
		} else {
			buffer_ = other.buffer_;
		}

		return *this;
	}


	void debug(){
		for (int i = 0; i < rows_; i++) {
			for (int j = 0; j < cols_; j++) {
				std::cout << buffer_[(i * cols_ + j) * offset_] << " ";
			}

			std::cout << std::endl;
		}

		std::cout << std::endl;
	}


	~MatrixHost(){
		if (fr_)
			free(buffer_);
	}
private:
	bool fr_;
};

class SquareMatrixHost: public MatrixHost {
public:
	SquareMatrixHost(int size) : MatrixHost(size, size) {};
};

}

#endif
