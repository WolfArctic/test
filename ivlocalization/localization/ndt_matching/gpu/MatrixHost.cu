#include "MatrixHost.h"
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace gpu {

extern "C" __global__ void copyMatrixDevToDev(MatrixDevice input, MatrixDevice output) {
	int row = threadIdx.x;
	int col = threadIdx.y;
	int rows_num = input.rows();
	int cols_num = input.cols();

	if (row < rows_num && col < cols_num)
		output(row, col) = input(row, col);
}

bool MatrixHost::moveToHost(MatrixDevice input){
	if (rows_ != input.rows() || cols_ != input.cols())
		return false;

	if (offset_ == input.offset()) {
		checkCudaErrors(cudaMemcpy(buffer_, input.buffer(), sizeof(double) * rows_ * cols_ * offset_, cudaMemcpyDeviceToHost));
		return true;
	}
	else {
		double *tmp;

		checkCudaErrors(cudaMalloc(&tmp, sizeof(double) * rows_ * cols_ * offset_));

		MatrixDevice tmp_output(rows_, cols_, offset_, tmp);

		dim3 block_x(rows_, cols_, 1);
		dim3 grid_x(1, 1, 1);

		copyMatrixDevToDev << <grid_x, block_x >> >(input, tmp_output);
		checkCudaErrors(cudaDeviceSynchronize());

		checkCudaErrors(cudaMemcpy(buffer_, tmp, sizeof(double) * rows_ * cols_ * offset_, cudaMemcpyDeviceToHost));
		checkCudaErrors(cudaFree(tmp));

		return true;
	}
}

bool MatrixHost::moveToGpu(MatrixDevice output){
	if (rows_ != output.rows() || cols_ != output.cols())
		return false;

	if (offset_ == output.offset()) {
		checkCudaErrors(cudaMemcpy(output.buffer(), buffer_, sizeof(double) * rows_ * cols_ * offset_, cudaMemcpyHostToDevice));
		return true;
	}
	else {
		double *tmp;

		checkCudaErrors(cudaMalloc(&tmp, sizeof(double) * rows_ * cols_ * offset_));
		checkCudaErrors(cudaMemcpy(tmp, buffer_, sizeof(double) * rows_ * cols_ * offset_, cudaMemcpyHostToDevice));

		MatrixDevice tmp_output(rows_, cols_, offset_, tmp);

		dim3 block_x(rows_, cols_, 1);
		dim3 grid_x(1, 1, 1);

		copyMatrixDevToDev<<<grid_x, block_x>>>(tmp_output, output);
		checkCudaErrors(cudaDeviceSynchronize());

		checkCudaErrors(cudaFree(tmp));

		return true;
	}
}
}
