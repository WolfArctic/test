#include "MatrixDevice.h"
#include "debug.h"

namespace gpu {
MatrixDevice::MatrixDevice(int rows, int cols) {
	rows_ = rows;
	cols_ = cols;
	offset_ = 1;
	fr_ = true;

	checkCudaErrors(cudaMalloc(&buffer_, sizeof(double) * rows_ * cols_ * offset_));
	checkCudaErrors(cudaMemset(buffer_, 0, sizeof(double) * rows_ * cols_ * offset_));
	checkCudaErrors(cudaDeviceSynchronize());
}

void MatrixDevice::memFree()
{
	if (fr_) {
		if (buffer_ != NULL)
			checkCudaErrors(cudaFree(buffer_));
	}
}


}
