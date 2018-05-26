#include "VoxelGrid.h"
#include "debug.h"
#include "common.h"
#include <math.h>
#include <limits>
#include <thrust/device_ptr.h>
#include <thrust/device_vector.h>
#include <thrust/copy.h>
#include <thrust/scan.h>
#include <thrust/fill.h>
#include <inttypes.h>

#include <vector>
#include <cmath>

#include <stdio.h>
#include <sys/time.h>

#include "SymmetricEigensolver.h"

#include "VoxelGrid_culib.cuh"

namespace gpu {

GVoxelGrid::GVoxelGrid(const GVoxelGrid &other)
{
	x_ = other.x_;
	y_ = other.y_;
	z_ = other.z_;

	points_num_ = other.points_num_;

	centroid_ = other.centroid_;
	covariance_ = other.covariance_;
	inverse_covariance_ = other.inverse_covariance_;
	points_per_voxel_ = other.points_per_voxel_;

	voxel_num_ = other.voxel_num_;
	max_x_ = other.max_x_;
	max_y_ = other.max_y_;
	max_z_ = other.max_z_;

	min_x_ = other.min_x_;
	min_y_ = other.min_y_;
	min_z_ = other.min_z_;

	voxel_x_ = other.voxel_x_;
	voxel_y_ = other.voxel_y_;
	voxel_z_ = other.voxel_z_;

	max_b_x_ = other.max_b_x_;
	max_b_y_ = other.max_b_y_;
	max_b_z_ = other.max_b_z_;

	min_b_x_ = other.min_b_x_;
	min_b_y_ = other.min_b_y_;
	min_b_z_ = other.min_b_z_;

	vgrid_x_ = other.vgrid_x_;
	vgrid_y_ = other.vgrid_y_;
	vgrid_z_ = other.vgrid_z_;

	min_points_per_voxel_ = other.min_points_per_voxel_;

	starting_point_ids_ = other.starting_point_ids_;
	point_ids_ = other.point_ids_;


	is_copied_ = true;
}


GVoxelGrid::~GVoxelGrid() {
	if (!is_copied_) {

		for (unsigned int i = 1; i < octree_centroids_.size(); i++) {
			checkCudaErrors(cudaFree(octree_centroids_[i]));
			checkCudaErrors(cudaFree(octree_points_per_node_[i]));
		}

		octree_centroids_.clear();
		octree_points_per_node_.clear();
		octree_grid_size_.clear();

		if (starting_point_ids_ != NULL) {
			checkCudaErrors(cudaFree(starting_point_ids_));
			starting_point_ids_ = NULL;
		}

		if (point_ids_ != NULL) {
			checkCudaErrors(cudaFree(point_ids_));
			point_ids_ = NULL;
		}

		if (centroid_ != NULL) {
			checkCudaErrors(cudaFree(centroid_));
			centroid_ = NULL;
		}

		if (covariance_ != NULL) {
			checkCudaErrors(cudaFree(covariance_));
			covariance_ = NULL;
		}

		if (inverse_covariance_ != NULL) {
			checkCudaErrors(cudaFree(inverse_covariance_));
			inverse_covariance_ = NULL;
		}

		if (points_per_voxel_ != NULL) {
			checkCudaErrors(cudaFree(points_per_voxel_));
			points_per_voxel_ = NULL;
		}
	}
}



void GVoxelGrid::initialize()
{
	if (centroid_ != NULL) {
		checkCudaErrors(cudaFree(centroid_));
		centroid_ = NULL;
	}

	if (covariance_ != NULL) {
		checkCudaErrors(cudaFree(covariance_));
		covariance_ = NULL;
	}

	if (inverse_covariance_ != NULL) {
		checkCudaErrors(cudaFree(inverse_covariance_));
		inverse_covariance_ = NULL;
	}

	if (points_per_voxel_ != NULL) {
		checkCudaErrors(cudaFree(points_per_voxel_));
		points_per_voxel_ = NULL;
	}

	checkCudaErrors(cudaMalloc(&centroid_, sizeof(double) * 3 * voxel_num_));
	checkCudaErrors(cudaMalloc(&covariance_, sizeof(double) * 9 * voxel_num_));
	checkCudaErrors(cudaMalloc(&inverse_covariance_, sizeof(double) * 9 * voxel_num_));
	checkCudaErrors(cudaMalloc(&points_per_voxel_, sizeof(int) * voxel_num_));

	checkCudaErrors(cudaMemset(inverse_covariance_, 0, sizeof(double) * 9 * voxel_num_));
	checkCudaErrors(cudaMemset(points_per_voxel_, 0, sizeof(int) * voxel_num_));
	checkCudaErrors(cudaDeviceSynchronize());
}


void GVoxelGrid::setLeafSize(float voxel_x, float voxel_y, float voxel_z) {
	voxel_x_ = voxel_x;
	voxel_y_ = voxel_y;
	voxel_z_ = voxel_z;
	}

/* Get the centroid list. */
double * GVoxelGrid::getCentroidList() { return centroid_; }

/* Get the covariance list. */
double * GVoxelGrid::getCovarianceList() { return covariance_; }

/* Get the pointer to the inverse covariances list. */
double * GVoxelGrid::getInverseCovarianceList() { return inverse_covariance_; }

int * GVoxelGrid::getPointsPerVoxelList() { return points_per_voxel_; }

void GVoxelGrid::computeCentroidAndCovariance()
{
	int block_x;
#ifdef USING_HP
	block_x = (voxel_num_ > (BLOCK_SIZE_X)) ? (BLOCK_SIZE_X): voxel_num_;//gpu_up
#else
	block_x = (voxel_num_ > (BLOCK_SIZE_X/2)) ? (BLOCK_SIZE_X/2): voxel_num_;
#endif
	int grid_x = (voxel_num_ - 1) / block_x + 1;

	initCentroidAndCovariance<<<grid_x, block_x>>>(x_, y_, z_, starting_point_ids_, point_ids_, centroid_, covariance_, voxel_num_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	double *pt_sum;

	checkCudaErrors(cudaMalloc(&pt_sum, sizeof(double) * voxel_num_ * 3));

	checkCudaErrors(cudaMemcpy(pt_sum, centroid_, sizeof(double) * voxel_num_ * 3, cudaMemcpyDeviceToDevice));

	updateVoxelCentroid<<<grid_x, block_x>>>(centroid_, points_per_voxel_, voxel_num_);
	checkCudaErrors(cudaGetLastError());

	updateVoxelCovariance<<<grid_x, block_x>>>(centroid_, pt_sum, covariance_, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(pt_sum));

	double *eigenvalues_dev, *eigenvectors_dev;

	checkCudaErrors(cudaMalloc(&eigenvalues_dev, sizeof(double) * 3 * voxel_num_));
	checkCudaErrors(cudaMalloc(&eigenvectors_dev, sizeof(double) * 9 * voxel_num_));

	// Solving eigenvalues and eigenvectors problem by the GPU.
	SymmetricEigensolver3x3 sv(voxel_num_);

	sv.setInputMatrices(covariance_);
	sv.setEigenvalues(eigenvalues_dev);
	sv.setEigenvectors(eigenvectors_dev);

	normalize<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEigenvalues<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec00<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec01<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec10<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec11<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeEvec2<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	updateEval<<<grid_x, block_x>>>(sv, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	updateEval2<<<grid_x, block_x>>>(eigenvalues_dev, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	computeInverseEigenvectors<<<grid_x, block_x>>>(inverse_covariance_, points_per_voxel_, voxel_num_, eigenvectors_dev, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	updateCovarianceS0<<<grid_x, block_x>>>(points_per_voxel_, voxel_num_, eigenvalues_dev, eigenvectors_dev, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	for (int i = 0; i < 3; i++) {
		updateCovarianceS1<<<grid_x, block_x>>>(covariance_, inverse_covariance_, points_per_voxel_, voxel_num_, eigenvectors_dev, min_points_per_voxel_, i);
		checkCudaErrors(cudaGetLastError());
	}
	checkCudaErrors(cudaDeviceSynchronize());

	computeInverseCovariance<<<grid_x, block_x>>>(covariance_, inverse_covariance_, points_per_voxel_, voxel_num_, min_points_per_voxel_);
	checkCudaErrors(cudaGetLastError());

	checkCudaErrors(cudaDeviceSynchronize());

	sv.memFree();
	checkCudaErrors(cudaFree(eigenvalues_dev));
	checkCudaErrors(cudaFree(eigenvectors_dev));
}

//Input are supposed to be in device memory
void GVoxelGrid::setInput(float *x, float *y, float *z, int points_num)
{
	if (points_num <= 0)
		return;
	x_ = x;
	y_ = y;
	z_ = z;
	points_num_ = points_num;

	findBoundaries();

	voxel_num_ = vgrid_x_ * vgrid_y_ * vgrid_z_;

	initialize();

	scatterPointsToVoxelGrid();

	computeCentroidAndCovariance();

	buildOctree();
}

//extern "C"  
int float2int(float input)
{
	if(input >= 0.0){
		return (int)input;
	} else {
		return (int)input - 1;
	}
}

void GVoxelGrid::findBoundaries()
{
	float *max_x, *max_y, *max_z, *min_x, *min_y, *min_z;

	checkCudaErrors(cudaMalloc(&max_x, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&max_y, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&max_z, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&min_x, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&min_y, sizeof(float) * points_num_));
	checkCudaErrors(cudaMalloc(&min_z, sizeof(float) * points_num_));

	checkCudaErrors(cudaMemcpy(max_x, x_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(max_y, y_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(max_z, z_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));

	checkCudaErrors(cudaMemcpy(min_x, x_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(min_y, y_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));
	checkCudaErrors(cudaMemcpy(min_z, z_, sizeof(float) * points_num_, cudaMemcpyDeviceToDevice));

	int points_num = points_num_;
	while (points_num > 1) {
		int half_points_num = (points_num - 1) / 2 + 1;
		int block_x = (half_points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_points_num;
		int grid_x = (half_points_num - 1) / block_x + 1;

		findMax<<<grid_x, block_x>>>(max_x, max_y, max_z, points_num, half_points_num);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());//add by panrui

		findMin<<<grid_x, block_x>>>(min_x, min_y, min_z, points_num, half_points_num);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());//add by panrui

		points_num = half_points_num;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMemcpy(&max_x_, max_x, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&max_y_, max_y, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&max_z_, max_z, sizeof(float), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaMemcpy(&min_x_, min_x, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&min_y_, min_y, sizeof(float), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&min_z_, min_z, sizeof(float), cudaMemcpyDeviceToHost));

//find by haotian
	max_b_x_ = (float2int(max_x_ / voxel_x_));
	max_b_y_ = (float2int(max_y_ / voxel_y_));
	max_b_z_ = (float2int(max_z_ / voxel_z_));

	min_b_x_ = (float2int(min_x_ / voxel_x_));
	min_b_y_ = (float2int(min_y_ / voxel_y_));
	min_b_z_ = (float2int(min_z_ / voxel_z_));

	vgrid_x_ = max_b_x_ - min_b_x_ + 1;
	vgrid_y_ = max_b_y_ - min_b_y_ + 1;
	vgrid_z_ = max_b_z_ - min_b_z_ + 1;

	checkCudaErrors(cudaFree(max_x));
	checkCudaErrors(cudaFree(max_y));
	checkCudaErrors(cudaFree(max_z));

	checkCudaErrors(cudaFree(min_x));
	checkCudaErrors(cudaFree(min_y));
	checkCudaErrors(cudaFree(min_z));
}


template <typename T>
void GVoxelGrid::ExclusiveScan(T *input, int ele_num, T *sum)
{
	thrust::device_ptr<T> dev_ptr(input);

	thrust::exclusive_scan(dev_ptr, dev_ptr + ele_num, dev_ptr);
	checkCudaErrors(cudaDeviceSynchronize());

	*sum = *(dev_ptr + ele_num - 1);
}

template <typename T>
void GVoxelGrid::ExclusiveScan(T *input, int ele_num)
{
	thrust::device_ptr<T> dev_ptr(input);

	thrust::exclusive_scan(dev_ptr, dev_ptr + ele_num, dev_ptr);
	checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::radiusSearch(float *qx, float *qy, float *qz, int points_num, float radius, int max_nn,
										int **valid_points, int **starting_voxel_id, int **valid_voxel_id,
										int *valid_voxel_num, int *valid_points_num)
{
	//Testing input query points
	int block_x = (points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : points_num;
	int grid_x = (points_num - 1) / block_x + 1;

	//Boundaries of candidate voxels per points
	int *max_vid_x, *max_vid_y, *max_vid_z;
	int *min_vid_x, *min_vid_y, *min_vid_z;

	checkCudaErrors(cudaMalloc(&max_vid_x, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&max_vid_y, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&max_vid_z, sizeof(int) * points_num));

	checkCudaErrors(cudaMalloc(&min_vid_x, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&min_vid_y, sizeof(int) * points_num));
	checkCudaErrors(cudaMalloc(&min_vid_z, sizeof(int) * points_num));

	//Determine the number of candidate voxel per points
	int *candidate_voxel_num_per_point;
	int total_candidate_voxel_num;

	checkCudaErrors(cudaMalloc(&candidate_voxel_num_per_point, sizeof(int) * (points_num + 1)));

	findBoundariesOfCandidateVoxels<<<grid_x, block_x>>>(qx, qy, qz, radius, points_num,
															voxel_x_, voxel_y_, voxel_z_,
															max_b_x_, max_b_y_, max_b_z_,
															min_b_x_, min_b_y_, min_b_z_,
															max_vid_x, max_vid_y, max_vid_z,
															min_vid_x, min_vid_y, min_vid_z,
															candidate_voxel_num_per_point);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	//Total candidate voxel num is determined by an exclusive scan on candidate_voxel_num_per_point
	ExclusiveScan(candidate_voxel_num_per_point, points_num + 1, &total_candidate_voxel_num);

	if (total_candidate_voxel_num <= 0) {
		std::cout << "No candidate voxel was found. Exiting..." << std::endl;

		checkCudaErrors(cudaFree(max_vid_x));
		checkCudaErrors(cudaFree(max_vid_y));
		checkCudaErrors(cudaFree(max_vid_z));

		checkCudaErrors(cudaFree(min_vid_x));
		checkCudaErrors(cudaFree(min_vid_y));
		checkCudaErrors(cudaFree(min_vid_z));

		checkCudaErrors(cudaFree(candidate_voxel_num_per_point));

		valid_points = NULL;
		starting_voxel_id = NULL;
		valid_voxel_id = NULL;

		*valid_voxel_num = 0;
		*valid_points_num = 0;

		return;
	}


	//Determine the voxel id of candidate voxels
	int *candidate_voxel_id;

	checkCudaErrors(cudaMalloc(&candidate_voxel_id, sizeof(int) * total_candidate_voxel_num));

	updateCandidateVoxelIds<<<grid_x, block_x>>>(points_num, vgrid_x_, vgrid_y_, vgrid_z_,
													max_vid_x, max_vid_y, max_vid_z,
													min_vid_x, min_vid_y, min_vid_z,
													candidate_voxel_num_per_point, candidate_voxel_id);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	//Go through the candidate voxel id list and find out which voxels are really inside the radius
	int *valid_voxel_mark;

	checkCudaErrors(cudaMalloc(&valid_voxel_mark, sizeof(int) * total_candidate_voxel_num));

	int *valid_voxel_count;

	checkCudaErrors(cudaMalloc(&valid_voxel_count, sizeof(int) * (points_num + 1)));

	int *valid_points_mark;

	checkCudaErrors(cudaMalloc(&valid_points_mark, sizeof(int) * points_num));

#ifdef USING_HP
	block_x = (total_candidate_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : total_candidate_voxel_num;//gpu_up
#else
	block_x = (total_candidate_voxel_num > BLOCK_SIZE_X/2) ? BLOCK_SIZE_X/2 : total_candidate_voxel_num;
#endif
	grid_x = (total_candidate_voxel_num - 1) / block_x + 1;

	///CHECK VALID VOXEL COUNT AGAIN
	inspectCandidateVoxels<<<grid_x, block_x>>>(qx, qy, qz, radius, max_nn, points_num,
													centroid_, points_per_voxel_, voxel_num_,
													candidate_voxel_num_per_point, candidate_voxel_id,
													valid_voxel_mark, valid_voxel_count, valid_points_mark);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	block_x = (total_candidate_voxel_num > (BLOCK_SIZE_X)) ? (BLOCK_SIZE_X) : total_candidate_voxel_num;
	grid_x = (total_candidate_voxel_num - 1) / block_x + 1;

	//Collect valid points
	int *valid_points_location;

	checkCudaErrors(cudaMalloc(&valid_points_location, sizeof(int) * (points_num + 1)));
	checkCudaErrors(cudaMemcpy(valid_points_location, valid_points_mark, sizeof(int) * points_num, cudaMemcpyDeviceToDevice));

	//Writing location to the output buffer is determined by an exclusive scan
	ExclusiveScan(valid_points_location, points_num + 1, valid_points_num);

	if (*valid_points_num <= 0) {
		std::cout << "No valid point was found. Exiting..." << std::endl;
		checkCudaErrors(cudaFree(max_vid_x));
		checkCudaErrors(cudaFree(max_vid_y));
		checkCudaErrors(cudaFree(max_vid_z));

		checkCudaErrors(cudaFree(min_vid_x));
		checkCudaErrors(cudaFree(min_vid_y));
		checkCudaErrors(cudaFree(min_vid_z));

		checkCudaErrors(cudaFree(candidate_voxel_num_per_point));
		checkCudaErrors(cudaFree(candidate_voxel_id));

		checkCudaErrors(cudaFree(valid_voxel_mark));
		checkCudaErrors(cudaFree(valid_voxel_count));
		checkCudaErrors(cudaFree(valid_points_mark));

		checkCudaErrors(cudaFree(valid_points_location));

		valid_points = NULL;
		starting_voxel_id = NULL;
		valid_voxel_id = NULL;

		*valid_voxel_num = 0;
		*valid_points_num = 0;

		return;
	}

	checkCudaErrors(cudaMalloc(valid_points, sizeof(int) * (*valid_points_num)));

	collectValidPoints<<<grid_x, block_x>>>(valid_points_mark, *valid_points, valid_points_location, points_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMalloc(starting_voxel_id, sizeof(int) * (*valid_points_num + 1)));

	collectValidVoxelCount<<<grid_x, block_x>>>(valid_voxel_count, *starting_voxel_id, valid_points_location, points_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	ExclusiveScan(*starting_voxel_id, *valid_points_num + 1, valid_voxel_num);

	//Collect valid voxels
	int *valid_voxel_location;

	checkCudaErrors(cudaMalloc(&valid_voxel_location, sizeof(int) * (total_candidate_voxel_num + 1)));
	checkCudaErrors(cudaMemcpy(valid_voxel_location, valid_voxel_mark, sizeof(int) * total_candidate_voxel_num, cudaMemcpyDeviceToDevice));

	ExclusiveScan(valid_voxel_location, total_candidate_voxel_num + 1, valid_voxel_num);

	if (*valid_voxel_num <= 0) {
		checkCudaErrors(cudaFree(max_vid_x));
		checkCudaErrors(cudaFree(max_vid_y));
		checkCudaErrors(cudaFree(max_vid_z));

		checkCudaErrors(cudaFree(min_vid_x));
		checkCudaErrors(cudaFree(min_vid_y));
		checkCudaErrors(cudaFree(min_vid_z));

		checkCudaErrors(cudaFree(candidate_voxel_num_per_point));
		checkCudaErrors(cudaFree(candidate_voxel_id));

		checkCudaErrors(cudaFree(valid_voxel_mark));
		checkCudaErrors(cudaFree(valid_voxel_count));
		checkCudaErrors(cudaFree(valid_points_mark));

		checkCudaErrors(cudaFree(valid_points_location));
		checkCudaErrors(cudaFree(valid_voxel_location));

		valid_points = NULL;
		starting_voxel_id = NULL;
		valid_voxel_id = NULL;

		*valid_voxel_num = 0;
		*valid_points_num = 0;
	}

	checkCudaErrors(cudaMalloc(valid_voxel_id, sizeof(int) * (*valid_voxel_num)));

	block_x = (total_candidate_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : total_candidate_voxel_num;
	grid_x = (total_candidate_voxel_num - 1) / block_x + 1;

	collectValidVoxels<<<grid_x, block_x>>>(valid_voxel_mark, candidate_voxel_id, *valid_voxel_id, valid_voxel_location, total_candidate_voxel_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(max_vid_x));
	checkCudaErrors(cudaFree(max_vid_y));
	checkCudaErrors(cudaFree(max_vid_z));

	checkCudaErrors(cudaFree(min_vid_x));
	checkCudaErrors(cudaFree(min_vid_y));
	checkCudaErrors(cudaFree(min_vid_z));

	checkCudaErrors(cudaFree(candidate_voxel_num_per_point));
	checkCudaErrors(cudaFree(candidate_voxel_id));

	checkCudaErrors(cudaFree(valid_voxel_mark));
	checkCudaErrors(cudaFree(valid_points_mark));
	checkCudaErrors(cudaFree(valid_voxel_count));

	checkCudaErrors(cudaFree(valid_points_location));
	checkCudaErrors(cudaFree(valid_voxel_location));
}

void GVoxelGrid::scatterPointsToVoxelGrid()
{
	if (starting_point_ids_ != NULL) {
		checkCudaErrors(cudaFree(starting_point_ids_));
		starting_point_ids_ = NULL;
	}

	if (point_ids_ != NULL) {
		checkCudaErrors(cudaFree(point_ids_));
		point_ids_ = NULL;
	}

	int block_x = (points_num_ > BLOCK_SIZE_X) ? BLOCK_SIZE_X : points_num_;
	int grid_x = (points_num_ - 1) / block_x + 1;

	insertPointsToGrid<<<grid_x, block_x>>>(x_, y_, z_, points_num_, points_per_voxel_,
												vgrid_x_, vgrid_y_, vgrid_z_,
												voxel_x_, voxel_y_, voxel_z_,
												min_b_x_, min_b_y_, min_b_z_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaMalloc(&starting_point_ids_, sizeof(int) * (voxel_num_ + 1)));

	int *writing_location;

	checkCudaErrors(cudaMalloc(&writing_location, sizeof(int) * voxel_num_));

	checkCudaErrors(cudaMemcpy(starting_point_ids_, points_per_voxel_, sizeof(int) * voxel_num_, cudaMemcpyDeviceToDevice));

	ExclusiveScan(starting_point_ids_, voxel_num_ + 1);

	checkCudaErrors(cudaMemcpy(writing_location, starting_point_ids_, sizeof(int) * voxel_num_, cudaMemcpyDeviceToDevice));

	checkCudaErrors(cudaMalloc(&point_ids_, sizeof(int) * points_num_));

	scatterPointsToVoxels<<<grid_x, block_x>>>(x_, y_, z_, points_num_,
												voxel_x_, voxel_y_, voxel_z_,
												min_b_x_, min_b_y_, min_b_z_,
												vgrid_x_, vgrid_y_, vgrid_z_,
												writing_location, point_ids_);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(writing_location));
}

void GVoxelGrid::buildOctree()
{
	for (unsigned int i = 1; i < octree_centroids_.size(); i++) {
		checkCudaErrors(cudaFree(octree_centroids_[i]));
		checkCudaErrors(cudaFree(octree_points_per_node_[i]));
	}

	octree_centroids_.clear();
	octree_points_per_node_.clear();
	octree_grid_size_.clear();

	//Push leafs to the octree list
	octree_centroids_.push_back(centroid_);
	octree_points_per_node_.push_back(points_per_voxel_);
	OctreeGridSize grid_size;

	grid_size.size_x = vgrid_x_;
	grid_size.size_y = vgrid_y_;
	grid_size.size_z = vgrid_z_;

	octree_grid_size_.push_back(grid_size);

	int node_number = voxel_num_;
	int child_grid_x, child_grid_y, child_grid_z;
	int parent_grid_x, parent_grid_y, parent_grid_z;

	int i = 0;

	while (node_number > 8) {
		child_grid_x = octree_grid_size_[i].size_x;
		child_grid_y = octree_grid_size_[i].size_y;
		child_grid_z = octree_grid_size_[i].size_z;

		parent_grid_x = (child_grid_x - 1) / 2 + 1;
		parent_grid_y = (child_grid_y - 1) / 2 + 1;
		parent_grid_z = (child_grid_z - 1) / 2 + 1;

		node_number = parent_grid_x * parent_grid_y * parent_grid_z;

		double *parent_centroids;
		int *points_per_parent;

		checkCudaErrors(cudaMalloc(&parent_centroids, sizeof(double) * 3 * node_number));
		checkCudaErrors(cudaMalloc(&points_per_parent, sizeof(int) * node_number));

		double *child_centroids = octree_centroids_[i];
		int *points_per_child = octree_points_per_node_[i];

		int block_x = (parent_grid_x > BLOCK_X) ? BLOCK_X : parent_grid_x;
		int block_y = (parent_grid_y > BLOCK_Y) ? BLOCK_Y : parent_grid_y;
		int block_z = (parent_grid_z > BLOCK_Z) ? BLOCK_Z : parent_grid_z;

		int grid_x = (parent_grid_x - 1) / block_x + 1;
		int grid_y = (parent_grid_y - 1) / block_y + 1;
		int grid_z = (parent_grid_z - 1) / block_z + 1;

		dim3 block(block_x, block_y, block_z);
		dim3 grid(grid_x, grid_y, grid_z);

		buildParent<<<grid, block>>>(child_centroids, points_per_child,
										child_grid_x, child_grid_y, child_grid_z, child_grid_x * child_grid_y * child_grid_z,
										parent_centroids, points_per_parent,
										parent_grid_x, parent_grid_y, parent_grid_z);
		checkCudaErrors(cudaGetLastError());
		octree_centroids_.push_back(parent_centroids);
		octree_points_per_node_.push_back(points_per_parent);

		grid_size.size_x = parent_grid_x;
		grid_size.size_y = parent_grid_y;
		grid_size.size_z = parent_grid_z;

		octree_grid_size_.push_back(grid_size);

		i++;
	}

	checkCudaErrors(cudaDeviceSynchronize());
}

void GVoxelGrid::nearestNeighborSearch(float *trans_x, float *trans_y, float *trans_z, int point_num, int *valid_distance, double *min_distance, float max_range)
{

	int *vid_x, *vid_y, *vid_z;

	checkCudaErrors(cudaMalloc(&vid_x, sizeof(int) * point_num));
	checkCudaErrors(cudaMalloc(&vid_y, sizeof(int) * point_num));
	checkCudaErrors(cudaMalloc(&vid_z, sizeof(int) * point_num));

	checkCudaErrors(cudaMemset(vid_x, 0, sizeof(int) * point_num));
	checkCudaErrors(cudaMemset(vid_y, 0, sizeof(int) * point_num));
	checkCudaErrors(cudaMemset(vid_z, 0, sizeof(int) * point_num));
	checkCudaErrors(cudaDeviceSynchronize());

	int block_x;
#ifdef USING_HP
	block_x = (point_num > (BLOCK_SIZE_X)) ? (BLOCK_SIZE_X) : point_num;//gpu_up
#else
	block_x = (point_num > (BLOCK_SIZE_X/2)) ? (BLOCK_SIZE_X/2) : point_num;
#endif
	int grid_x = (point_num - 1) / block_x + 1;

	// Go through top of the octree to the bottom
	for (int i = octree_centroids_.size() - 1; i >= 0; i--) {
		double *centroids = octree_centroids_[i];
		int *points_per_node = octree_points_per_node_[i];
		int vgrid_x = octree_grid_size_[i].size_x;
		int vgrid_y = octree_grid_size_[i].size_y;
		int vgrid_z = octree_grid_size_[i].size_z;
		int node_num = vgrid_x * vgrid_y * vgrid_z;

		nearestOctreeNodeSearch<<<grid_x, block_x>>>(trans_x, trans_y, trans_z,
														vid_x, vid_y, vid_z,
														point_num,
														centroids, points_per_node,
														vgrid_x, vgrid_y, vgrid_z, node_num);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize()); //added by panrui
	}


//added by panrui
block_x = (point_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : point_num;
grid_x = (point_num - 1) / block_x + 1;

	nearestPointSearch<<<grid_x, block_x>>>(trans_x, trans_y, trans_z, point_num,
												x_, y_, z_, points_num_,
												vid_x, vid_y, vid_z,
												vgrid_x_, vgrid_y_, vgrid_z_, voxel_num_,
												starting_point_ids_, point_ids_,
												min_distance);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize()); //added by panrui



	verifyDistances<<<grid_x, block_x>>>(valid_distance, min_distance, max_range, point_num);
	checkCudaErrors(cudaGetLastError());
	checkCudaErrors(cudaDeviceSynchronize());

	checkCudaErrors(cudaFree(vid_x));
	checkCudaErrors(cudaFree(vid_y));
	checkCudaErrors(cudaFree(vid_z));
}

}
