#include "MatrixHost.h"
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace gpu {

extern "C" __device__ int voxelId(float x, float y, float z,
									float voxel_x, float voxel_y, float voxel_z,
									int min_b_x, int min_b_y, int min_b_z,
									int vgrid_x, int vgrid_y, int vgrid_z);

extern "C" __global__ void initCentroidAndCovariance(float *x, float *y, float *z, int *starting_point_ids, int *point_ids,
														double *centroids, double *covariances, int voxel_num);

extern "C" __global__ void updateVoxelCentroid(double *centroid, int *points_per_voxel, int voxel_num);

extern "C" __global__ void updateVoxelCovariance(double *centroid, double *pt_sum, double *covariance, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeInverseEigenvectors(double *inverse_covariance, int *points_per_voxel, int voxel_num, double *eigenvectors, int min_points_per_voxel);

extern "C" __global__ void updateCovarianceS0(int *points_per_voxel, int voxel_num, double *eigenvalues, double *eigenvectors, int min_points_per_voxel);

extern "C" __global__ void updateCovarianceS1(double *covariance, double *inverse_covariance, int *points_per_voxel, int voxel_num, double *eigenvectors, int min_points_per_voxel, int col);

extern "C" __global__ void computeInverseCovariance(double *covariance, double *inverse_covariance, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

template<typename T>
__global__ void init(T *input, int size, int local_size);

extern "C" __global__ void initBoolean(bool *input, int size);

extern "C" __global__ void normalize(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeEigenvalues(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeEvec00(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeEvec01(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeEvec10(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeEvec11(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void computeEvec2(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void updateEval(SymmetricEigensolver3x3 sv, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C" __global__ void updateEval2(double *eigenvalues, int *points_per_voxel, int voxel_num, int min_points_per_voxel);

extern "C"  __global__ void findMax(float *x, float *y, float *z, int full_size, int half_size);

extern "C"  __global__ void findMin(float *x, float *y, float *z, int full_size, int half_size);

extern "C"  __global__ void findBoundariesOfCandidateVoxels(float *x, float *y, float *z,
																float radius, int points_num,
																float voxel_x, float voxel_y, float voxel_z,
																int max_b_x, int max_b_y, int max_b_z,
																int min_b_x, int min_b_y, int min_b_z,
																int *max_vid_x, int *max_vid_y, int *max_vid_z,
																int *min_vid_x, int *min_vid_y, int *min_vid_z,
																int *candidate_voxel_per_point);

extern "C"  __global__ void collectValidPoints(int *valid_points_mark, int *valid_points_id, int *valid_points_location, int points_num);

extern "C"  __global__ void updateCandidateVoxelIds(int points_num,
													int vgrid_x, int vgrid_y, int vgrid_z,
													int *max_vid_x, int *max_vid_y, int *max_vid_z,
													int *min_vid_x, int *min_vid_y, int *min_vid_z,
													int *starting_voxel_id,
													int *candidate_voxel_id);

extern "C" __global__ void inspectCandidateVoxels(float *x, float *y, float *z,
													float radius, int max_nn, int points_num,
													double *centroid, int *points_per_voxel, int offset,
													int *starting_voxel_id, int *candidate_voxel_id,
													int *valid_voxel_mark, int *valid_voxel_count, int *valid_points_mark);

extern "C"  __global__ void collectValidVoxels(int *valid_voxels_mark, int *candidate_voxel_id, int *output, int *writing_location, int candidate_voxel_num);

extern "C" __global__ void collectValidVoxelCount(int *input_valid_voxel_count, int *output_valid_voxel_count, int *writing_location, int points_num);

extern "C" __global__ void buildParent(double *child_centroids, int *points_per_child,
										int child_grid_x, int child_grid_y, int child_grid_z, int child_num,
										double *parent_centroids, int *points_per_parent,
										int parent_grid_x, int parent_grid_y, int parent_grid_z);

extern "C"  __global__ void insertPointsToGrid(float *x, float *y, float *z, int points_num,
												int *points_per_voxel,
												int vgrid_x, int vgrid_y, int vgrid_z,
												float voxel_x, float voxel_y, float voxel_z,
												int min_b_x, int min_b_y, int min_b_z);

extern "C" __global__ void scatterPointsToVoxels(float *x, float *y, float *z, int points_num,
													float voxel_x, float voxel_y, float voxel_z,
													int min_b_x, int min_b_y, int min_b_z,
													int vgrid_x, int vgrid_y, int vgrid_z,
													int *writing_locations, int *point_ids);

extern "C" __global__ void nearestOctreeNodeSearch(float *x, float *y, float *z,
													int *vid_x, int *vid_y, int *vid_z,
													int points_num,
													double *centroids, int *points_per_node,
													int vgrid_x, int vgrid_y, int vgrid_z, int node_num);


extern "C" __global__ void nearestPointSearch(float *qx, float *qy, float *qz, int qpoints_num,
												float *rx, float *ry, float *rz, int rpoints_num,
												int *vid_x, int *vid_y, int *vid_z,
												int vgrid_x, int vgrid_y, int vgrid_z, int voxel_num,
												int *starting_point_id, int *point_id, double *min_distance);

extern "C" __global__ void verifyDistances(int *valid_distance, double *min_distance, double max_range, int points_num);











}
