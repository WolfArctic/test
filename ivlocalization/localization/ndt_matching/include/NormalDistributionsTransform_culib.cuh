#include "MatrixHost.h"
#include "debug.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

namespace gpu {
extern "C" __global__ void computePointGradients0(float *x, float *y, float *z, int points_num,
													int *valid_points, int valid_points_num,
													MatrixDevice j_ang_a, MatrixDevice j_ang_b, MatrixDevice j_ang_c, MatrixDevice j_ang_d,
													double *point_gradients);

extern "C" __global__ void computePointGradients1(float *x, float *y, float *z, int points_num,
										int *valid_points, int valid_points_num,
										MatrixDevice j_ang_e, MatrixDevice j_ang_f, MatrixDevice j_ang_g, MatrixDevice j_ang_h,
										double *point_gradients);

extern "C" __global__ void computePointHessian0(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												MatrixDevice h_ang_a2, MatrixDevice h_ang_a3,
												MatrixDevice h_ang_b2, MatrixDevice h_ang_b3,
												double *point_hessians);

extern "C" __global__ void computePointHessian1(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												MatrixDevice h_ang_c2, MatrixDevice h_ang_c3,
												MatrixDevice h_ang_d1, MatrixDevice h_ang_d2,
												MatrixDevice h_ang_d3,
												double *point_hessians);

extern "C" __global__ void computePointHessian2(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												MatrixDevice h_ang_e1, MatrixDevice h_ang_e2, MatrixDevice h_ang_e3,
												MatrixDevice h_ang_f1, MatrixDevice h_ang_f2, MatrixDevice h_ang_f3,
												double *point_hessians);

extern "C" __global__ void computeScoreList(int *starting_voxel_id, int *voxel_id, int valid_points_num,
												double *e_x_cov_x, double gauss_d1, double *score);

extern "C" __global__ void computeScoreGradientList(float *trans_x, float *trans_y, float *trans_z,
														int *valid_points,
														int *starting_voxel_id, int *voxel_id, int valid_points_num,
														double *centroid, int voxel_num, double *e_x_cov_x,
														double *cov_dxd_pi, double gauss_d1, int valid_voxel_num,
														double *score_gradients);

extern "C" __global__ void computeScoreGradientListS2(float *trans_x, float *trans_y, float *trans_z,
														int *valid_points,
														int *starting_voxel_id, int *voxel_id, int valid_points_num,
														double *centroid, int voxel_num, double *e_x_cov_x,
														double *cov_dxd_pi, double gauss_d1, int valid_voxel_num,
														double *score_gradients);

extern "C" __global__ void computeExCovX(float *trans_x, float *trans_y, float *trans_z, int *valid_points,
											int *starting_voxel_id, int *voxel_id, int valid_points_num,
											double *centroid, double *inverse_covariance, int voxel_num,
											double gauss_d1, double gauss_d2,
											double *e_x_cov_x);

extern "C" __global__ void updateExCovX(double *e_x_cov_x, double gauss_d2, int valid_voxel_num);

extern "C" __global__ void computeCovDxdPi(int *valid_points, int *starting_voxel_id, int *voxel_id, int valid_points_num,
											double *inverse_covariance, int voxel_num,
											double gauss_d1, double gauss_d2, double *point_gradients,
											double *cov_dxd_pi, int valid_voxel_num);

extern "C" __global__ void computeHessianListS0(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients,
													double *tmp_hessian, int valid_voxel_num);

extern "C" __global__ void computeHessianListS1(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients,
													double *tmp_hessian, int valid_voxel_num);

extern "C" __global__ void computeHessianListS2(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients,
													double *tmp_hessian, int valid_voxel_num);

extern "C" __global__ void computeHessianListS3(float *trans_x, float *trans_y, float *trans_z,
												int *valid_points,
												int *starting_voxel_id, int *voxel_id, int valid_points_num,
												double *centroid, int voxel_num,
												double gauss_d1, double gauss_d2,
												double *hessians,
												double *e_x_cov_x, double *tmp_hessian, double *cov_dxd_pi,
												int valid_voxel_num);

extern "C" __global__ void computeHessianListS4(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_hessians, double *hessians,
													double *e_x_cov_x);

extern "C" __global__ void computeHessianListS5(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_hessians, double *hessians,
													double *e_x_cov_x);

extern "C" __global__ void computeHessianListS6(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_hessians, double *hessians,
													double *e_x_cov_x);

extern "C" __global__ void computeHessianListS7(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients, double *hessians,
													double *e_x_cov_x,
													double *cov_dxd_pi, int valid_voxel_num);

extern "C" __global__ void matrixSum(double *matrix_list, int full_size, int half_size, int rows, int cols, int offset);

extern "C" __global__ void sumScore(double *score, int full_size, int half_size);

extern "C" __global__ void gpuTransform(float *in_x, float *in_y, float *in_z,
										float *trans_x, float *trans_y, float *trans_z,
										int point_num, MatrixDevice transform);

template <typename T>
__global__ void gpuSum(T *input, int size, int half_size);



































}
