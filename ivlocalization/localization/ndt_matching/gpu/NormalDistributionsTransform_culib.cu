#include "NormalDistributionsTransform.h"
#include "debug.h"
#include <cmath>
#include <iostream>
#include <pcl/common/transforms.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>

#include <ros/ros.h>

#include "NormalDistributionsTransform_culib.cuh"
namespace gpu {

/* First step of computing point gradients */
extern "C" __global__ void computePointGradients0(float *x, float *y, float *z, int points_num,
													int *valid_points, int valid_points_num,
													MatrixDevice j_ang_a, MatrixDevice j_ang_b, MatrixDevice j_ang_c, MatrixDevice j_ang_d,
													double *point_gradients)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		double o_x = static_cast<double>(x[pid]);
		double o_y = static_cast<double>(y[pid]);
		double o_z = static_cast<double>(z[pid]);

		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);		//3x6 Matrix

		//Set the 3x3 block start from (0, 0) to identity matrix
		pg(0, 0) = 1;
		pg(1, 1) = 1;
		pg(2, 2) = 1;

		//Compute point derivatives
		pg(1, 3) = o_x * j_ang_a(0) + o_y * j_ang_a(1) + o_z * j_ang_a(2);
		pg(2, 3) = o_x * j_ang_b(0) + o_y * j_ang_b(1) + o_z * j_ang_b(2);
		pg(0, 4) = o_x * j_ang_c(0) + o_y * j_ang_c(1) + o_z * j_ang_c(2);
		pg(1, 4) = o_x * j_ang_d(0) + o_y * j_ang_d(1) + o_z * j_ang_d(2);
	}
}


/* Second step of computing point gradients */
extern "C" __global__ void computePointGradients1(float *x, float *y, float *z, int points_num,
										int *valid_points, int valid_points_num,
										MatrixDevice j_ang_e, MatrixDevice j_ang_f, MatrixDevice j_ang_g, MatrixDevice j_ang_h,
										double *point_gradients)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		double o_x = static_cast<double>(x[pid]);
		double o_y = static_cast<double>(y[pid]);
		double o_z = static_cast<double>(z[pid]);

		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);		//3x6 Matrix

		//Compute point derivatives
		pg(2, 4) = o_x * j_ang_e(0) + o_y * j_ang_e(1) + o_z * j_ang_e(2);
		pg(0, 5) = o_x * j_ang_f(0) + o_y * j_ang_f(1) + o_z * j_ang_f(2);
		pg(1, 5) = o_x * j_ang_g(0) + o_y * j_ang_g(1) + o_z * j_ang_g(2);
		pg(2, 5) = o_x * j_ang_h(0) + o_y * j_ang_h(1) + o_z * j_ang_h(2);
	}
}

/* First step of computing point hessians */
extern "C" __global__ void computePointHessian0(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												MatrixDevice h_ang_a2, MatrixDevice h_ang_a3,
												MatrixDevice h_ang_b2, MatrixDevice h_ang_b3,
												double *point_hessians)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		double o_x = static_cast<double>(x[pid]);
		double o_y = static_cast<double>(y[pid]);
		double o_z = static_cast<double>(z[pid]);

		MatrixDevice ph(18, 6, valid_points_num, point_hessians + i);		//18x6 Matrix

		ph(9, 3) = 0;
		ph(10, 3) = o_x * h_ang_a2(0) + o_y * h_ang_a2(1) + o_z * h_ang_a2(2);
		ph(11, 3) = o_x * h_ang_a3(0) + o_y * h_ang_a3(1) + o_z * h_ang_a3(2);

		ph(12, 3) = ph(9, 4) = 0;
		ph(13, 3) = ph(10, 4) = o_x * h_ang_b2(0) + o_y * h_ang_b2(1) + o_z * h_ang_b2(2);
		ph(14, 3) = ph(11, 4) = o_x * h_ang_b3(0) + o_y * h_ang_b3(1) + o_z * h_ang_b3(2);

	}
}

/* Second step of computing point hessians */
extern "C" __global__ void computePointHessian1(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												MatrixDevice h_ang_c2, MatrixDevice h_ang_c3,
												MatrixDevice h_ang_d1, MatrixDevice h_ang_d2,
												MatrixDevice h_ang_d3,
												double *point_hessians)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		double o_x = static_cast<double>(x[pid]);
		double o_y = static_cast<double>(y[pid]);
		double o_z = static_cast<double>(z[pid]);

		MatrixDevice ph(18, 6, valid_points_num, point_hessians + i);		//18x6 Matrix

		ph(15, 3) = ph(9, 5) = 0;
		ph(16, 3) = ph(10, 5) = o_x * h_ang_c2(0) + o_y * h_ang_c2(1) + o_z * h_ang_c2(2);
		ph(17, 3) = ph(11, 5) = o_x * h_ang_c3(0) + o_y * h_ang_c3(1) + o_z * h_ang_c3(2);

		ph(12, 4) = o_x * h_ang_d1(0) + o_y * h_ang_d1(1) + o_z * h_ang_d1(2);
		ph(13, 4) = o_x * h_ang_d2(0) + o_y * h_ang_d2(1) + o_z * h_ang_d2(2);
		ph(14, 4) = o_x * h_ang_d3(0) + o_y * h_ang_d3(1) + o_z * h_ang_d3(2);
	}
}

/* Final step of computing point hessians */
extern "C" __global__ void computePointHessian2(float *x, float *y, float *z, int points_num,
												int *valid_points, int valid_points_num,
												MatrixDevice h_ang_e1, MatrixDevice h_ang_e2, MatrixDevice h_ang_e3,
												MatrixDevice h_ang_f1, MatrixDevice h_ang_f2, MatrixDevice h_ang_f3,
												double *point_hessians)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		//Orignal coordinates
		double o_x = static_cast<double>(x[pid]);
		double o_y = static_cast<double>(y[pid]);
		double o_z = static_cast<double>(z[pid]);

		MatrixDevice ph(18, 6, valid_points_num, point_hessians + i);		//18x6 Matrix

		ph(15, 4) = ph(12, 5) = o_x * h_ang_e1(0) + o_y * h_ang_e1(1) + o_z * h_ang_e1(2);
		ph(16, 4) = ph(13, 5) = o_x * h_ang_e2(0) + o_y * h_ang_e2(1) + o_z * h_ang_e2(2);
		ph(17, 4) = ph(14, 5) = o_x * h_ang_e3(0) + o_y * h_ang_e3(1) + o_z * h_ang_e3(2);

		ph(15, 5) = o_x * h_ang_f1(0) + o_y * h_ang_f1(1) + o_z * h_ang_f1(2);
		ph(16, 5) = o_x * h_ang_f2(0) + o_y * h_ang_f2(1) + o_z * h_ang_f2(2);
		ph(17, 5) = o_x * h_ang_f3(0) + o_y * h_ang_f3(1) + o_z * h_ang_f3(2);
	}
}

/* compute score_inc list for input points.
 * The final score_inc is calculated by a reduction sum
 * on this score_inc list. */
extern "C" __global__ void computeScoreList(int *starting_voxel_id, int *voxel_id, int valid_points_num,
												double *e_x_cov_x, double gauss_d1, double *score)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {

		double score_inc = 0;

		for (int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double tmp_ex = e_x_cov_x[vid];

			score_inc += (tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex) ? 0 : -gauss_d1 * tmp_ex;
		}

		score[i] = score_inc;
	}
}

/* First step to compute score gradient list for input points */
extern "C" __global__ void computeScoreGradientList(float *trans_x, float *trans_y, float *trans_z,
														int *valid_points,
														int *starting_voxel_id, int *voxel_id, int valid_points_num,
														double *centroid, int voxel_num, double *e_x_cov_x,
														double *cov_dxd_pi, double gauss_d1, int valid_voxel_num,
														double *score_gradients)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	for (int i = id; i < valid_points_num && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice sg(6, 1, valid_points_num, score_gradients + i);		//6x1 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double t_x = static_cast<double>(trans_x[pid]);
			double t_y = static_cast<double>(trans_y[pid]);
			double t_z = static_cast<double>(trans_z[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[vid]);
			MatrixDevice cov_dxd_pi_mat(3, 6, valid_voxel_num, cov_dxd_pi + vid);

			t_x -= centr(0);
			t_y -= centr(1);
			t_z -= centr(2);

			double tmp_ex = e_x_cov_x[vid];

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;

				sg(col) += (t_x * cov_dxd_pi_mat(0, col) + t_y * cov_dxd_pi_mat(1, col) + t_z * cov_dxd_pi_mat(2, col)) * tmp_ex;
			}
		}
	}
}

/* Second step to compute score gradient list */
extern "C" __global__ void computeScoreGradientListS2(float *trans_x, float *trans_y, float *trans_z,
														int *valid_points,
														int *starting_voxel_id, int *voxel_id, int valid_points_num,
														double *centroid, int voxel_num, double *e_x_cov_x,
														double *cov_dxd_pi, double gauss_d1, int valid_voxel_num,
														double *score_gradients)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		MatrixDevice sg(6, 1, valid_points_num, score_gradients + i);		//6x1 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double t_x = static_cast<double>(trans_x[pid]);
			double t_y = static_cast<double>(trans_y[pid]);
			double t_z = static_cast<double>(trans_z[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[vid]);
			MatrixDevice cov_dxd_pi_mat(3, 6, valid_voxel_num, cov_dxd_pi + vid);

			t_x -= centr(0);
			t_y -= centr(1);
			t_z -= centr(2);

			double tmp_ex = e_x_cov_x[vid];

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;

				for (int col = 0; col < 6; col++)
					sg(col) += (t_x * cov_dxd_pi_mat(0, col) + t_y * cov_dxd_pi_mat(1, col) + t_z * cov_dxd_pi_mat(2, col)) * tmp_ex;
			}
		}
	}
}

/* Intermediate step to compute e_x_cov_x */
extern "C" __global__ void computeExCovX(float *trans_x, float *trans_y, float *trans_z, int *valid_points,
											int *starting_voxel_id, int *voxel_id, int valid_points_num,
											double *centroid, double *inverse_covariance, int voxel_num,
											double gauss_d1, double gauss_d2,
											double *e_x_cov_x)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num; i += stride) {
		int pid = valid_points[i];

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double t_x = static_cast<double>(trans_x[pid]);
			double t_y = static_cast<double>(trans_y[pid]);
			double t_z = static_cast<double>(trans_z[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[vid]);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + voxel_id[vid]);	//3x3 matrix

			t_x -= centr(0);
			t_y -= centr(1);
			t_z -= centr(2);

			e_x_cov_x[vid] =  exp(-gauss_d2 * ((t_x * icov(0, 0) + t_y * icov(0, 1) + t_z * icov(0, 2)) * t_x
										+ ((t_x * icov(1, 0) + t_y * icov(1, 1) + t_z * icov(1, 2)) * t_y)
										+ ((t_x * icov(2, 0) + t_y * icov(2, 1) + t_z * icov(2, 2)) * t_z)) / 2.0);
		}
	}
}



/* update e_x_cov_x - Reusable portion of Equation 6.12 and 6.13 [Magnusson 2009] */
extern "C" __global__ void updateExCovX(double *e_x_cov_x, double gauss_d2, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_voxel_num; i += stride) {
		e_x_cov_x[i] *= gauss_d2;
	}
}

/* compute cov_dxd_pi as reusable portion of Equation 6.12 and 6.13 [Magnusson 2009]*/
extern "C" __global__ void computeCovDxdPi(int *valid_points, int *starting_voxel_id, int *voxel_id, int valid_points_num,
											double *inverse_covariance, int voxel_num,
											double gauss_d1, double gauss_d2, double *point_gradients,
											double *cov_dxd_pi, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = id; i < valid_points_num && row < 3 && col < 6; i += stride) {
		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);		//3x6 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + voxel_id[vid]);	//3x3 matrix
			MatrixDevice cov_dxd_pi_mat(3, 6, valid_voxel_num, cov_dxd_pi + vid);

			cov_dxd_pi_mat(row, col) = icov(row, 0) * pg(0, col) + icov(row, 1) * pg(1, col) + icov(row, 2) * pg(2, col);
		}
	}
}

/* First step to compute hessian list for input points */
extern "C" __global__ void computeHessianListS0(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients,
													double *tmp_hessian, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	for (int i = id; i < valid_points_num && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);		//3x6 Matrix

		for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
			//Transformed coordinates
			int vid = voxel_id[j];
			double t_x = static_cast<double>(trans_x[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + vid);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + vid);	//3x3 matrix
			MatrixDevice tmp(6, 1, valid_voxel_num, tmp_hessian + j);

			t_x -= centr(0);

			tmp(col) = t_x * (icov(0, 0) * pg(0, col) + icov(0, 1) * pg(1, col) + icov(0, 2) * pg(2, col));
		}
	}
}

/* Second step to compute hessian list for input points */
extern "C" __global__ void computeHessianListS1(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients,
													double *tmp_hessian, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	for (int i = id; i < valid_points_num && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);		//3x6 Matrix

		for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
			//Transformed coordinates
			int vid = voxel_id[j];
			double t_y = static_cast<double>(trans_y[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + vid);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + vid);	//3x3 matrix
			MatrixDevice tmp(6, 1, valid_voxel_num, tmp_hessian + j);

			t_y -= centr(1);

			tmp(col) += t_y * (icov(1, 0) * pg(0, col) + icov(1, 1) * pg(1, col) + icov(1, 2) * pg(2, col));
		}
	}
}


/* Third step to compute hessian list for input points */
extern "C" __global__ void computeHessianListS2(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients,
													double *tmp_hessian, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int col = blockIdx.y;

	for (int i = id; i < valid_points_num && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);		//3x6 Matrix

		for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
			//Transformed coordinates
			int vid = voxel_id[j];
			double t_z = static_cast<double>(trans_z[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + vid);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + vid);	//3x3 matrix
			MatrixDevice tmp(6, 1, valid_voxel_num, tmp_hessian + j);

			t_z -= centr(2);

			tmp(col) += t_z * (icov(2, 0) * pg(0, col) + icov(2, 1) * pg(1, col) + icov(2, 2) * pg(2, col));
		}
	}
}

/* Fourth step to compute hessian list */
extern "C" __global__ void computeHessianListS3(float *trans_x, float *trans_y, float *trans_z,
												int *valid_points,
												int *starting_voxel_id, int *voxel_id, int valid_points_num,
												double *centroid, int voxel_num,
												double gauss_d1, double gauss_d2,
												double *hessians,
												double *e_x_cov_x, double *tmp_hessian, double *cov_dxd_pi,
												int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = id; i < valid_points_num && row < 6 && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice h(6, 6, valid_points_num, hessians + i);				//6x6 Matrix

		for ( int j = starting_voxel_id[i]; j < starting_voxel_id[i + 1]; j++) {
			//Transformed coordinates
			double t_x = static_cast<double>(trans_x[pid]);
			double t_y = static_cast<double>(trans_y[pid]);
			double t_z = static_cast<double>(trans_z[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[j]);
			MatrixDevice tmp(6, 1, valid_voxel_num, tmp_hessian + j);
			MatrixDevice cov_dxd_pi_mat(3, 6, valid_voxel_num, cov_dxd_pi + j);

			t_x -= centr(0);
			t_y -= centr(1);
			t_z -= centr(2);

			double tmp_ex, tmp_cov_dxd_x, tmp_cov_dxd_y, tmp_cov_dxd_z;

			tmp_ex = e_x_cov_x[j];

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;

				tmp_cov_dxd_x = cov_dxd_pi_mat(0, row);
				tmp_cov_dxd_y = cov_dxd_pi_mat(1, row);
				tmp_cov_dxd_z = cov_dxd_pi_mat(2, row);

				h(row, col) += -gauss_d2 * (t_x * tmp_cov_dxd_x + t_y * tmp_cov_dxd_y + t_z * tmp_cov_dxd_z) * tmp(col) * tmp_ex;
			}
		}
	}
}

/* Fifth step to compute hessian list */
extern "C" __global__ void computeHessianListS4(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_hessians, double *hessians,
													double *e_x_cov_x)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = id; i < valid_points_num && row < 6 && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice ph(18, 6, valid_points_num, point_hessians + i);		//18x6 Matrix
		MatrixDevice h(6, 6, valid_points_num, hessians + i);				//6x6 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double t_x = static_cast<double>(trans_x[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[vid]);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + voxel_id[vid]);	//3x3 matrix

			t_x -= centr(0);

			double tmp_ex = e_x_cov_x[vid];	//e_x_cov_x

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;
						//Compute hessian
				h(row, col) += t_x * (icov(0, 0) * ph(3 * row, col) + icov(0, 1) * ph(3 * row + 1, col) + icov(0, 2) * ph(3 * row + 2, col)) * tmp_ex;
			}
		}
	}
}

/* Sixth step to compute hessian list */
extern "C" __global__ void computeHessianListS5(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_hessians, double *hessians,
													double *e_x_cov_x)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;
	int stride = blockDim.x * gridDim.x;

	for (int i = id; i < valid_points_num && row < 6 && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice ph(18, 6, valid_points_num, point_hessians + i);		//18x6 Matrix
		MatrixDevice h(6, 6, valid_points_num, hessians + i);				//6x6 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double t_y = static_cast<double>(trans_y[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[vid]);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + voxel_id[vid]);	//3x3 matrix

			t_y -= centr(1);

			double tmp_ex = e_x_cov_x[vid];	//e_x_cov_x

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;
				//Compute hessian
				h(row, col) += t_y * (icov(1, 0) * ph(3 * row, col) + icov(1, 1) * ph(3 * row + 1, col) + icov(1, 2) * ph(3 * row + 2, col)) * tmp_ex;
			}
		}
	}
}

/* Seventh step to compute hessian list */
extern "C" __global__ void computeHessianListS6(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double *centroid, double *inverse_covariance, int voxel_num,
													double gauss_d1, double gauss_d2,
													double *point_hessians, double *hessians,
													double *e_x_cov_x)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = id; i < valid_points_num && row < 6 && col < 6; i += stride) {
		int pid = valid_points[i];

		MatrixDevice ph(18, 6, valid_points_num, point_hessians + i);		//18x6 Matrix
		MatrixDevice h(6, 6, valid_points_num, hessians + i);				//6x6 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			//Transformed coordinates
			double t_z = static_cast<double>(trans_z[pid]);
			MatrixDevice centr(3, 1, voxel_num, centroid + voxel_id[vid]);
			MatrixDevice icov(3, 3, voxel_num, inverse_covariance + voxel_id[vid]);	//3x3 matrix

			t_z -= centr(2);

			double tmp_ex = e_x_cov_x[vid];	//e_x_cov_x

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;
				//Compute hessian
				h(row, col) += t_z * (icov(2, 0) * ph(3 * row, col) + icov(2, 1) * ph(3 * row + 1, col) + icov(2, 2) * ph(3 * row + 2, col)) * tmp_ex;
			}
		}
	}
}

/* Eighth step to compute hessian list */
extern "C" __global__ void computeHessianListS7(float *trans_x, float *trans_y, float *trans_z,
													int *valid_points,
													int *starting_voxel_id, int *voxel_id, int valid_points_num,
													double gauss_d1, double gauss_d2,
													double *point_gradients, double *hessians,
													double *e_x_cov_x,
													double *cov_dxd_pi, int valid_voxel_num)
{
	int id = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = id; i < valid_points_num && row < 6 && col < 6; i += stride) {
		MatrixDevice pg(3, 6, valid_points_num, point_gradients + i);
		MatrixDevice h(6, 6, valid_points_num, hessians + i);				//6x6 Matrix

		for ( int vid = starting_voxel_id[i]; vid < starting_voxel_id[i + 1]; vid++) {
			MatrixDevice cov_dxd_pi_mat(3, 6, valid_voxel_num, cov_dxd_pi + vid);
			//Transformed coordinates
			double tmp_ex = e_x_cov_x[vid];
			double tmp_cov_dxd_x, tmp_cov_dxd_y, tmp_cov_dxd_z;

			if (!(tmp_ex > 1 || tmp_ex < 0 || tmp_ex != tmp_ex)) {
				tmp_ex *= gauss_d1;

				tmp_cov_dxd_x = cov_dxd_pi_mat(0, row);
				tmp_cov_dxd_y = cov_dxd_pi_mat(1, row);
				tmp_cov_dxd_z = cov_dxd_pi_mat(2, row);

				//Compute hessian
				h(row, col) += (pg(0, col) * tmp_cov_dxd_x + pg(1, col) * tmp_cov_dxd_y + pg(2, col) * tmp_cov_dxd_z) * tmp_ex;
			}
		}
	}
}

/* Compute sum of a list of matrices */
extern "C" __global__ void matrixSum(double *matrix_list, int full_size, int half_size, int rows, int cols, int offset)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	int row = blockIdx.y;
	int col = blockIdx.z;

	for (int i = index; i < half_size && row < rows && col < cols; i += stride) {
		MatrixDevice left(rows, cols, offset, matrix_list + i);
		double *right_ptr = (i + half_size < full_size) ? matrix_list + i + half_size : NULL;
		MatrixDevice right(rows, cols, offset, right_ptr);

		if (right_ptr != NULL) {
			left(row, col) += right(row, col);
		}
	}
}

/* Compute sum of score_inc list */
extern "C" __global__ void sumScore(double *score, int full_size, int half_size)
{
	int index = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = index; i < half_size; i += stride) {
		score[i] += (i + half_size < full_size) ? score[i + half_size] : 0;
	}
}


extern "C" __global__ void gpuTransform(float *in_x, float *in_y, float *in_z,
										float *trans_x, float *trans_y, float *trans_z,
										int point_num, MatrixDevice transform)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;
	float x, y, z;

	for (int i = idx; i < point_num; i += stride) {
		x = in_x[i];
		y = in_y[i];
		z = in_z[i];
		trans_x[i] = transform(0, 0) * x + transform(0, 1) * y + transform(0, 2) * z + transform(0, 3);
		trans_y[i] = transform(1, 0) * x + transform(1, 1) * y + transform(1, 2) * z + transform(1, 3);
		trans_z[i] = transform(2, 0) * x + transform(2, 1) * y + transform(2, 2) * z + transform(2, 3);
	}
}


}
