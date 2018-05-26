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
#define V2_ 1
#define FOUR_COUNT 1
std::chrono::time_point<std::chrono::system_clock> debugtime;
double debugtime_ans = 0.0;
namespace gpu {

GNormalDistributionsTransform::GNormalDistributionsTransform()
{
	GRegistration::GRegistration();

	gauss_d1_ = gauss_d2_ = 0;
	outlier_ratio_ = 0.55;
	step_size_ = 0.1;
	resolution_ = 1.0;
//	resolution_ = 1.0f;
	trans_probability_ = 0;

	double gauss_c1, gauss_c2, gauss_d3;

	// Initializes the guassian fitting parameters (eq. 6.8) [Magnusson 2009]
	gauss_c1 = 10.0 * (1 - outlier_ratio_);
	gauss_c2 = outlier_ratio_ / pow (resolution_, 3);
	gauss_d3 = -log (gauss_c2);
	gauss_d1_ = -log ( gauss_c1 + gauss_c2 ) - gauss_d3;
	gauss_d2_ = -2 * log ((-log ( gauss_c1 * exp ( -0.5 ) + gauss_c2 ) - gauss_d3) / gauss_d1_);

	transformation_epsilon_ = 0.1;
	max_iterations_ = 35;

	j_ang_a_ = MatrixHost(3, 1);
	j_ang_b_ = MatrixHost(3, 1);
	j_ang_c_ = MatrixHost(3, 1);
	j_ang_d_ = MatrixHost(3, 1);
	j_ang_e_ = MatrixHost(3, 1);
	j_ang_f_ = MatrixHost(3, 1);
	j_ang_g_ = MatrixHost(3, 1);
	j_ang_h_ = MatrixHost(3, 1);

	h_ang_a2_ = MatrixHost(3, 1);
	h_ang_a3_ = MatrixHost(3, 1);
	h_ang_b2_ = MatrixHost(3, 1);
	h_ang_b3_ = MatrixHost(3, 1);
	h_ang_c2_ = MatrixHost(3, 1);
	h_ang_c3_ = MatrixHost(3, 1);
	h_ang_d1_ = MatrixHost(3, 1);
	h_ang_d2_ = MatrixHost(3, 1);
	h_ang_d3_ = MatrixHost(3, 1);
	h_ang_e1_ = MatrixHost(3, 1);
	h_ang_e2_ = MatrixHost(3, 1);
	h_ang_e3_ = MatrixHost(3, 1);
	h_ang_f1_ = MatrixHost(3, 1);
	h_ang_f2_ = MatrixHost(3, 1);
	h_ang_f3_ = MatrixHost(3, 1);

	dj_ang_a_ = MatrixDevice(3, 1);
	dj_ang_b_ = MatrixDevice(3, 1);
	dj_ang_c_ = MatrixDevice(3, 1);
	dj_ang_d_ = MatrixDevice(3, 1);
	dj_ang_e_ = MatrixDevice(3, 1);
	dj_ang_f_ = MatrixDevice(3, 1);
	dj_ang_g_ = MatrixDevice(3, 1);
	dj_ang_h_ = MatrixDevice(3, 1);

	dh_ang_a2_ = MatrixDevice(3, 1);
	dh_ang_a3_ = MatrixDevice(3, 1);
	dh_ang_b2_ = MatrixDevice(3, 1);
	dh_ang_b3_ = MatrixDevice(3, 1);
	dh_ang_c2_ = MatrixDevice(3, 1);
	dh_ang_c3_ = MatrixDevice(3, 1);
	dh_ang_d1_ = MatrixDevice(3, 1);
	dh_ang_d2_ = MatrixDevice(3, 1);
	dh_ang_d3_ = MatrixDevice(3, 1);
	dh_ang_e1_ = MatrixDevice(3, 1);
	dh_ang_e2_ = MatrixDevice(3, 1);
	dh_ang_e3_ = MatrixDevice(3, 1);
	dh_ang_f1_ = MatrixDevice(3, 1);
	dh_ang_f2_ = MatrixDevice(3, 1);
	dh_ang_f3_ = MatrixDevice(3, 1);

	real_iterations_ = 0;
}

GNormalDistributionsTransform::~GNormalDistributionsTransform()
{
	dj_ang_a_.memFree();
	dj_ang_b_.memFree();
	dj_ang_c_.memFree();
	dj_ang_d_.memFree();
	dj_ang_e_.memFree();
	dj_ang_f_.memFree();
	dj_ang_g_.memFree();
	dj_ang_h_.memFree();

	dh_ang_a2_.memFree();
	dh_ang_a3_.memFree();
	dh_ang_b2_.memFree();
	dh_ang_b3_.memFree();
	dh_ang_c2_.memFree();
	dh_ang_c3_.memFree();
	dh_ang_d1_.memFree();
	dh_ang_d2_.memFree();
	dh_ang_d3_.memFree();
	dh_ang_e1_.memFree();
	dh_ang_e2_.memFree();
	dh_ang_e3_.memFree();
	dh_ang_f1_.memFree();
	dh_ang_f2_.memFree();
	dh_ang_f3_.memFree();

	voxel_grid_.~GVoxelGrid();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
	void GNormalDistributionsTransform::setStepSize(double step_size)
	{
		step_size_ = step_size;
	}

	void GNormalDistributionsTransform::setResolution(float resolution)
	{
//fprintf(stderr, "setresolution to %f **********************************************************\n", resolution);
		resolution_ = resolution;
//fprintf(stderr, "resolotion_ = %f **********************************************************\n", resolution_);
	}

	void GNormalDistributionsTransform::setOutlierRatio(double olr)
	{
		outlier_ratio_ = olr;
	}

	double GNormalDistributionsTransform::getStepSize()
	{
		return step_size_;
	}

	float GNormalDistributionsTransform::getResolution()
	{
		return resolution_;
	}

	double GNormalDistributionsTransform::getOutlierRatio()
	{
		return outlier_ratio_;
	}

	double GNormalDistributionsTransform::getTransformationProbability()
	{
		return trans_probability_;
	}

	int GNormalDistributionsTransform::getRealIterations() { return real_iterations_; }
///////////////////////////////////////////////////////////////////////////////////////////////

template <typename T>
__global__ void gpuSum(T *input, int size, int half_size)
{
	int idx = threadIdx.x + blockIdx.x * blockDim.x;
	int stride = blockDim.x * gridDim.x;

	for (int i = idx; i < half_size; i += stride) {
		input[i] += (half_size < size) ? input[i + half_size] : 0;
	}
}

void GNormalDistributionsTransform::setInputTarget(pcl::PointCloud<pcl::PointXYZI>::Ptr input)
{
	// Copy input map data from the host memory to the GPU memory
	GRegistration::setInputTarget(input);

	// Build the voxel grid
	if (target_points_number_ != 0) {
		voxel_grid_.setLeafSize(resolution_, resolution_, resolution_);
		voxel_grid_.setInput(target_x_, target_y_, target_z_, target_points_number_);
	}
}

void GNormalDistributionsTransform::setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	// Copy input map data from the host memory to the GPU memory
	GRegistration::setInputTarget(input);

	// Build the voxel grid
	if (target_points_number_ != 0) {
		voxel_grid_.setLeafSize(1.0, 1.0, 1.0);
		voxel_grid_.setInput(target_x_, target_y_, target_z_, target_points_number_);
	}
}

void GNormalDistributionsTransform::computeTransformation(Eigen::Matrix<float, 4, 4> &guess)
{
	struct timeval start, end;

	nr_iterations_ = 0;
	converged_ = false;

	double gauss_c1, gauss_c2, gauss_d3;

	gauss_c1 = 10 * ( 1 - outlier_ratio_);
	gauss_c2 = outlier_ratio_ / pow(resolution_, 3);
	gauss_d3 = - log(gauss_c2);
	gauss_d1_ = -log(gauss_c1 + gauss_c2) - gauss_d3;
	gauss_d2_ = -2 * log((-log(gauss_c1 * exp(-0.5) + gauss_c2) - gauss_d3) / gauss_d1_);
	if (guess != Eigen::Matrix4f::Identity()) {
		final_transformation_ = guess;
		transformPointCloud(x_, y_, z_, trans_x_, trans_y_, trans_z_, points_number_, guess);
	}
	Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> eig_transformation;
	eig_transformation.matrix() = final_transformation_;

	Eigen::Matrix<double, 6, 1> p, delta_p, score_gradient;
	Eigen::Vector3f init_translation = eig_transformation.translation();

	tfScalar roll_, pitch_, yaw_;
    
	tf::Matrix3x3 mat_l;  // localizer
        mat_l.setValue(static_cast<double>(final_transformation_(0, 0)), static_cast<double>(final_transformation_(0, 1)), static_cast<double>(final_transformation_(0, 2)),
                   static_cast<double>(final_transformation_(1, 0)), static_cast<double>(final_transformation_(1, 1)), static_cast<double>(final_transformation_(1, 2)),
                   static_cast<double>(final_transformation_(2, 0)), static_cast<double>(final_transformation_(2, 1)), static_cast<double>(final_transformation_(2, 2)));
	mat_l.getRPY(roll_, pitch_, yaw_, 1);

	p << init_translation(0), init_translation(1), init_translation(2), roll_, pitch_, yaw_;
	Eigen::Matrix<double, 6, 6> hessian;

	double score = 0;
	double delta_p_norm;

	gettimeofday(&start, NULL);
	score = computeDerivatives(score_gradient, hessian, trans_x_, trans_y_, trans_z_, points_number_, p);
	gettimeofday(&end, NULL);
	gettimeofday(&start, NULL);
	while (!converged_) {
		previous_transformation_ = transformation_;
		
		Eigen::JacobiSVD<Eigen::Matrix<double, 6, 6>> sv(hessian, Eigen::ComputeFullU | Eigen::ComputeFullV);

		delta_p = sv.solve(-score_gradient);

		delta_p_norm = delta_p.norm();

		if (delta_p_norm == 0 || delta_p_norm != delta_p_norm) {

			trans_probability_ = score / static_cast<double>(points_number_);
			converged_ = delta_p_norm == delta_p_norm;
			return;
		}

		delta_p.normalize();

		delta_p_norm = computeStepLengthMT(p, delta_p, delta_p_norm, step_size_, 
			transformation_epsilon_ / 2, score, score_gradient, hessian, 
			trans_x_, trans_y_, trans_z_, points_number_);
		delta_p *= delta_p_norm;

#ifdef FOUR_COUNT
Eigen::Matrix3f R;  
    R = Eigen::AngleAxis<float>(static_cast<float>(delta_p(3)), Eigen::Vector3f::UnitX())  
        * Eigen::AngleAxis<float>(static_cast<float>(delta_p(4)), Eigen::Vector3f::UnitY())  
        * Eigen::AngleAxis<float>(static_cast<float>(delta_p(5)), Eigen::Vector3f::UnitZ());  
    Eigen::Quaternionf q;  
    q = R;

    double a[3][3];
	a[0][0] = 1-2*(q.y()*q.y()+q.z()*q.z());
	a[0][1] = 2*(q.x()*q.y()-q.z()*q.w());
	a[0][2] = 2*(q.x()*q.z()+q.y()*q.w());
	a[1][0] = 2*(q.x()*q.y()+q.z()*q.w());
	a[1][1] = 1-2*(q.x()*q.x()+q.z()*q.z());
	a[1][2] = 2*(q.y()*q.z()-q.x()*q.w());
	a[2][0] = 2*(q.x()*q.z()-q.y()*q.w());
	a[2][1] = 2*(q.y()*q.z()+q.x()*q.w());
	a[2][2] = 1-2*(q.x()*q.x()+q.y()*q.y());

transformation_ << a[0][0],a[0][1],a[0][2],static_cast<float>(delta_p(0)),a[1][0],a[1][1],a[1][2],static_cast<float>(delta_p(1)),
					a[2][0],a[2][1],a[2][2],static_cast<float>(delta_p(2)),0,0,0,1;
#endif

		p = p + delta_p;

		//Not update visualizer
		nr_iterations_++;
		if (nr_iterations_ > 15 || (nr_iterations_ && (std::fabs(delta_p_norm) < transformation_epsilon_)))//max_iterations_
			converged_ = true;
	}
	gettimeofday(&end, NULL);

	char buffer_1026[20];
	sprintf(buffer_1026,"%d:%d;\n",points_number_,nr_iterations_);
	save_debug_data(buffer_1026,0,1);

	trans_probability_ = score / static_cast<double>(points_number_);
}


double GNormalDistributionsTransform::computeDerivatives(Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
														float *trans_x, float *trans_y, float *trans_z,
														int points_num, Eigen::Matrix<double, 6, 1> pose, bool compute_hessian)
{
	MatrixHost p(6, 1);

	for (int i = 0; i < 6; i++) {
		p(i) = pose(i, 0);
	}

	score_gradient.setZero ();
	hessian.setZero ();

	//Compute Angle Derivatives
	computeAngleDerivatives(p);

	//Radius Search
	int *valid_points, *voxel_id, *starting_voxel_id;
	int valid_voxel_num, valid_points_num;

	valid_points = voxel_id = starting_voxel_id = NULL;


	voxel_grid_.radiusSearch(trans_x, trans_y, trans_z, points_num, resolution_, INT_MAX, &valid_points, &starting_voxel_id, &voxel_id, &valid_voxel_num, &valid_points_num);

	double *covariance = voxel_grid_.getCovarianceList();
	double *inverse_covariance = voxel_grid_.getInverseCovarianceList();
	double *centroid = voxel_grid_.getCentroidList();
	int *points_per_voxel = voxel_grid_.getPointsPerVoxelList();
	int voxel_num = voxel_grid_.getVoxelNum();

	if (valid_points_num == 0)
		return 0;

	//Update score gradient and hessian matrix

	double *gradients, *hessians, *point_gradients, *point_hessians, *score;

	checkCudaErrors(cudaMalloc(&gradients, sizeof(double) * valid_points_num * 6));
	checkCudaErrors(cudaMemset(gradients, 0, sizeof(double) * valid_points_num * 6));

	checkCudaErrors(cudaMalloc(&hessians, sizeof(double) * valid_points_num * 6 * 6));
	checkCudaErrors(cudaMemset(hessians, 0, sizeof(double) * valid_points_num * 6 * 6));

	checkCudaErrors(cudaMalloc(&point_gradients, sizeof(double) * valid_points_num * 3 * 6));
	checkCudaErrors(cudaMemset(point_gradients, 0, sizeof(double) * valid_points_num * 3 * 6));

	checkCudaErrors(cudaMalloc(&point_hessians, sizeof(double) * valid_points_num * 18 * 6));
	checkCudaErrors(cudaMemset(point_hessians, 0, sizeof(double) * valid_points_num * 18 * 6));

	checkCudaErrors(cudaMalloc(&score, sizeof(double) * valid_points_num));

	int block_x;
#ifdef USING_HP
	block_x = (valid_points_num > (BLOCK_SIZE_X)) ? (BLOCK_SIZE_X) : valid_points_num;//gpu_up
#else
	block_x = (valid_points_num > (BLOCK_SIZE_X/2)) ? (BLOCK_SIZE_X/2) : valid_points_num;
#endif
	int grid_x = (valid_points_num - 1) / block_x + 1;

	dim3 grid;
	computePointGradients0<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_a_, dj_ang_b_, dj_ang_c_, dj_ang_d_,
												point_gradients);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	computePointGradients1<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_e_, dj_ang_f_, dj_ang_g_, dj_ang_h_,
												point_gradients);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	if (compute_hessian) {
		computePointHessian0<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_a2_, dh_ang_a3_,
												dh_ang_b2_, dh_ang_b3_,
												point_hessians);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computePointHessian1<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_c2_, dh_ang_c3_,
												dh_ang_d1_, dh_ang_d2_, dh_ang_d3_,
												point_hessians);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computePointHessian2<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_e1_, dh_ang_e2_, dh_ang_e3_,
												dh_ang_f1_, dh_ang_f2_, dh_ang_f3_,
												point_hessians);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui
	}
	checkCudaErrors(cudaDeviceSynchronize());

	double *tmp_hessian;

	checkCudaErrors(cudaMalloc(&tmp_hessian, sizeof(double) * valid_voxel_num * 6));

	double *e_x_cov_x;

	checkCudaErrors(cudaMalloc(&e_x_cov_x, sizeof(double) * valid_voxel_num));

	double *cov_dxd_pi;

	checkCudaErrors(cudaMalloc(&cov_dxd_pi, sizeof(double) * valid_voxel_num * 3 * 6));

	computeExCovX<<<grid_x, block_x>>>(trans_x, trans_y, trans_z, valid_points,
										starting_voxel_id, voxel_id, valid_points_num,
										centroid, inverse_covariance, voxel_num,
										gauss_d1_, gauss_d2_,
										e_x_cov_x);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	computeScoreList<<<grid_x, block_x>>>(starting_voxel_id, voxel_id, valid_points_num, e_x_cov_x, gauss_d1_, score);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	int block_x2 = (valid_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_voxel_num;
	int grid_x2 = (valid_voxel_num - 1) / block_x2 + 1;

	updateExCovX<<<grid_x2, block_x2>>>(e_x_cov_x, gauss_d2_, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	grid.x = grid_x;
	grid.y = 3;
	grid.z = 6;

	computeCovDxdPi<<<grid, block_x>>>(valid_points, starting_voxel_id, voxel_id, valid_points_num,
											inverse_covariance, voxel_num,
											gauss_d1_, gauss_d2_, point_gradients,
											cov_dxd_pi, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	grid.x = grid_x;
	grid.y = 6;
	grid.z = 1;
	computeScoreGradientList<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, voxel_num, e_x_cov_x,
													cov_dxd_pi, gauss_d1_, valid_voxel_num, gradients);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());//added by panrui

	if (compute_hessian) {
		grid.y = 6;
		grid.z = 1;
		computeHessianListS0<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, inverse_covariance, voxel_num,
													gauss_d1_, gauss_d2_,
													point_gradients, tmp_hessian, valid_voxel_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computeHessianListS1<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, inverse_covariance, voxel_num,
													gauss_d1_, gauss_d2_,
													point_gradients, tmp_hessian, valid_voxel_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computeHessianListS2<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, inverse_covariance, voxel_num,
													gauss_d1_, gauss_d2_,
													point_gradients, tmp_hessian, valid_voxel_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		grid.z = 6;
		computeHessianListS3<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, voxel_num,
													gauss_d1_, gauss_d2_,
													hessians,
													e_x_cov_x, tmp_hessian, cov_dxd_pi,
													valid_voxel_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computeHessianListS4<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, inverse_covariance, voxel_num,
													gauss_d1_, gauss_d2_,
													point_hessians, hessians,
													e_x_cov_x);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computeHessianListS5<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
														starting_voxel_id, voxel_id, valid_points_num,
														centroid, inverse_covariance, voxel_num,
														gauss_d1_, gauss_d2_,
														point_hessians, hessians,
														e_x_cov_x);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computeHessianListS6<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
														starting_voxel_id, voxel_id, valid_points_num,
														centroid, inverse_covariance, voxel_num,
														gauss_d1_, gauss_d2_,
														point_hessians, hessians,
														e_x_cov_x);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		computeHessianListS7<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													gauss_d1_, gauss_d2_,
													point_gradients, hessians,
													e_x_cov_x, cov_dxd_pi, valid_voxel_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui
	}
	int full_size = valid_points_num;
	int half_size = (full_size - 1) / 2 + 1;
	while (full_size > 1) {
		block_x = (half_size > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_size;
		grid_x = (half_size - 1) / block_x + 1;

		grid.x = grid_x;
		grid.y = 1;
		grid.z = 6;
		matrixSum<<<grid, block_x>>>(gradients, full_size, half_size, 1, 6, valid_points_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		grid.y = 6;
		matrixSum<<<grid, block_x>>>(hessians, full_size, half_size, 6, 6, valid_points_num);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		sumScore<<<grid_x, block_x>>>(score, full_size, half_size);
		checkCudaErrors(cudaGetLastError());
//		checkCudaErrors(cudaDeviceSynchronize());//added by panrui

		full_size = half_size;
		half_size = (full_size - 1) / 2 + 1;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	MatrixDevice dgrad(1, 6, valid_points_num, gradients), dhess(6, 6, valid_points_num, hessians);
	MatrixHost hgrad(1, 6), hhess(6, 6);

	hgrad.moveToHost(dgrad);
	hhess.moveToHost(dhess);
	for (int i = 0; i < 6; i++) {
		score_gradient(i) = hgrad(i);
	}
	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			hessian(i, j) = hhess(i, j);
		}
	}

	double score_inc;

	checkCudaErrors(cudaMemcpy(&score_inc, score, sizeof(double), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(gradients));
	checkCudaErrors(cudaFree(hessians));
	checkCudaErrors(cudaFree(point_hessians));
	checkCudaErrors(cudaFree(point_gradients));
	checkCudaErrors(cudaFree(score));

	checkCudaErrors(cudaFree(tmp_hessian));

	checkCudaErrors(cudaFree(e_x_cov_x));
	checkCudaErrors(cudaFree(cov_dxd_pi));

	if (valid_points != NULL)
		checkCudaErrors(cudaFree(valid_points));

	if (voxel_id != NULL)
		checkCudaErrors(cudaFree(voxel_id));

	if (starting_voxel_id != NULL)
		checkCudaErrors(cudaFree(starting_voxel_id));

	return score_inc;
}

void GNormalDistributionsTransform::computeAngleDerivatives(MatrixHost pose, bool compute_hessian)
{
	double cx, cy, cz, sx, sy, sz;

	if (fabs(pose(3)) < 10e-5) {
		cx = 1.0;
		sx = 0.0;
	} else {
		cx = cos(pose(3));
		sx = sin(pose(3));
	}

	if (fabs(pose(4)) < 10e-5) {
		cy = 1.0;
		sy = 0.0;
	} else {
		cy = cos(pose(4));
		sy = sin(pose(4));
	}

	if (fabs(pose(5)) < 10e-5) {
		cz = 1.0;
		sz = 0.0;
	} else {
		cz = cos(pose(5));
		sz = sin(pose(5));
	}

	j_ang_a_(0) = -sx * sz + cx * sy * cz;
	j_ang_a_(1) = -sx * cz - cx * sy * sz;
	j_ang_a_(2) = -cx * cy;

	j_ang_b_(0) = cx * sz + sx * sy * cz;
	j_ang_b_(1) = cx * cz - sx * sy * sz;
	j_ang_b_(2) = -sx * cy;

	j_ang_c_(0) = -sy * cz;
	j_ang_c_(1) = sy * sz;
	j_ang_c_(2) = cy;

	j_ang_d_(0) = sx * cy * cz;
	j_ang_d_(1) = -sx * cy * sz;
	j_ang_d_(2) = sx * sy;

	j_ang_e_(0) = -cx * cy * cz;
	j_ang_e_(1) = cx * cy * sz;
	j_ang_e_(2) = -cx * sy;

	j_ang_f_(0) = -cy * sz;
	j_ang_f_(1) = -cy * cz;
	j_ang_f_(2) = 0;

	j_ang_g_(0) = cx * cz - sx * sy * sz;
	j_ang_g_(1) = -cx * sz - sx * sy * cz;
	j_ang_g_(2) = 0;

	j_ang_h_(0) = sx * cz + cx * sy * sz;
	j_ang_h_(1) = cx * sy * cz - sx * sz;
	j_ang_h_(2) = 0;

	j_ang_a_.moveToGpu(dj_ang_a_);
	j_ang_b_.moveToGpu(dj_ang_b_);
	j_ang_c_.moveToGpu(dj_ang_c_);
	j_ang_d_.moveToGpu(dj_ang_d_);
	j_ang_e_.moveToGpu(dj_ang_e_);
	j_ang_f_.moveToGpu(dj_ang_f_);
	j_ang_g_.moveToGpu(dj_ang_g_);
	j_ang_h_.moveToGpu(dj_ang_h_);

	if (compute_hessian) {
		h_ang_a2_(0) = -cx * sz - sx * sy * cz;
		h_ang_a2_(1) = -cx * cz + sx * sy * sz;
		h_ang_a2_(2) = sx * cy;

		h_ang_a3_(0) = -sx * sz + cx * sy * cz;
		h_ang_a3_(1) = -cx * sy * sz - sx * cz;
		h_ang_a3_(2) = -cx * cy;

		h_ang_b2_(0) = cx * cy * cz;
		h_ang_b2_(1) = -cx * cy * sz;
		h_ang_b2_(2) = cx * sy;

		h_ang_b3_(0) = sx * cy * cz;
		h_ang_b3_(1) = -sx * cy * sz;
		h_ang_b3_(2) = sx * sy;

		h_ang_c2_(0) = -sx * cz - cx * sy * sz;
		h_ang_c2_(1) = sx * sz - cx * sy * cz;
		h_ang_c2_(2) = 0;

		h_ang_c3_(0) = cx * cz - sx * sy * sz;
		h_ang_c3_(1) = -sx * sy * cz - cx * sz;
		h_ang_c3_(2) = 0;

		h_ang_d1_(0) = -cy * cz;
		h_ang_d1_(1) = cy * sz;
		h_ang_d1_(2) = sy;

		h_ang_d2_(0) = -sx * sy * cz;
		h_ang_d2_(1) = sx * sy * sz;
		h_ang_d2_(2) = sx * cy;

		h_ang_d3_(0) = cx * sy * cz;
		h_ang_d3_(1) = -cx * sy * sz;
		h_ang_d3_(2) = -cx * cy;

		h_ang_e1_(0) = sy * sz;
		h_ang_e1_(1) = sy * cz;
		h_ang_e1_(3) = 0;

		h_ang_e2_(0) = -sx * cy * sz;
		h_ang_e2_(1) = -sx * cy * cz;
		h_ang_e2_(2) = 0;

		h_ang_e3_(0) = cx * cy * sz;
		h_ang_e3_(1) = cx * cy * cz;
		h_ang_e3_(2) = 0;

		h_ang_f1_(0) = -cy * cz;
		h_ang_f1_(1) = cy * sz;
		h_ang_f1_(2) = 0;

		h_ang_f2_(0) = -cx * sz - sx * sy * cz;
		h_ang_f2_(1) = -cx * cz + sx * sy * sz;
		h_ang_f2_(2) = 0;

		h_ang_f3_(0) = -sx * sz + cx * sy * cz;
		h_ang_f3_(1) = -cx * sy * sz - sx * cz;
		h_ang_f3_(2) = 0;

		h_ang_a2_.moveToGpu(dh_ang_a2_);
		h_ang_a3_.moveToGpu(dh_ang_a3_);
		h_ang_b2_.moveToGpu(dh_ang_b2_);
		h_ang_b3_.moveToGpu(dh_ang_b3_);
		h_ang_c2_.moveToGpu(dh_ang_c2_);
		h_ang_c3_.moveToGpu(dh_ang_c3_);
		h_ang_d1_.moveToGpu(dh_ang_d1_);
		h_ang_d2_.moveToGpu(dh_ang_d2_);
		h_ang_d3_.moveToGpu(dh_ang_d3_);
		h_ang_e1_.moveToGpu(dh_ang_e1_);
		h_ang_e2_.moveToGpu(dh_ang_e2_);
		h_ang_e3_.moveToGpu(dh_ang_e3_);
		h_ang_f1_.moveToGpu(dh_ang_f1_);
		h_ang_f2_.moveToGpu(dh_ang_f2_);
		h_ang_f3_.moveToGpu(dh_ang_f3_);
	}

}

void GNormalDistributionsTransform::transformPointCloud(float *in_x, float *in_y, float *in_z,
														float *trans_x, float *trans_y, float *trans_z,
														int points_number, Eigen::Matrix<float, 4, 4> transform)
{
	Eigen::Transform<float, 3, Eigen::Affine> t(transform);

	MatrixHost htrans(3, 4);
	MatrixDevice dtrans(3, 4);

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 4; j++) {
			htrans(i, j) = t(i, j);
		}
	}

	htrans.moveToGpu(dtrans);

	if (points_number > 0) {

//added by panrui
		int block_x;
#ifdef USING_HP
		block_x = (points_number <= (BLOCK_SIZE_X)) ? points_number : (BLOCK_SIZE_X);//gpu_up
#else
		block_x = (points_number <= (BLOCK_SIZE_X/2)) ? points_number : (BLOCK_SIZE_X/2);
#endif
		int grid_x = (points_number - 1) / block_x + 1;

		gpuTransform<<<grid_x, block_x >>>(in_x, in_y, in_z, trans_x, trans_y, trans_z, points_number, dtrans);
		checkCudaErrors(cudaGetLastError());
		checkCudaErrors(cudaDeviceSynchronize());
	}

	dtrans.memFree();
}

double GNormalDistributionsTransform::computeStepLengthMT(const Eigen::Matrix<double, 6, 1> &x, Eigen::Matrix<double, 6, 1> &step_dir,
															double step_init, double step_max, double step_min, double &score,
															Eigen::Matrix<double, 6, 1> &score_gradient, Eigen::Matrix<double, 6, 6> &hessian,
															float *trans_x, float *trans_y, float *trans_z, int points_num)
{
	double phi_0 = -score;
	double d_phi_0 = -(score_gradient.dot(step_dir));

	Eigen::Matrix<double, 6, 1> x_t;

	if (d_phi_0 >= 0) {
		if (d_phi_0 == 0)
			return 0;
		else {
			d_phi_0 *= -1;
			step_dir *= -1;
		}
	}

	int max_step_iterations = 10;
	int step_iterations = 0;


	double mu = 1.e-4;
	double nu = 0.9;
	double a_l = 0, a_u = 0;

	double f_l = auxilaryFunction_PsiMT(a_l, phi_0, phi_0, d_phi_0, mu);
	double g_l = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	double f_u = auxilaryFunction_PsiMT(a_u, phi_0, phi_0, d_phi_0, mu);
	double g_u = auxilaryFunction_dPsiMT(d_phi_0, d_phi_0, mu);

	bool interval_converged = (step_max - step_min) > 0, open_interval = true;

	double a_t = step_init;
	a_t = std::min(a_t, step_max);
	a_t = std::max(a_t, step_min);

	x_t = x + step_dir * a_t;

#ifdef FOUR_COUNT
Eigen::Matrix3f R2;  
    R2 = Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX())  
        * Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY())  
        * Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ());  
    Eigen::Quaternionf q2;  
    q2 = R2;

    double a[3][3];
	a[0][0] = 1-2*(q2.y()*q2.y()+q2.z()*q2.z());
	a[0][1] = 2*(q2.x()*q2.y()-q2.z()*q2.w());
	a[0][2] = 2*(q2.x()*q2.z()+q2.y()*q2.w());
	a[1][0] = 2*(q2.x()*q2.y()+q2.z()*q2.w());
	a[1][1] = 1-2*(q2.x()*q2.x()+q2.z()*q2.z());
	a[1][2] = 2*(q2.y()*q2.z()-q2.x()*q2.w());
	a[2][0] = 2*(q2.x()*q2.z()-q2.y()*q2.w());
	a[2][1] = 2*(q2.y()*q2.z()+q2.x()*q2.w());
	a[2][2] = 1-2*(q2.x()*q2.x()+q2.y()*q2.y());

final_transformation_ << a[0][0],a[0][1],a[0][2],static_cast<float>(x_t(0)),a[1][0],a[1][1],a[1][2],static_cast<float>(x_t(1)),
					a[2][0],a[2][1],a[2][2],static_cast<float>(x_t(2)),0,0,0,1;
#endif

	transformPointCloud(x_, y_, z_, trans_x, trans_y, trans_z, points_num, final_transformation_);

	score = computeDerivatives(score_gradient, hessian, trans_x, trans_y, trans_z, points_num, x_t);

	double phi_t = -score;
	double d_phi_t = -(score_gradient.dot(step_dir));
	double psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
	double d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

	while (!interval_converged && step_iterations < max_step_iterations && !(psi_t <= 0 && d_phi_t <= -nu * d_phi_0)) {
		if (open_interval) {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			a_t = trialValueSelectionMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}

		a_t = (a_t < step_max) ? a_t : step_max;
		a_t = (a_t > step_min) ? a_t : step_min;
		
		x_t = x + step_dir * a_t;

#ifdef FOUR_COUNT
	Eigen::Matrix3f R;  
	    R = Eigen::AngleAxis<float>(static_cast<float>(x_t(3)), Eigen::Vector3f::UnitX())  
		* Eigen::AngleAxis<float>(static_cast<float>(x_t(4)), Eigen::Vector3f::UnitY())  
		* Eigen::AngleAxis<float>(static_cast<float>(x_t(5)), Eigen::Vector3f::UnitZ());  
	    Eigen::Quaternionf q;  
	    q = R;

	    double a[3][3];
		a[0][0] = 1-2*(q.y()*q.y()+q.z()*q.z());
		a[0][1] = 2*(q.x()*q.y()-q.z()*q.w());
		a[0][2] = 2*(q.x()*q.z()+q.y()*q.w());
		a[1][0] = 2*(q.x()*q.y()+q.z()*q.w());
		a[1][1] = 1-2*(q.x()*q.x()+q.z()*q.z());
		a[1][2] = 2*(q.y()*q.z()-q.x()*q.w());
		a[2][0] = 2*(q.x()*q.z()-q.y()*q.w());
		a[2][1] = 2*(q.y()*q.z()+q.x()*q.w());
		a[2][2] = 1-2*(q.x()*q.x()+q.y()*q.y());

	final_transformation_ << a[0][0],a[0][1],a[0][2],static_cast<float>(x_t(0)),a[1][0],a[1][1],a[1][2],static_cast<float>(x_t(1)),
						a[2][0],a[2][1],a[2][2],static_cast<float>(x_t(2)),0,0,0,1;
#endif
		transformPointCloud(x_, y_, z_, trans_x, trans_y, trans_z, points_num, final_transformation_);

		score = computeDerivatives(score_gradient, hessian, trans_x, trans_y, trans_z, points_num, x_t, false);

		phi_t -= score;
		d_phi_t -= (score_gradient.dot(step_dir));
		psi_t = auxilaryFunction_PsiMT(a_t, phi_t, phi_0, d_phi_0, mu);
		d_psi_t = auxilaryFunction_dPsiMT(d_phi_t, d_phi_0, mu);

		if (open_interval && (psi_t <= 0 && d_psi_t >= 0)) {
			open_interval = false;

			f_l += phi_0 - mu * d_phi_0 * a_l;
			g_l += mu * d_phi_0;

			f_u += phi_0 - mu * d_phi_0 * a_u;
			g_u += mu * d_phi_0;
		}

		if (open_interval) {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, psi_t, d_psi_t);
		} else {
			interval_converged = updateIntervalMT(a_l, f_l, g_l, a_u, f_u, g_u, a_t, phi_t, d_phi_t);
		}
		step_iterations++;
	}

	if (step_iterations) {
		computeHessian(hessian, trans_x, trans_y, trans_z, points_num, x_t);
	}

	real_iterations_ += step_iterations;

	return a_t;
}


//Copied from ndt.hpp
double GNormalDistributionsTransform::trialValueSelectionMT (double a_l, double f_l, double g_l,
															double a_u, double f_u, double g_u,
															double a_t, double f_t, double g_t)
{
	// Case 1 in Trial Value Selection [More, Thuente 1994]
	if (f_t > f_l) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		double w = std::sqrt (z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, f_t and g_l
		// Equation 2.4.2 [Sun, Yuan 2006]
		double a_q = a_l - 0.5 * (a_l - a_t) * g_l / (g_l - (f_l - f_t) / (a_l - a_t));

		if (std::fabs (a_c - a_l) < std::fabs (a_q - a_l))
		  return (a_c);
		else
		  return (0.5 * (a_q + a_c));
	}
	// Case 2 in Trial Value Selection [More, Thuente 1994]
	else if (g_t * g_l < 0) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		double w = std::sqrt (z * z - g_t * g_l);
		// Equation 2.4.56 [Sun, Yuan 2006]
		double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates f_l, g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		if (std::fabs (a_c - a_t) >= std::fabs (a_s - a_t))
		  return (a_c);
		else
		  return (a_s);
	}
	// Case 3 in Trial Value Selection [More, Thuente 1994]
	else if (std::fabs (g_t) <= std::fabs (g_l)) {
		// Calculate the minimizer of the cubic that interpolates f_l, f_t, g_l and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_l) / (a_t - a_l) - g_t - g_l;
		double w = std::sqrt (z * z - g_t * g_l);
		double a_c = a_l + (a_t - a_l) * (w - g_l - z) / (g_t - g_l + 2 * w);

		// Calculate the minimizer of the quadratic that interpolates g_l and g_t
		// Equation 2.4.5 [Sun, Yuan 2006]
		double a_s = a_l - (a_l - a_t) / (g_l - g_t) * g_l;

		double a_t_next;

		if (std::fabs (a_c - a_t) < std::fabs (a_s - a_t))
		  a_t_next = a_c;
		else
		  a_t_next = a_s;

		if (a_t > a_l)
		  return (std::min (a_t + 0.66 * (a_u - a_t), a_t_next));
		else
		  return (std::max (a_t + 0.66 * (a_u - a_t), a_t_next));
	}
	// Case 4 in Trial Value Selection [More, Thuente 1994]
	else {
		// Calculate the minimizer of the cubic that interpolates f_u, f_t, g_u and g_t
		// Equation 2.4.52 [Sun, Yuan 2006]
		double z = 3 * (f_t - f_u) / (a_t - a_u) - g_t - g_u;
		double w = std::sqrt (z * z - g_t * g_u);
		// Equation 2.4.56 [Sun, Yuan 2006]
		return (a_u + (a_t - a_u) * (w - g_u - z) / (g_t - g_u + 2 * w));
	}
}

//Copied from ndt.hpp
double GNormalDistributionsTransform::updateIntervalMT (double &a_l, double &f_l, double &g_l,
														double &a_u, double &f_u, double &g_u,
														double a_t, double f_t, double g_t)
{
  // Case U1 in Update Algorithm and Case a in Modified Update Algorithm [More, Thuente 1994]
	if (f_t > f_l) {
		a_u = a_t;
		f_u = f_t;
		g_u = g_t;
		return (false);
	}
	// Case U2 in Update Algorithm and Case b in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) > 0) {
		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Case U3 in Update Algorithm and Case c in Modified Update Algorithm [More, Thuente 1994]
	else if (g_t * (a_l - a_t) < 0) {
		a_u = a_l;
		f_u = f_l;
		g_u = g_l;

		a_l = a_t;
		f_l = f_t;
		g_l = g_t;
		return (false);
	}
	// Interval Converged
	else
		return (true);
}

void GNormalDistributionsTransform::computeHessian(Eigen::Matrix<double, 6, 6> &hessian, float *trans_x, float *trans_y, float *trans_z, int points_num, Eigen::Matrix<double, 6, 1> &p)
{
	int *valid_points, *voxel_id, *starting_voxel_id;
	int valid_voxel_num, valid_points_num;
	//Radius Search

	voxel_grid_.radiusSearch(trans_x, trans_y, trans_z, points_num, resolution_, INT_MAX, &valid_points, &starting_voxel_id, &voxel_id, &valid_voxel_num, &valid_points_num);

	double *centroid = voxel_grid_.getCentroidList();
	double *covariance = voxel_grid_.getCovarianceList();
	double *inverse_covariance = voxel_grid_.getInverseCovarianceList();
	int *points_per_voxel = voxel_grid_.getPointsPerVoxelList();
	int voxel_num = voxel_grid_.getVoxelNum();

	if (valid_points_num <= 0)
		return;

	//Update score gradient and hessian matrix
	double *hessians, *point_gradients, *point_hessians;

	checkCudaErrors(cudaMalloc(&hessians, sizeof(double) * valid_points_num * 6 * 6));
	checkCudaErrors(cudaMemset(hessians, 0, sizeof(double) * valid_points_num * 6 * 6));

	checkCudaErrors(cudaMalloc(&point_gradients, sizeof(double) * valid_points_num * 3 * 6));
	checkCudaErrors(cudaMemset(point_gradients, 0, sizeof(double) * valid_points_num * 3 * 6));

	checkCudaErrors(cudaMalloc(&point_hessians, sizeof(double) * valid_points_num * 18 * 6));
	checkCudaErrors(cudaMemset(point_hessians, 0, sizeof(double) * valid_points_num * 18 * 6));

	int block_x = (valid_points_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_points_num;
	int grid_x = (valid_points_num - 1) / block_x + 1;
	dim3 grid;

	computePointGradients0<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_a_, dj_ang_b_, dj_ang_c_, dj_ang_d_,
												point_gradients);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computePointGradients1<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dj_ang_e_, dj_ang_f_, dj_ang_g_, dj_ang_h_,
												point_gradients);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computePointHessian0<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_a2_, dh_ang_a3_,
												dh_ang_b2_, dh_ang_b3_,
												point_hessians);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computePointHessian1<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_c2_, dh_ang_c3_,
												dh_ang_d1_, dh_ang_d2_, dh_ang_d3_,
												point_hessians);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computePointHessian2<<<grid_x, block_x>>>(x_, y_, z_, points_number_,
												valid_points, valid_points_num,
												dh_ang_e1_, dh_ang_e2_, dh_ang_e3_,
												dh_ang_f1_, dh_ang_f2_, dh_ang_f3_,
												point_hessians);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	double *tmp_hessian;

	checkCudaErrors(cudaMalloc(&tmp_hessian, sizeof(double) * valid_voxel_num * 6));

	double *e_x_cov_x;

	checkCudaErrors(cudaMalloc(&e_x_cov_x, sizeof(double) * valid_voxel_num));

	double *cov_dxd_pi;

	checkCudaErrors(cudaMalloc(&cov_dxd_pi, sizeof(double) * valid_voxel_num * 3 * 6));

	computeExCovX<<<grid_x, block_x>>>(trans_x, trans_y, trans_z, valid_points,
										starting_voxel_id, voxel_id, valid_points_num,
										centroid, inverse_covariance, voxel_num,
										gauss_d1_, gauss_d2_,
										e_x_cov_x);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	grid.x = grid_x;
	grid.y = 3;
	grid.z = 6;
	computeCovDxdPi<<<grid, block_x>>>(valid_points, starting_voxel_id, voxel_id, valid_points_num,
											inverse_covariance, voxel_num,
											gauss_d1_, gauss_d2_, point_gradients,
											cov_dxd_pi, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	int block_x2 = (valid_voxel_num > BLOCK_SIZE_X) ? BLOCK_SIZE_X : valid_voxel_num;
	int grid_x2 = (valid_voxel_num - 1) / block_x2 + 1;

	updateExCovX<<<grid_x2, block_x2>>>(e_x_cov_x, gauss_d2_, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	grid.y = 6;
	grid.z = 1;
	computeHessianListS0<<<grid_x, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, inverse_covariance, voxel_num,
												gauss_d1_, gauss_d2_,
												point_gradients, tmp_hessian, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computeHessianListS1<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, inverse_covariance, voxel_num,
												gauss_d1_, gauss_d2_,
												point_gradients, tmp_hessian, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computeHessianListS2<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, inverse_covariance, voxel_num,
												gauss_d1_, gauss_d2_,
												point_gradients, tmp_hessian, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	grid.z = 6;
	computeHessianListS3<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, voxel_num,
												gauss_d1_, gauss_d2_,
												hessians,
												e_x_cov_x, tmp_hessian, cov_dxd_pi,
												valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());


	computeHessianListS4<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												centroid, inverse_covariance, voxel_num,
												gauss_d1_, gauss_d2_,
												point_hessians, hessians,
												e_x_cov_x);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computeHessianListS5<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, inverse_covariance, voxel_num,
													gauss_d1_, gauss_d2_,
													point_hessians, hessians,
													e_x_cov_x);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computeHessianListS6<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
													starting_voxel_id, voxel_id, valid_points_num,
													centroid, inverse_covariance, voxel_num,
													gauss_d1_, gauss_d2_,
													point_hessians, hessians,
													e_x_cov_x);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	computeHessianListS7<<<grid, block_x>>>(trans_x, trans_y, trans_z, valid_points,
												starting_voxel_id, voxel_id, valid_points_num,
												gauss_d1_, gauss_d2_,
												point_gradients, hessians,
												e_x_cov_x, cov_dxd_pi, valid_voxel_num);
	checkCudaErrors(cudaGetLastError());
//	checkCudaErrors(cudaDeviceSynchronize());

	int full_size = valid_points_num;
	int half_size = (full_size - 1) / 2 + 1;

	while (full_size > 1) {
		block_x = (half_size > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_size;
		grid_x = (half_size - 1) / block_x + 1;

		grid.x = grid_x;
		grid.y = 6;
		grid.z = 6;
		matrixSum<<<grid_x, block_x>>>(hessians, full_size, half_size, 6, 6, valid_points_num);
//		checkCudaErrors(cudaDeviceSynchronize());

		full_size = half_size;
		half_size = (full_size - 1) / 2 + 1;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	MatrixDevice dhessian(6, 6, valid_points_num, hessians);
	MatrixHost hhessian(6, 6);

	hhessian.moveToHost(dhessian);

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j < 6; j++) {
			hessian(i, j) = hhessian(i, j);
		}
	}

	checkCudaErrors(cudaFree(hessians));
	checkCudaErrors(cudaFree(point_hessians));
	checkCudaErrors(cudaFree(point_gradients));

	checkCudaErrors(cudaFree(tmp_hessian));
	checkCudaErrors(cudaFree(e_x_cov_x));
	checkCudaErrors(cudaFree(cov_dxd_pi));

	if (valid_points != NULL) {
		checkCudaErrors(cudaFree(valid_points));
	}

	if (voxel_id != NULL) {
		checkCudaErrors(cudaFree(voxel_id));
	}

	if (starting_voxel_id != NULL) {
		checkCudaErrors(cudaFree(starting_voxel_id));
	}

	dhessian.memFree();
}


double GNormalDistributionsTransform::getFitnessScore(double max_range)
{
	double fitness_score = 0.0;

	float *trans_x, *trans_y, *trans_z;

	checkCudaErrors(cudaMalloc(&trans_x, sizeof(float) * points_number_));
	checkCudaErrors(cudaMalloc(&trans_y, sizeof(float) * points_number_));
	checkCudaErrors(cudaMalloc(&trans_z, sizeof(float) * points_number_));

	transformPointCloud(x_, y_, z_, trans_x, trans_y, trans_z, points_number_, final_transformation_);

	int *valid_distance;

	checkCudaErrors(cudaMalloc(&valid_distance, sizeof(int) * points_number_));

	double *min_distance;

	checkCudaErrors(cudaMalloc(&min_distance, sizeof(double) * points_number_));

	voxel_grid_.nearestNeighborSearch(trans_x, trans_y, trans_z, points_number_, valid_distance, min_distance, max_range);

	int size = points_number_;
	int half_size;

	while (size > 1) {
		half_size = (size - 1) / 2 + 1;

		int block_x = (half_size > BLOCK_SIZE_X) ? BLOCK_SIZE_X : half_size;
		int grid_x = (half_size - 1) / block_x + 1;

		gpuSum<double><<<grid_x, block_x>>>(min_distance, size, half_size);
		checkCudaErrors(cudaGetLastError());

		gpuSum<int><<<grid_x, block_x>>>(valid_distance, size, half_size);
		checkCudaErrors(cudaGetLastError());

		size = half_size;
	}

	checkCudaErrors(cudaDeviceSynchronize());

	int nr;

	checkCudaErrors(cudaMemcpy(&nr, valid_distance, sizeof(int), cudaMemcpyDeviceToHost));
	checkCudaErrors(cudaMemcpy(&fitness_score, min_distance, sizeof(double), cudaMemcpyDeviceToHost));

	checkCudaErrors(cudaFree(trans_x));
	checkCudaErrors(cudaFree(trans_y));
	checkCudaErrors(cudaFree(trans_z));
	checkCudaErrors(cudaFree(valid_distance));
	checkCudaErrors(cudaFree(min_distance));

	if (nr > 0)
		return (fitness_score / nr);

	return DBL_MAX;
}

}
