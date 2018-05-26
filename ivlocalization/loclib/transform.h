#ifndef TRANSFORM_H
#define TRANSFORM_H
#pragma once
#include <vector>
#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>

#include "common.h"

namespace loclib
{
	template<typename T1, typename T2>
	class transform
	{
	public:
		typedef typename Eigen::Matrix<T2, 4, 4> Matrix4;
		typedef typename Eigen::Matrix<T2, 3, 3> Matrix3;
		typedef typename Eigen::Matrix<T2, Eigen::Dynamic, 1> VectorX;
		typedef typename Eigen::Matrix<T2, 3, 1> Vector3;
		typedef typename Eigen::Quaternion<T2> Quaternion;
		typedef typename Eigen::AngleAxis<T2> AngleAxis;
	  typedef typename Eigen::Translation<T2, 3> Translation3;
		typedef sPose3D<T1> Pose3D;
		typedef geometry_msgs::Pose PoseMsg;
		typedef tf::Matrix3x3 Matrix3TF;
		typedef tf::Quaternion QuaternionTF;

		explicit transform(){};
		virtual ~transform(){};
		/*******************/
		void MsgToPose(PoseMsg input, Pose3D &output);

		/*******************/
		void PoseToMsg(Pose3D input, PoseMsg &output);

		/*******************/
		void PoseToEigen(Pose3D input, Matrix4 &output);

		/*******************/
		void EigenToPose(Matrix4 input, Pose3D &output);

		/*******************/
		void EigenToMsg(Matrix4 input, PoseMsg &output);

		/*******************/
		void MsgToEigen(PoseMsg input, Matrix4 &output);

		/*******************/
		void rotationDifference(VectorX input1, VectorX input2, Quaternion &output);

		/*******************/
		void calculateRelativeTransform(VectorX input1, VectorX input2, VectorX &output);

		/*******************/
		void EigenMatrixToTFMatrix(Matrix4 input, Matrix3TF &output);

		/*******************/
		void TFMatrixToEigenMatrix(Matrix3TF input, Matrix4 &output);

	private:

	};
}

#include "transform.hpp"

#endif