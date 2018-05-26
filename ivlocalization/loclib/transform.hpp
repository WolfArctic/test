template<typename T1, typename T2>
void loclib::transform<T1, T2>::MsgToPose(PoseMsg input, Pose3D &output){
  output.x = static_cast<T1>(input.position.x);
  output.y = static_cast<T1>(input.position.y);
  output.z = static_cast<T1>(input.position.z);
  QuaternionTF temp_quat(input.orientation.x, input.orientation.y, input.orientation.z, input.orientation.w);
  tfScalar temp_roll, temp_pitch, temp_yaw;
  Matrix3TF(temp_quat).getRPY(temp_roll, temp_pitch, temp_yaw);
  output.roll = temp_roll;
  output.pitch = temp_pitch;
  output.yaw = temp_yaw;
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::PoseToMsg(Pose3D input, PoseMsg &output){
  output.position.x = static_cast<double>(input.x);
  output.position.y = static_cast<double>(input.y);
  output.position.z = static_cast<double>(input.z);
  output.orientation = tf::createQuaternionMsgFromRollPitchYaw(static_cast<double>(input.roll), 
  															   static_cast<double>(input.pitch), 
  															   static_cast<double>(input.yaw));
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::PoseToEigen(Pose3D input, Matrix4 &output){
  AngleAxis rotation_x(static_cast<T2>(input.roll), Vector3::UnitX());
  AngleAxis rotation_y(static_cast<T2>(input.pitch), Vector3::UnitY());
  AngleAxis rotation_z(static_cast<T2>(input.yaw), Vector3::UnitZ());
  Translation3 temp_translation(input.x, input.y, 0);
  output = (temp_translation * rotation_z * rotation_y * rotation_x).matrix();
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::EigenToPose(Matrix4 input, Pose3D &output){
  Matrix3TF temp_tf_matrix;
  EigenMatrixToTFMatrix(input, temp_tf_matrix);
  output.x = static_cast<T1>(input(0, 3));
  output.y = static_cast<T1>(input(1, 3));
  output.z = static_cast<T1>(input(2, 3));
  sPose3Dd temp_pose;
  temp_tf_matrix.getRPY(temp_pose.roll, temp_pose.pitch, temp_pose.yaw);
  output.roll = static_cast<T1>(temp_pose.roll);
  output.pitch = static_cast<T1>(temp_pose.pitch);
  output.yaw = static_cast<T1>(temp_pose.yaw);
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::EigenToMsg(Matrix4 input, PoseMsg &output){
  output.position.x = static_cast<double>(input(0, 3));
  output.position.y = static_cast<double>(input(1, 3));
  output.position.z = static_cast<double>(input(2, 3));
  Matrix3 temp_matrix;
  for(unsigned int i = 0; i < 3; ++ i)
  	for(unsigned int j = 0; j < 3; ++ j)
  		temp_matrix(i, j) = input(i, j);
  Quaternion output_quaternion(temp_matrix);
  output.orientation.x = static_cast<double>(output_quaternion.x());
  output.orientation.y = static_cast<double>(output_quaternion.y());
  output.orientation.z = static_cast<double>(output_quaternion.z());
  output.orientation.w = static_cast<double>(output_quaternion.w());
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::MsgToEigen(PoseMsg input, Matrix4 &output){
  Quaternion output_quaternion(static_cast<T2>(input.orientation.w), 
  							   static_cast<T2>(input.orientation.x), 
  							   static_cast<T2>(input.orientation.y), 
  							   static_cast<T2>(input.orientation.z));
  Matrix3 temp_matrix;
  for(unsigned int i = 0; i < 3; ++ i)
  	for(unsigned int j = 0; j < 3; ++ j)
  		output(i, j) = output_quaternion.toRotationMatrix()(i, j);
  output.col(3) << static_cast<T2>(input.position.x), static_cast<T2>(input.position.y), 
  				   static_cast<T2>(input.position.z), 1;
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::rotationDifference(VectorX input1, VectorX input2, Quaternion &output){
  Quaternion prior_quaternion(input1(6), input1(3), input1(4), input1(5));
  Matrix3 prior_quaternion_matrix(prior_quaternion);
  Quaternion posterior_quaternion(input2(6), input2(3), input2(4), input2(5));
  Matrix3 posterior_quaternion_matrix(posterior_quaternion);
  Matrix3 posterior_to_prior = prior_quaternion_matrix.inverse() * posterior_quaternion_matrix;
  output = posterior_to_prior;
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::calculateRelativeTransform(VectorX input1, VectorX input2, VectorX &output){
  /**********w, x, y, z (order) in Eigen***********/
  Quaternion prior_quaternion(input1(6), input1(3), input1(4), input1(5));
  Matrix3 prior_quaternion_matrix(prior_quaternion);
  Quaternion posterior_quaternion(input2(6), input2(3), input2(4), input2(5));
  Matrix3 posterior_quaternion_matrix(posterior_quaternion);
  Matrix4 prior_matrix(Matrix4::Identity());
  Matrix4 posterior_matrix(Matrix4::Identity());
  prior_matrix.block<3,3>(0,0) = prior_quaternion_matrix;
  posterior_matrix.block<3,3>(0,0) = posterior_quaternion_matrix;
  prior_matrix.col(3) << input1(0), input1(1), input1(2), 1;
  posterior_matrix.col(3) << input2(0), input2(1), input2(2), 1;
  Matrix4 transform_matrix = prior_matrix.inverse() * posterior_matrix;
  Quaternion output_quaternion(transform_matrix.block<3,3>(0,0));
  output << transform_matrix(0,3), transform_matrix(1,3), transform_matrix(2,3), 
            output_quaternion.x(), output_quaternion.y(), output_quaternion.z(), output_quaternion.w();
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::EigenMatrixToTFMatrix(Matrix4 input, Matrix3TF &output){
  output.setValue(static_cast<double>(input(0, 0)), static_cast<double>(input(0, 1)),
                  static_cast<double>(input(0, 2)), static_cast<double>(input(1, 0)),
                  static_cast<double>(input(1, 1)), static_cast<double>(input(1, 2)),
                  static_cast<double>(input(2, 0)), static_cast<double>(input(2, 1)),
                  static_cast<double>(input(2, 2)));  
}

template<typename T1, typename T2>
void loclib::transform<T1, T2>::TFMatrixToEigenMatrix(Matrix3TF input, Matrix4 &output){
  for(unsigned int i = 0; i < 3 ; i ++)
    for(unsigned int j = 0; j < 3; j ++)
      output(i, j) = static_cast<T2>(input[i][j]);
}
