/*
 *  Copyright (c) 2015, Nagoya University

 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "hermite_curve.h"

namespace generatecurve
{
void createVectorFromPose(const geometry_msgs::Pose &p, tf::Vector3 *v)
{
  tf::Transform pose;
  tf::poseMsgToTF(p, pose);
  tf::Vector3 x_axis(1, 0, 0);
  *v = pose.getBasis() * x_axis;
}

void getPointAndVectorFromPose(const geometry_msgs::Pose &pose, Element2D *point, Element2D *vector)
{
  point->set(pose.position.x, pose.position.y);

  tf::Vector3 tmp_tf_vevtor;
  createVectorFromPose(pose, &tmp_tf_vevtor);
  vector->set(tmp_tf_vevtor.getX(), tmp_tf_vevtor.getY());
}

std::vector<geometry_msgs::Pose> generateHermiteCurveForROS(const geometry_msgs::Pose &start,
                                                            const geometry_msgs::Pose &end,
                                                            const double vlength)
{
  std::vector<geometry_msgs::Pose> wps;
  Element2D p0(0, 0), v0(0, 0), p1(0, 0), v1(0, 0);
  getPointAndVectorFromPose(start, &p0, &v0);
  getPointAndVectorFromPose(end, &p1, &v1);

  std::vector<Element2D> result = generateHermiteCurve(p0, v0, p1, v1, vlength);

  double height_d = fabs(start.position.z - end.position.z);
  for (uint32_t i = 0; i < result.size(); i++)
  {
    geometry_msgs::Pose wp;
    wp.position.x = result.at(i).x;
    wp.position.y = result.at(i).y;
    // height
    wp.position.z = (i == 0) ? start.position.z : (i == result.size() - 1)
                                       ? end.position.z : start.position.z < end.position.z
                                       ? start.position.z + height_d * i / result.size()
                                       : start.position.z - height_d * i / result.size();

    // orientation
    if (i != result.size() - 1)
    {
      double radian = atan2(result.at(i + 1).y - result.at(i).y, result.at(i + 1).x - result.at(i).x);
      wp.orientation = tf::createQuaternionMsgFromYaw(radian);
    }
    else
    {
      wp.orientation = wps.at(wps.size() - 1).orientation;
    }

    wps.push_back(wp);
  }
  return wps;
}

std::vector<Element2D> generateHermiteCurve(const Element2D &p0, const Element2D &v0, const Element2D &p1,
                                            const Element2D &v1, const double vlength)
{
  std::vector<Element2D> result;
  const double interval = 0.15;
  int32_t divide = 2;
  const int32_t loop = 2 * std::round(std::hypot(p1.x - p0.x, p1.y - p1.y) / 0.15);
  divide = 2;
  // std::cout<<"**** "<<loop<<" ****"<<std::endl;
  while (divide < loop)
  {
    result.reserve(divide);
    for (int32_t i = 0; i < divide; i++)
    {
      double u = i * 1.0 / (divide - 1);
      double u_square = pow(u, 2);
      double u_cube = pow(u, 3);
      double coeff_p0 = 2 * u_cube - 3 * u_square + 1;
      double coeff_v0 = u_cube - 2 * u_square + u;
      double coeff_p1 = (-1) * 2 * u_cube + 3 * u_square;
      double coeff_v1 = u_cube - u_square;
      result.push_back(
          Element2D((p0.x * coeff_p0 + vlength * v0.x * coeff_v0 + p1.x * coeff_p1 + vlength * v1.x * coeff_v1),
                    (p0.y * coeff_p0 + vlength * v0.y * coeff_v0 + p1.y * coeff_p1 + vlength * v1.y * coeff_v1)));
    }
    float dt = sqrt(pow(result.at(0).x - result.at(1).x, 2) + pow(result.at(0).y - result.at(1).y, 2));
    if (interval > dt || divide == loop - 1)
      return result;
    else
    {
      result.clear();
      result.shrink_to_fit();
      divide++;
    }
  }
  return result;
}
}  // namespace
