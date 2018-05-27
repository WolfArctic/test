#include "jmt.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

JMT::JMT(const State& start, const State& end, const double t) 
{

  MatrixXd A = MatrixXd(3,3);
  VectorXd b = VectorXd(3);
  VectorXd x = VectorXd(3);
  this->c = VectorXd(6);

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

    A <<   t3,     t4,    t5,
         3*t2,   4*t3,  5*t4,
         6*t ,  12*t2, 20*t3;

    b << end.p - (start.p + start.v * t + 0.5 * start.a * t2),
         end.v - (start.v + start.a * t),
         end.a -  start.a;

    x = A.inverse() * b;

    // if(start.a > 0.001 && end.v - start.v > 0 )
    // {
    //   double a3_a = (end.v - start.v - start.a * t) / (3*t2);
    //   this->c << start.p,
    //        start.v,
    //        start.a / 2.0,
    //        a3_a,
    //        0,
    //        0;
    // }



    if(end.v < 0.001 && fabs(start.v - end.v) > 0.2) //emergency stop
    {
      double a2_e = (end.v - start.v) / (2*t);
      this->c << start.p,
           start.v,
           a2_e,
           0,
           0,
           0;

    }
    else if((start.a > 0.001 && end.v - start.v > 0 ) || (start.a < -0.001 && end.v - start.v < 0 ))
    {
      MatrixXd A_a = MatrixXd(2,2);
      MatrixXd b_a = VectorXd(2);
      VectorXd x_a = VectorXd(2);

      A_a << 3*t2,  4*t3,
            6*t,  12*t2;

      b_a << end.v - (start.v + start.a * t),
            end.a - start.a;

      x_a = A_a.inverse() * b_a;
      double a3_a = (end.v - start.v - start.a * t) / (3*t2);
      this->c << start.p,
           start.v,
           start.a / 2.0,
           x_a[0],
           x_a[1],
           0;
    }
    else
    {
      this->c << start.p,
           start.v,
           start.a / 2.0,
           x[0],
           x[1],
           x[2];
    }

    //std::cout << this->c << std::endl;
}

double JMT::get(const double t) const
{

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  Eigen::VectorXd T = VectorXd(6);
  T << 1.0, t, t2, t3, t4, t5;

  return T.transpose() * this->c;
}


double JMT::get_velocity(const double t) const
{

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  Eigen::VectorXd T = VectorXd(6);
  T << 0.0, 1.0, 2*t, 3*t2, 4*t3, 5*t4;

  return T.transpose() * this->c;
}

double JMT::get_acceleration(const double t) const
{

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  Eigen::VectorXd T = VectorXd(6);
  T << 0.0, 0.0, 2, 6*t, 12*t2, 20*t3;

  return T.transpose() * this->c;
}

/*
 Calculates the derivative of a polynomial and returns the corresponding coefficients.
 */
std::vector<double> differentiate(const std::vector<double> &coefficients)
{
  std::vector<double> new_cos = {};
  for (int i=0; i<coefficients.size();i++ )
  {
    if (i==0)
    {
      continue;
    }
    new_cos.push_back(i * coefficients[i]);
  }
  return new_cos;
}

/*
 Takes the coefficients of a polynomial and point t, calculate the f value
 */
double to_equation(const std::vector<double> &coefficients, double t)
{
  double total = 0.0;
  for (int i = 0; i<coefficients.size(); i++ )
  {
    total += coefficients[i] * pow(t, i);
  }
  return total;
}
