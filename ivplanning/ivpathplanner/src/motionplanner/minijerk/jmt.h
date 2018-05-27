#ifndef JMT_H_
#define JMT_H_

#include <iostream>
#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Dense>
#include "helper.h"

class JMT 
{

  public:
    Eigen::VectorXd c;
    JMT( const State& start, const State& end, const double t);
    double get(const double t) const;
    double get_velocity(const double t) const;
    double get_acceleration(const double t) const;
};

std::vector<double> differentiate(const std::vector<double> &coefficients);
double to_equation(const std::vector<double> &coefficients, double t);

#endif // JMT_H_
