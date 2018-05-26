#ifndef KALMAN_APPLICATIONS_LSAV_POSITIONMEASUREMENTMODEL_HPP_
#define KALMAN_APPLICATIONS_LSAV_POSITIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanApplications
{
namespace LSAV
{

/**
 * @brief Measurement vector measuring the robot position
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Measurement : public Kalman::Vector<T, 5>
{
public:
    KALMAN_VECTOR(Measurement, T, 5)
    
    static constexpr size_t Z_X = 0;
    
    static constexpr size_t Z_Y = 1;

    static constexpr size_t Z_V = 2;

    static constexpr size_t Z_YAW = 3;

    //yaw rate
    static constexpr size_t Z_YAWRATE = 4;
    
    T z_x()       const { return (*this)[ Z_X ]; }
    T z_y()       const { return (*this)[ Z_Y ]; }
    T z_v()       const { return (*this)[ Z_V ]; }
    T z_yaw()     const { return (*this)[ Z_YAW ]; }
    T z_yawrate()      const { return (*this)[ Z_YAWRATE ]; }
    
    T& z_x()      { return (*this)[ Z_X ]; }
    T& z_y()      { return (*this)[ Z_Y ]; }
    T& z_v()      { return (*this)[ Z_V ]; }
    T& z_yaw()    { return (*this)[ Z_YAW ]; }
    T& z_yawrate()     { return (*this)[ Z_YAWRATE ]; }
};

/**
 * @brief Measurement model for measuring the position of the robot
 *        using two beacon-landmarks
 *
 * This is the measurement model for measuring the position of the robot.
 * The measurement is given by two landmarks in the space, whose positions are known.
 * The robot can measure the direct distance to both the landmarks, for instance
 * through visual localization techniques.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class MeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, Measurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;
    
    //! Measurement type shortcut definition
    typedef Measurement<T> M;
    
    /**
     * @brief Constructor
     *
     */
    MeasurementModel()
    {   
        // Setup noise jacobian. As this one is static, we can define it once
        // and do not need to update it dynamically
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        //measurement of x
        measurement.z_x() = x.x();

        //measurement of y
        measurement.z_y() = x.y();

        //measurement of v
        measurement.z_v() = x.vx() * std::cos(x.yaw()) + x.vy() * std::sin(x.yaw());

        //measurement of yaw
        measurement.z_yaw() = x.yaw();

        //measurement of yaw rate
        measurement.z_yawrate() = x.yawrate();
        
        return measurement;
    }

protected:
    void updateJacobians( const S& x )
    {}

};

} // namespace LSAV
} // namespace KalmanApplications

#endif