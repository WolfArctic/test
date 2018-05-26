#ifndef KALMAN_APPLICATIONS_LSAV_SYSTEMMODEL_HPP_
#define KALMAN_APPLICATIONS_LSAV_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>

namespace KalmanApplications
{
namespace LSAV
{

/**
 * @brief System state vector-type for a 3DOF planar robot
 *
 * This is a system state for a very simple planar robot that
 * is characterized by its (x,y)-Position and angular orientation.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 8>
{
public:
    KALMAN_VECTOR(State, T, 8)
    
    //! X-position
    static constexpr size_t X = 0;
    //! Y-Position
    static constexpr size_t Y = 1;
    //! Orientation
    static constexpr size_t VX = 2;

    static constexpr size_t VY = 3;

    static constexpr size_t AX = 4;

    static constexpr size_t AY = 5;

    static constexpr size_t YAW = 6;

    static constexpr size_t YAWRATE = 7;
    
    T x()         const { return (*this)[ X ]; }
    T y()         const { return (*this)[ Y ]; }
    T vx()        const { return (*this)[ VX ]; }
    T vy()        const { return (*this)[ VY ]; }
    T ax()        const { return (*this)[ AX ]; }
    T ay()        const { return (*this)[ AY ]; }
    T yaw()       const { return (*this)[ YAW ]; }
    T yawrate()   const { return (*this)[ YAWRATE ]; }
    
    T& x()      { return (*this)[ X ]; }
    T& y()      { return (*this)[ Y ]; }
    T& vx()      { return (*this)[ VX ]; }
    T& vy()      { return (*this)[ VY ]; }
    T& ax()      { return (*this)[ AX ]; }
    T& ay()      { return (*this)[ AY ]; }
    T& yaw()      { return (*this)[ YAW ]; }
    T& yawrate()  { return (*this)[ YAWRATE ]; }
};

/**
 * @brief System control-input vector-type for a 3DOF planar robot
 *
 * This is the system control-input of a very simple planar robot that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 2>
{
public:
    KALMAN_VECTOR(Control, T, 2)
    
    //! Velocity
    static constexpr size_t V = 0;
    //! Angular Rate (Orientation-change)
    static constexpr size_t DTHETA = 1;
    
    T v()       const { return (*this)[ V ]; }
    T dtheta()  const { return (*this)[ DTHETA ]; }
    
    T& v()      { return (*this)[ V ]; }
    T& dtheta() { return (*this)[ DTHETA ]; }
};

/**
 * @brief System model for a simple planar 3DOF robot
 *
 * This is the system model defining how our robot moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef State<T> S;

    //! Control type shortcut definition
    typedef Control<T> C;

    static constexpr size_t deltaT = 0.02;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;

        x_.x() = x.x() + x.vx() * deltaT + 0.5 * x.ax() * deltaT * deltaT;

        x_.y() = x.y() + x.vy() * deltaT + 0.5 * x.ay() * deltaT * deltaT;

        x_.vx() = x.vx() + x.ax() * deltaT;

        x_.vy() = x.vy() + x.ay() * deltaT;

        x_.ax() = x.ax();

        x_.ay() = x.ay();
        
        x_.yaw() = x.yaw() + x.yawrate() * deltaT;

        x_.yawrate() = x.yawrate();
        
        // Return transitioned state vector
        return x_;
    }
    
protected:
    void updateJacobians( const S& x, const C& u)
    {}
};

} // namespace LSAV
} // namespace KalmanApplications

#endif