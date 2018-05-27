#ifndef _VEKF_H
#define _VEKF_H

#include "../../../../../avoslib/kalman/ekfilter.hpp"
using namespace Kalman;

//!  template<typename T, K_UINT_32 BEG, bool OQ = false, bool OVR = false, bool DBG = true>
//! - \c T : Type of elements contained in matrices and vectors. Usually 
//!          \c float or \c double.
//! - \c BEG : Starting index of matrices and vectors. Can be either 0 or 1.
//! - \c OQ : Optimize calculations on \a Q. This can be turned on if \a Q 
//!           is diagonal.
//! - \c OVR : Optimize calculations on \a V and \a R. This can be turned on
//!            if \a V and \a R are both diagonal matrices.
//! - \c DGB : Debug flag. If \c true, then bound-checking will be performed,
//!            and \c OutOfBoundError exceptions can be thrown.
class cVabsekf : public Kalman::EKFilter<double,0,true,true,true> 
{
public:
	cVabsekf();
	void resetR( Vector );	

protected:
	void makeBaseA();
	void makeBaseH();
	void makeBaseW();
	void makeBaseQ();
	void makeBaseV();
	//void makeBaseR();


	void makeA();
	void makeR();
	void makeProcess();
	void makeMeasure();
	Vector tmp_R;
	double Period;
};
typedef cVabsekf::Vector MyVector;
typedef cVabsekf::Matrix MyMatrix;

#endif
