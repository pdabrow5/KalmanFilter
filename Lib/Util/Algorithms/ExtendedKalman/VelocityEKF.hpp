/*
 * VelocityEKF.hpp
 *
 *  Created on: Dec 7, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_EXTENDEDKALMAN_VELOCITYEKF_HPP_
#define UTIL_ALGORITHMS_EXTENDEDKALMAN_VELOCITYEKF_HPP_
#include "EKF.hpp"

using namespace Mat;

namespace Algorithms
{
constexpr u_short VelocityEKF_stateLen = 6;
constexpr u_short VelocityEKF_controlLen = 3;
constexpr u_short VelocityEKF_meassurementLen = 6;

using VelEKFBase = ExtendedKalmanFilter<VelocityEKF_stateLen, VelocityEKF_controlLen, VelocityEKF_meassurementLen>;

class VelocityEKF: public VelEKFBase
{
public:
	using VelEKFBase::StateVec;
	using VelEKFBase::ControlVec;
	using VelEKFBase::MeassurementVec;
	using VelEKFBase::StateCovarianceMatrix;
	using VelEKFBase::ControlCovarianceMatrix;
	using VelEKFBase::MeasurementCovarianceMatrix;
	using VelEKFBase::VelEKFBase;
	virtual void Update(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov, float time) override;
protected:
//Helper functions
	//Prediction step
	virtual StateVec _f(const ControlVec& controlVec, float time) const override;
	virtual StateCovarianceMatrix _F(const ControlVec& controlVec, float time) const override;
	virtual StateCovarianceMatrix _Q(const ControlVec& controlVec, const ControlCovarianceMatrix& controlCov, float time) const override;
	//Correction step
	virtual MeassurementVec _h() const override; //not used
	virtual Matrix<VelocityEKF_meassurementLen, VelocityEKF_stateLen> _H() const override; //not used
	virtual MeasurementCovarianceMatrix _InvertMatrix(const MeasurementCovarianceMatrix& matrix) const override; //not used
	//Custom update functions
	void _UpdateVelocity(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov);
	void _UpdatePosition(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov);
	Matrix<3,3> _Invert3x3Matrix(const Matrix<3,3>& matrix) const;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_EXTENDEDKALMAN_VELOCITYEKF_HPP_ */
