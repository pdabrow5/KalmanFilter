/*
 * OrientationEKF.hpp
 *
 *  Created on: Dec 8, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_EXTENDEDKALMAN_ORIENTATIONEKF_HPP_
#define UTIL_ALGORITHMS_EXTENDEDKALMAN_ORIENTATIONEKF_HPP_
#include "EKF.hpp"

using namespace Mat;

namespace Algorithms
{
constexpr float EarthRadius = 6371000; //meters
constexpr float _ER = 1 / EarthRadius;

constexpr u_short OrientationEKF_stateLen = 4;
constexpr u_short OrientationEKF_controlLen = 3;
constexpr u_short OrientationEKF_meassurementLen = 6;

using OriEKFBase = ExtendedKalmanFilter<OrientationEKF_stateLen, OrientationEKF_controlLen, OrientationEKF_meassurementLen>;

class OrientationEKF: public OriEKFBase
{
public:
	using OriEKFBase::StateVec;
	using OriEKFBase::ControlVec;
	using OriEKFBase::MeassurementVec;
	using OriEKFBase::StateCovarianceMatrix;
	using OriEKFBase::ControlCovarianceMatrix;
	using OriEKFBase::MeasurementCovarianceMatrix;
	using OriEKFBase::OriEKFBase;
	virtual void Update(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov, float time) override;
	void Initialise(const Matrix<3, 1>& acc, const Matrix<3, 1>& mag, float time);
protected:
//Helper functions
	//Prediction step
	virtual StateVec _f(const ControlVec& controlVec, float time) const override;
	virtual StateCovarianceMatrix _F(const ControlVec& controlVec, float time) const override;
	virtual StateCovarianceMatrix _Q(const ControlVec& controlVec, const ControlCovarianceMatrix& controlCov, float time) const override;
	//Correction step
	virtual MeassurementVec _h() const override {return MeassurementVec{-1.0f};} //not used
	virtual Matrix<OrientationEKF_meassurementLen, OrientationEKF_stateLen> _H() const override {return Matrix<OrientationEKF_meassurementLen, OrientationEKF_stateLen>{-1.0f};}//not used
	virtual MeasurementCovarianceMatrix _InvertMatrix(const MeasurementCovarianceMatrix& matrix) const override {return MeasurementCovarianceMatrix{-1.0f};} //not used
	//Custom update functions
	void _UpdateAcc(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov);
	void _UpdateMag(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov);
	Matrix<3,3> _Invert3x3Matrix(const Matrix<3,3>& matrix) const;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_EXTENDEDKALMAN_ORIENTATIONEKF_HPP_ */
