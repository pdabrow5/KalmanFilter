/*
 * EKF.hpp
 *
 *  Created on: Dec 4, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_EXTENDEDKALMAN_EKF_HPP_
#define UTIL_ALGORITHMS_EXTENDEDKALMAN_EKF_HPP_

#include "Vector.hpp"

using namespace Mat;

namespace Algorithms
{

template <u_short stateLen, u_short controlLen, u_short meassurementLen>
class ExtendedKalmanFilter
{
public:
	//Aliases to be used outside class
	using StateVec = V_Vector<stateLen>;
	using ControlVec = V_Vector<controlLen>;
	using MeassurementVec = V_Vector<meassurementLen>;
	using StateCovarianceMatrix = Matrix<stateLen, stateLen>;
	using ControlCovarianceMatrix = Matrix<controlLen, controlLen>;
	using MeasurementCovarianceMatrix = Matrix<meassurementLen, meassurementLen>;

	ExtendedKalmanFilter(const StateVec& state, const StateCovarianceMatrix& stateCovMatrix, float time)
		: _state{state}, _stateCovMatrix{stateCovMatrix}, _time{time}{}
	virtual void Init(const StateVec& state, const StateCovarianceMatrix& stateCovMatrix, float time)
		{_state = state;
		_stateCovMatrix = stateCovMatrix;
		_time = time;}

	virtual void Predict(const ControlVec& controlVec, const ControlCovarianceMatrix& controlCov, float time);
	virtual void Update(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov, float time);

	const StateVec& GetState() const {return _state;}
	const StateCovarianceMatrix& GetStateCov() const {return _stateCovMatrix;}
	float GetTime() const {return _time;}
protected:
//Helper functions
	//Prediction step
	virtual StateVec _f(const ControlVec& controlVec, float time) const = 0;
	virtual StateCovarianceMatrix _F(const ControlVec& controlVec, float time) const = 0;
	virtual StateCovarianceMatrix _Q(const ControlVec& controlVec, const ControlCovarianceMatrix& controlCov, float time) const = 0;
	//Correction step
	virtual MeassurementVec _h() const = 0;
	virtual Matrix<meassurementLen, stateLen> _H() const = 0;
	virtual MeasurementCovarianceMatrix _InvertMatrix(const MeasurementCovarianceMatrix& matrix) const = 0;

//Protected variables
	StateVec _state;
	StateCovarianceMatrix _stateCovMatrix;
	float _time;
};

template <u_short stateLen, u_short controlLen, u_short meassurementLen>
void ExtendedKalmanFilter<stateLen, controlLen, meassurementLen>::Predict(
		const ControlVec& controlVec,
		const ControlCovarianceMatrix& controlCov,
		float time)
{
	const StateCovarianceMatrix F = _F(controlVec, time);
	_stateCovMatrix = F * _stateCovMatrix * F.Transposed() + _Q(controlVec, controlCov, time);
	_state = _f(controlVec, time);
	_time = time;
}

template <u_short stateLen, u_short controlLen, u_short meassurementLen>
void ExtendedKalmanFilter<stateLen, controlLen, meassurementLen>::Update(
		const MeassurementVec& meassurementVec,
		const MeasurementCovarianceMatrix& meassurementCov,
		float time)
{
	auto H = _H();
	auto S = H * _stateCovMatrix * H.Transposed() + meassurementCov;
	auto K = _stateCovMatrix * H.Transposed() *_InvertMatrix(S);
	_state += K * (meassurementVec - _h());
	_stateCovMatrix = (Eye<stateLen>(1.0f) - K*H) * _stateCovMatrix;
	_time = time;
}

}//namespace Algorithms


#endif /* UTIL_ALGORITHMS_EXTENDEDKALMAN_EKF_HPP_ */
