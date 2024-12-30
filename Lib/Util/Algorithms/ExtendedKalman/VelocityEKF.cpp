/*
 * VelocityEKF.cpp
 *
 *  Created on: Dec 7, 2024
 *      Author: pawda
 */
#include "VelocityEKF.hpp"

#include <math.h>
#include "constants.h"

namespace Algorithms
{

void VelocityEKF::Update(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov, float time)
{
	_UpdateVelocity(meassurementVec, meassurementCov);
	_UpdatePosition(meassurementVec, meassurementCov);
	_time = time;
}

void VelocityEKF::_UpdateVelocity(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov)
{
	V_Vector<3> z{{meassurementVec(0), meassurementVec(1), meassurementVec(2)}};
	V_Vector<3> h{{_state(0), _state(1), _state(2)}};
	static Matrix<3, VelocityEKF_stateLen> H{0.0f};
	H(0,0) = 1.0f; H(1,1) = 1.0f; H(2,2) = 1.0f;
	static Matrix<3, 3> R{0.0f};
	R(0,0) = meassurementCov(0,0); R(1,1) = meassurementCov(1,1); R(2,2) = meassurementCov(2,2);

	auto S = H * _stateCovMatrix * H.Transposed() + R;
	auto K = _stateCovMatrix * H.Transposed() * _Invert3x3Matrix(S);
	_state += K * (z - h);
	_stateCovMatrix = (Eye<VelocityEKF_stateLen>(1.0f) - K*H) * _stateCovMatrix;
}

void VelocityEKF::_UpdatePosition(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov)
{
	V_Vector<3> z{{meassurementVec(3), meassurementVec(4), meassurementVec(5)}};
	V_Vector<3> h{{_state(3), _state(4), _state(5)}};
	static Matrix<3, VelocityEKF_stateLen> H{0.0f};
	H(0,3) = 1.0f; H(1,4) = 1.0f; H(2,5) = 1.0f;
	static Matrix<3, 3> R{0.0f};
	R(0,0) = meassurementCov(3,3)*_ER; R(1,1) = meassurementCov(4,4)*_ER / cos(_state(3)); R(2,2) = meassurementCov(5,5);

	auto S = H * _stateCovMatrix * H.Transposed() + R;
	auto K = _stateCovMatrix * H.Transposed() * _Invert3x3Matrix(S);
	_state += K * (z - h);
	_stateCovMatrix = (Eye<VelocityEKF_stateLen>(1.0f) - K*H) * _stateCovMatrix;
}

VelocityEKF::StateVec VelocityEKF::_f(const ControlVec& controlVec, float time) const
{
	const float dt = time - _time;
	const float dt2 = dt*dt;
	const float dt2_2 = dt2 * 0.5f;
	StateVec result;
	result(0) = _state(0) + controlVec(0)*dt;
	result(1) = _state(1) + controlVec(1)*dt;
	result(2) = _state(2) + controlVec(2)*dt;
	result(3) = _state(3) + (_state(1)*dt + controlVec(1)*dt2_2)*_ER;
	result(4) = _state(4) + (_state(0)*dt + controlVec(0)*dt2_2)*_ER / cos(_state(3));
	result(5) = _state(5) + _state(2)*dt + controlVec(2)*dt2_2;
	return result;
}

VelocityEKF::StateCovarianceMatrix VelocityEKF::_F(const ControlVec& controlVec, float time) const
{
	const float dt = time - _time;
	const float dt2 = dt*dt;
	const float dt2_2 = dt2 * 0.5f;
	StateCovarianceMatrix result = Eye<VelocityEKF_stateLen>(1.0f);
	result(3, 1) = dt*_ER;
	result(4, 0) = dt*_ER / cos(_state(3)); result(4, 3) = (_state(0)*dt + dt2_2*controlVec(0)) * sin(_state(3)) * _ER / (cos(_state(3)) * cos(_state(3)));
	result(5, 2) = dt;
	return result;
}

VelocityEKF::StateCovarianceMatrix VelocityEKF::_Q(const ControlVec& controlVec, const ControlCovarianceMatrix& controlCov, float time) const
{
	const float dt = time - _time;
	const float dt2 = dt*dt;
	const float dt2_2 = dt2 * 0.5f;
	Matrix<VelocityEKF_stateLen, VelocityEKF_controlLen> W{0.0f};
	W(0,0) = dt; W(1,1) = dt; W(2,2) = dt;
	W(3,1) = dt2_2*_ER;
	W(4,0) = W(3,1) / cos(_state(3));
	W(5,2) = dt2_2;
	return W*controlCov*W.Transposed();
}

VelocityEKF::MeassurementVec VelocityEKF::_h() const
{
	return _state;
}

Matrix<VelocityEKF_meassurementLen, VelocityEKF_stateLen> VelocityEKF::_H() const
{
	return Eye<VelocityEKF_meassurementLen>(1.0f);
}

VelocityEKF::MeasurementCovarianceMatrix VelocityEKF::_InvertMatrix(const MeasurementCovarianceMatrix& matrix) const
{
	auto result = matrix;
	return result;
}

Matrix<3,3> VelocityEKF::_Invert3x3Matrix(const Matrix<3,3>& matrix) const
{
	static float a_1_1, a_1_2, a_1_3, a_2_1, a_2_2, a_2_3, a_3_1, a_3_2, a_3_3;
	a_1_1 = matrix(0,0); a_1_2 = matrix(0,1); a_1_3 = matrix(0,2);
	a_2_1 = matrix(1,0); a_2_2 = matrix(1,1); a_2_3 = matrix(1,2);
	a_3_1 = matrix(2,0); a_3_2 = matrix(2,1); a_3_3 = matrix(2,2);
	float det = (a_1_1*a_2_2*a_3_3 - a_1_1*a_2_3*a_3_2 - a_1_2*a_2_1*a_3_3 + a_1_2*a_2_3*a_3_1 + a_1_3*a_2_1*a_3_2 - a_1_3*a_2_2*a_3_1);
	float det1 = 1.0f / det;
	static Matrix<3, 3> result =
	{{
		(a_2_2*a_3_3 - a_2_3*a_3_2)*det1, -(a_1_2*a_3_3 - a_1_3*a_3_2)*det1,  (a_1_2*a_2_3 - a_1_3*a_2_2)*det1,
		-(a_2_1*a_3_3 - a_2_3*a_3_1)*det1,  (a_1_1*a_3_3 - a_1_3*a_3_1)*det1, -(a_1_1*a_2_3 - a_1_3*a_2_1)*det1,
		(a_2_1*a_3_2 - a_2_2*a_3_1)*det1, -(a_1_1*a_3_2 - a_1_2*a_3_1)*det1,  (a_1_1*a_2_2 - a_1_2*a_2_1)*det1,
	}};
	return result;
}

} //namespace Algorithms



