/*
 * OrientationEKF.cpp
 *
 *  Created on: Dec 8, 2024
 *      Author: pawda
 */
#include "OrientationEKF.hpp"

#include <math.h>

namespace
{
	float sgn(float val)
	{
		return ((0.0f < val) - (0.0f > val));
	}
}

namespace Algorithms
{

constexpr float magDip = 1.191455d;
const V_Vector<3> _r{{0.0f, cos(magDip), -sin(magDip)}};
const V_Vector<3> r{{0.0f, _r(1) * (1 / _r.Norm()), _r(2) * (1 / _r.Norm())}};
const V_Vector<3> r2{{0.0f, r(1)*2.0f, r(2)*2.0f}};

void OrientationEKF::Update(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov, float time)
{
	float norm_1 = 1.0f / _state.Norm();
	_state*= norm_1;
	_UpdateAcc(meassurementVec, meassurementCov);
	_UpdateMag(meassurementVec, meassurementCov);
	_time = time;
}

void OrientationEKF::Initialise(const Matrix<3, 1>& acc, const Matrix<3, 1>& mag, float time)
{
	float ax{acc(0,0)}, ay{acc(1,0)}, az{acc(2,0)};
	float mx{mag(0,0)}, my{mag(1,0)}, mz{mag(2,0)};

	float amx{ay*mz - az*my};
	float amy{az*mx - ax*mz};
	float amz{ax*my - ay*mx};

	float amax{amy*az - amz*ay};
	float amay{amz*ax - amx*az};
	float amaz{amx*ay - amy*ax};

	float ad = 1.0f / sqrt(ax*ax + ay*ay +az*az);
	ax *= ad; ay *= ad; az *= ad;

	float amd = 1.0f / sqrt(amx*amx + amy*amy +amz*amz);
	amx *= amd; amy *= amd; amz *= amd;

	float amad = 1.0f / sqrt(amax*amax + amay*amay +amaz*amaz);
	amax *= amad; amay *= amad; amaz *= amad;

	_state(0) = 0.5f * sqrt(amax + amy + az + 1.0f);
	_state(1) = 0.5f * sgn(amz - ay) * sqrt(amax - amy - az + 1.0f);
	_state(2) = 0.5f * sgn(ax - amaz) * sqrt(amy - amax - az + 1.0f);
	_state(3) = 0.5f * sgn(amz - ay) * sqrt(az - amax - amy + 1.0f);
	_time = time;
	_stateCovMatrix = Eye<OrientationEKF_stateLen>(1.0f);
}

void OrientationEKF::_UpdateAcc(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov)
{
	V_Vector<3> z{{meassurementVec(0), meassurementVec(1), meassurementVec(2)}};
	z *= 1.0f / z.Norm();
	//[0, 0, 1]
	V_Vector<3> h{{
		2.0f * (_state(1)*_state(3) - _state(0)*_state(2)),
		2.0f * (_state(0)*_state(1) + _state(2)*_state(3)),
		2.0f * (0.5f - _state(1)*_state(1) - _state(2)*_state(2))
	}};
	Matrix<3, OrientationEKF_stateLen> H{{
		-2.0f*_state(2), 2.0f*_state(3), -2.0f*_state(0), 2.0f*_state(1),
		2.0f*_state(1), 2.0f*_state(0), 2.0f*_state(3), 2.0f*_state(2),
		2.0f*_state(0), -2.0f*_state(1), -2.0f*_state(2), 2.0f*_state(3)
	}};
	static Matrix<3, 3> R{0.0f};
	R(0,0) = meassurementCov(0,0); R(1,1) = meassurementCov(1,1); R(2,2) = meassurementCov(2,2);
	auto S = H * _stateCovMatrix * H.Transposed() + R;
	auto K = _stateCovMatrix * H.Transposed() * _Invert3x3Matrix(S);
	_state += K * (z - h);
	_stateCovMatrix = (Eye<OrientationEKF_stateLen>(1.0f) - K*H) * _stateCovMatrix;
}

void OrientationEKF::_UpdateMag(const MeassurementVec& meassurementVec, const MeasurementCovarianceMatrix& meassurementCov)
{
	V_Vector<3> z{{meassurementVec(3), meassurementVec(4), meassurementVec(5)}};
	z *= 1.0f / z.Norm();
	V_Vector<3> h{{
		(r2(1)*(_state(0)*_state(3) + _state(1)*_state(2)) + r2(2)*(_state(1)*_state(3) - _state(0)*_state(2))),
		(r2(1)*(0.5f - _state(1)*_state(1) - _state(3)*_state(3)) + r2(2)*(_state(0)*_state(1) + _state(2)*_state(3))),
		(r2(1)*(_state(2)*_state(3) - _state(0)*_state(1)) + r2(2)*(0.5f - _state(1)*_state(1) - _state(2)*_state(2)))
	}};
	Matrix<3, OrientationEKF_stateLen> H{{
		(r2(1)*_state(3) - r2(2)*_state(2)), (r2(1)*_state(2) + r2(2)*_state(3)), (r2(1)*_state(1) - r2(2)*_state(0)), (r2(1)*_state(0) + r2(2)*_state(1)),
		(r2(1)*_state(0) + r2(2)*_state(1)), (-r2(1)*_state(1) + r2(2)*_state(0)), (r2(1)*_state(2) + r2(2)*_state(3)), (-r2(1)*_state(3) + r2(2)*_state(2)),
		(-r2(1)*_state(1) + r2(2)*_state(0)), (-r2(1)*_state(0) - r2(2)*_state(1)), (r2(1)*_state(3) - r2(2)*_state(2)), (r2(1)*_state(2) + r2(2)*_state(3))
	}};
	static Matrix<3, 3> R{0.0f};
	R(0,0) = meassurementCov(3,3); R(1,1) = meassurementCov(4,4); R(2,2) = meassurementCov(5,5);
	auto S = H * _stateCovMatrix * H.Transposed() + R;
	auto K = _stateCovMatrix * H.Transposed() * _Invert3x3Matrix(S);
	_state += K * (z - h);
	_stateCovMatrix = (Eye<OrientationEKF_stateLen>(1.0f) - K*H) * _stateCovMatrix;
}

OrientationEKF::StateVec OrientationEKF::_f(const ControlVec& controlVec, float time) const
{
	const float dt = time - _time;
	const float dt_2 = dt*0.5f;
	return StateVec{{
		_state(0) - dt_2*controlVec(0)*_state(1) - dt_2*controlVec(1)*_state(2) - dt_2*controlVec(2)*_state(3),
		_state(1) + dt_2*controlVec(0)*_state(0) - dt_2*controlVec(1)*_state(3) + dt_2*controlVec(2)*_state(2),
		_state(2) + dt_2*controlVec(0)*_state(3) + dt_2*controlVec(1)*_state(0) - dt_2*controlVec(2)*_state(1),
		_state(3) - dt_2*controlVec(0)*_state(2) + dt_2*controlVec(1)*_state(1) + dt_2*controlVec(2)*_state(0)
	}};
}

OrientationEKF::StateCovarianceMatrix OrientationEKF::_F(const ControlVec& controlVec, float time) const
{
	const float dt = time - _time;
	const float dt_2 = dt*0.5f;
	return StateCovarianceMatrix{{
		1.0f, -dt_2*controlVec(0), -dt_2*controlVec(1), -dt_2*controlVec(2),
		dt_2*controlVec(0), 1.0f, dt_2*controlVec(2), -dt_2*controlVec(1),
		dt_2*controlVec(1), -dt_2*controlVec(2), 1.0f, dt_2*controlVec(0),
		dt_2*controlVec(2), dt_2*controlVec(1), -dt_2*controlVec(0), 1.0f
	}};
}

OrientationEKF::StateCovarianceMatrix OrientationEKF::_Q(const ControlVec& controlVec, const ControlCovarianceMatrix& controlCov, float time) const
{
	const float dt = time - _time;
	const float dt_2 = dt*0.5f;
	Matrix<OrientationEKF_stateLen, OrientationEKF_controlLen> W{{
		-_state(1), -_state(2), -_state(3),
		_state(0), -_state(3), _state(2),
		_state(3), _state(0), -_state(1),
		-_state(2), _state(1), _state(0)
	}};
	W *= dt_2;
	return W*controlCov*W.Transposed();
}

Matrix<3,3> OrientationEKF::_Invert3x3Matrix(const Matrix<3,3>& matrix) const
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







