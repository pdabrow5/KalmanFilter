/*
 * AHRSKalman.cpp
 *
 *  Created on: Jun 25, 2024
 *      Author: pawda
 */

#include "AHRSKalman.hpp"

#include <math.h>

using namespace Mat;

namespace Algorithms
{

float sgn(float val)
{
	return ((0.0f < val) - (0.0f > val));
}

const Matrix<3, 3>& Inverse3_3Matrix(const Matrix<3, 3>& mat)
{
	static Matrix<3, 3> result;
	static float a_1_1, a_1_2, a_1_3, a_2_1, a_2_2, a_2_3, a_3_1, a_3_2, a_3_3;
	a_1_1 = mat(0,0); a_1_2 = mat(0,1); a_1_3 = mat(0,2);
	a_2_1 = mat(1,0); a_2_2 = mat(1,1); a_2_3 = mat(1,2);
	a_3_1 = mat(2,0); a_3_2 = mat(2,1); a_3_3 = mat(2,2);
	float det = (a_1_1*a_2_2*a_3_3 - a_1_1*a_2_3*a_3_2 - a_1_2*a_2_1*a_3_3 + a_1_2*a_2_3*a_3_1 + a_1_3*a_2_1*a_3_2 - a_1_3*a_2_2*a_3_1);
	float det1 = 1.0f / det;
	result =
	{{
		(a_2_2*a_3_3 - a_2_3*a_3_2)*det1, -(a_1_2*a_3_3 - a_1_3*a_3_2)*det1,  (a_1_2*a_2_3 - a_1_3*a_2_2)*det1,
		-(a_2_1*a_3_3 - a_2_3*a_3_1)*det1,  (a_1_1*a_3_3 - a_1_3*a_3_1)*det1, -(a_1_1*a_2_3 - a_1_3*a_2_1)*det1,
		(a_2_1*a_3_2 - a_2_2*a_3_1)*det1, -(a_1_1*a_3_2 - a_1_2*a_3_1)*det1,  (a_1_1*a_2_2 - a_1_2*a_2_1)*det1,
	}};
	return result;
}

const Quaternion& AHRSKalman::GetState() const
{
	return _X;
}

const Matrix<4,4>& AHRSKalman::GetNoiseCovariance() const
{
	return _P;
}

void AHRSKalman::InitialiseKalman(const Matrix<3, 1>& acc, const Matrix<3, 1>& mag, float time)
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

	_X.w = 0.5f * sqrt(amax + amy + az + 1.0f);
	_X.x = 0.5f * sgn(amz - ay) * sqrt(amax - amy - az + 1.0f);
	_X.y = 0.5f * sgn(ax - amaz) * sqrt(amy - amax - az + 1.0f);
	_X.z = 0.5f * sgn(amz - ay) * sqrt(az - amax - amy + 1.0f);
	_lastUpdateTime = time;
	_lastCorrectionTime = time;
}

void AHRSKalman::UpdateState(const Matrix<3, 1>& U, float time)
{
//Update State
	float hdt = (time - _lastUpdateTime) * 0.5f; //Half-delta-time
	_lastUpdateTime = time;
	static Matrix<4, 4> F;
	static Matrix<4, 3> W;
	static Quaternion newX;
	float wx{U(0,0)}, wy{U(1,0)}, wz{U(2,0)};
	newX.w = _X.w + hdt*(-wx*_X.x - wy*_X.y - wz*_X.z);
	newX.x = _X.x + hdt*(wx*_X.w - wy*_X.z + wz*_X.y);
	newX.y = _X.y + hdt*(wx*_X.z + wy*_X.w - wz*_X.x);
	newX.z = _X.z + hdt*(-wx*_X.y + wy*_X.x + wz*_X.w);

	auto qW = _X * hdt;

	W =
	{{
		-qW.x, -qW.y, -qW.z,
		qW.w, -qW.z, qW.y,
		qW.z, qW.w, -qW.x,
		-qW.y, qW.x, qW.w
	}};

	_Q = W*W.Transposed() * _gyroNoiseVariance;

	F =
	{{
		1.0f, -hdt*wx, -hdt*wy, -hdt*wz,
		hdt*wx, 1.0f, hdt*wz, -hdt*wy,
		hdt*wy, -hdt*wz, 1.0f, hdt*wx,
		hdt*wz, hdt*wy, -hdt*wx, 1.0f
	}};

	_X = newX;
	_P = F * _P * F.Transposed() + _Q;
	_X = _X.Normalised();
}

void AHRSKalman::CorrectStateMag(const Matrix<3, 1>& mag, float time)
{
	float dip = 0.5f;
	static const float ry{cos(dip)}, rz{-sin(dip)};
	static Matrix<3, 1> Z;
	static Matrix<3, 1> h;
	static Matrix<3, 4> H;
	static Matrix<4, 3> K;
	static const Matrix<3, 3> R = Eye<3>(_magNoiseVariance);
	static Matrix<3, 3> S;
	static Matrix<4, 4> I = Eye<4>(1.0f);
	static Matrix<4, 1> res;
	static Quaternion add;
	float md = 1.0f / sqrt(mag(0,0)*mag(0,0) + mag(1,0)*mag(1,0) + mag(2,0)*mag(2,0));

	Z =
	{{
		mag(0,0)*md,
		mag(1,0)*md,
		mag(2,0)*md
	}};

	h =
	{{
		2.0f*(ry*(_X.w*_X.z + _X.x*_X.y) + rz*(_X.x*_X.z - _X.w*_X.y)),
		2.0f*(ry*(0.5f - _X.x*_X.x - _X.z*_X.z) + rz*(_X.w*_X.x + _X.y*_X.z)),
		2.0f*(ry*(_X.y*_X.z - _X.w*_X.x) + rz*(0.5f - _X.x*_X.x - _X.y*_X.y))
	}};

	H =
	{{
		2.0f*ry*_X.z - 2.0f*rz*_X.y, 2.0f*ry*_X.z + 2.0f*rz*_X.z, 2.0f*ry*_X.x - 2.0f*rz*_X.w, 2.0f*ry*_X.w + 2.0f*rz*_X.x,
		2.0f*ry*_X.w + 2.0f*rz*_X.x, -2.0f*ry*_X.x + 2.0f*rz*_X.w, 2.0f*ry*_X.y + 2.0f*rz*_X.z, -2.0f*ry*_X.z + 2.0f*rz*_X.y,
		-2.0f*ry*_X.x + 2.0f*rz*_X.w, -2.0f*ry*_X.w - 2.0f*rz*_X.x, 2.0f*ry*_X.z - 2.0f*rz*_X.y, 2.0f*ry*_X.y + 2.0f*rz*_X.z
	}};

	S = H*_P*H.Transposed() + R;
	K = _P*H.Transposed()*Inverse3_3Matrix(S);
	res = K*(Z - h);
	add.w = res(0,0); add.x = res(1,0); add.y = res(2,0); add.z = res(3,0);
	_X = _X + add;
	_P = (I - K*H)*_P;
}

void AHRSKalman::CorrectStateAcc(const Matrix<3, 1>& acc, float time)
{
	static const float gz{1.0f};
	static const float gz2 = 2.0f*gz;
	static Matrix<3, 1> Z;
	static Matrix<3, 1> h;
	static Matrix<3, 4> H;
	static Matrix<4, 3> K;
	static const Matrix<3, 3> R = Eye<3>(_accNoiseVariance);
	static Matrix<3, 3> S;
	static Matrix<4, 4> I = Eye<4>();
	static Matrix<4, 1> res;
	static Quaternion add;

	float gd = 1.0f / sqrt(acc(0,0)*acc(0,0) + acc(1,0)*acc(1,0) + acc(2,0)*acc(2,0));
	Z =
	{{
		acc(0,0)*gd,
		acc(1,0)*gd,
		acc(2,0)*gd
	}};

	h =
	{{
		gz2*(_X.x*_X.z - _X.w*_X.y),
		gz2*(_X.w*_X.x + _X.y*_X.z),
		gz2*(0.5f - _X.x*_X.x - _X.y*_X.y)
	}};

	H =
	{{
		-gz2*_X.y, gz2*_X.z, -gz2*_X.w, gz2*_X.x,
		gz2*_X.x, gz2*_X.w, gz2*_X.z, gz2*_X.y,
		gz2*_X.w, -gz2*_X.x, -gz2*_X.y, gz2*_X.z
	}};

	S = H*_P*H.Transposed() + R;
	K = _P*H.Transposed()*Inverse3_3Matrix(S);
	res = K*(Z - h);
	add.w = res(0,0); add.x = res(1,0); add.y = res(2,0); add.z = res(3,0);
	_X = _X + add;
	_P = (I - K*H)*_P;
}

float AHRSKalman::GetRoll() const
{
	float roll = atan2(2.0f * (_X.w * _X.x + _X.y * _X.z), _X.w * _X.w - _X.x * _X.x - _X.y * _X.y + _X.z * _X.z);
	return roll * 57.29578f;
}

float AHRSKalman::GetPitch() const
{
	float pitch = -asin(2.0f * (_X.x * _X.z - _X.w * _X.y));
	return pitch * 57.29578f;
}

float AHRSKalman::GetYaw() const
{
	float yaw = atan2(2.0f * (_X.x * _X.y + _X.w * _X.z), _X.w * _X.w + _X.x * _X.x - _X.y * _X.y - _X.z * _X.z);
	return yaw * 57.29578f + 180.0f;
}

}//namespace Algorithms
