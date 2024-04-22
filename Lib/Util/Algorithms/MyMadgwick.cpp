/*
 * MyMadgwick.cpp
 *
 *  Created on: Apr 14, 2024
 *      Author: pawda
 */

#include "MyMadgwick.hpp"

#include <math.h>

#include <stdio.h>

using namespace Mat;

namespace Algorithms
{

Matrix<4, 6> Jgbt(const Quaternion& q, const Quaternion& b);
Matrix<6, 1> fgb(const Quaternion& q, const Quaternion& a, const Quaternion& b, const Quaternion& m);

MadgwickFilter::MadgwickFilter(float beta): _beta(beta)
{
	_q = {1.0f, 0.0f, 0.0f, 0.0f};
}

void MadgwickFilter::Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float time)
{
	static float last_time{time};
	float deltat = time - last_time;
	last_time = time;

	//printf("Myfilter: %f\n\r", deltat);

	Quaternion w{0.0f, gx, gy, gz};
	Quaternion Sm{0.0f, mx, my, mz}; Sm.Normalise();
	Quaternion Sa{0.0f, ax, ay, az}; Sa.Normalise();
	Quaternion q_wt = 0.5f * (_q * w);
	Quaternion Eh = _q * Sm * _q.Inversed();
	Quaternion Eb{0.0f, 0.0f, 0.0f, 0.0f};
	Eb.x = sqrt(Eh.x*Eh.x + Eh.y*Eh.y); Eb.z = Eh.z;
	//printf("Myfilter: %f\t%f\n\r", Eb.x, Eb.z);
	//printf("Myfilter: %f\t%f\t%f\n\r", Sm.x, Sm.y, Sm.z);
	//printf("Myfilter: bx: %f,\tbz: %f\n\r", Eb.x, Eb.z);
	auto Jgbt_fgb = Jgbt(_q, Eb) * fgb(_q, Sa, Eb * 0.5, Sm);
	Quaternion qJgbt_fgb{Jgbt_fgb(0,0), Jgbt_fgb(1,0), Jgbt_fgb(2,0), Jgbt_fgb(3,0)};
	qJgbt_fgb.Normalise();
	//printf("Myfilter: %f\t%f\t%f\t%f\n\r", qJgbt_fgb.w, qJgbt_fgb.x, qJgbt_fgb.y, qJgbt_fgb.z);
	_q = _q + (q_wt - _beta*qJgbt_fgb) * deltat;
	//_q = {0.923871f, -0.000442f, -0.000182f, -0.382663f}; ///DEBUG
	_q.Normalise();
}

Quaternion MadgwickFilter::GetOrientation() const
{
	return _q;
}

float MadgwickFilter::GetRoll() const
{
	float roll = atan2(2.0f * (_q.w * _q.x + _q.y * _q.z), _q.w * _q.w - _q.x * _q.x - _q.y * _q.y + _q.z * _q.z);
	return roll * 57.29578f;
}

float MadgwickFilter::GetPitch() const
{
	float pitch = -asin(2.0f * (_q.x * _q.z - _q.w * _q.y));
	return pitch * 57.29578f;
}

float MadgwickFilter::GetYaw() const
{
	float yaw = atan2(2.0f * (_q.x * _q.y + _q.w * _q.z), _q.w * _q.w + _q.x * _q.x - _q.y * _q.y - _q.z * _q.z);
	return yaw * 57.29578f + 180.0f;
}

Matrix<4, 6> Jgbt(const Quaternion& q, const Quaternion& b)
{
	Matrix<4, 6> R
	{{
		-2.0f*q.y,	2.0f*q.x,	0.0f,		-2.0f*b.z*q.y,					-2.0f*b.x*q.z + 2.0f*b.z*q.x,	2.0f*b.x*q.y,
		2.0f*q.z,	2.0f*q.w,	-4.0f*q.x,	2.0f*b.z*q.z,					2.0f*b.x*q.y + 2.0f*b.z*q.w,	2.0f*b.x*q.z - 4.0f*b.z*q.x,
		-2.0f*q.w,	2.0f*q.z,	-4.0f*q.y,	-4.0f*b.x*q.y - 2.0f*b.z*q.w,	2.0f*b.x*q.x + 2.0f*b.z*q.z,	2.0f*b.x*q.w - 4.0f*b.z*q.y,
		2.0f*q.x,	2.0f*q.y,	0.0f,		-4.0f*b.x*q.z + 2.0f*b.z*q.x,	-2.0f*b.x*q.w + 2.0f*b.z*q.y,	2.0f*b.x*q.x
	}};
	return R;
}

Matrix<6, 1> fgb(const Quaternion& q, const Quaternion& a, const Quaternion& b, const Quaternion& m)
{
	Matrix<6, 1> R
	{{
		2.0f*(q.x*q.z - q.w*q.y) - a.x,
		2.0f*(q.w*q.x + q.y*q.z) - a.y,
		2.0f*(0.5f - q.x*q.x - q.y*q.y) - a.z,
		2.0f*b.x*(0.5f - q.y*q.y - q.z*q.z) + 2.0f*b.z*(q.x*q.z - q.w*q.y) - m.x,
		2.0f*b.x*(q.x*q.y - q.w*q.z) + 2.0f*b.z*(q.w*q.x + q.y*q.z) - m.y,
		2.0f*b.x*(q.w*q.y + q.x*q.z) + 2.0f*b.z*(0.5f - q.x*q.x - q.y*q.y) - m.z
	}};
	return R;
}

} //namespace Algorithms
