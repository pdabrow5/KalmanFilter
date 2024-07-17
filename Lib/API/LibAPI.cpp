/*
 * LibAPI.cpp
 *
 *  Created on: Apr 6, 2024
 *      Author: pawda
 */

#include "LibAPI.h"

#include "../Util/Algorithms/MadgwickOriginal.h"
#include "../Util/Algorithms/MyMadgwick.hpp"
#include "../Util/Algorithms/FusionAlgorithm.hpp"
#include "../Util/Algorithms/ExtendedKalman/AHRSKalman.hpp"
#include "Quaternion.hpp"
#include "calibrate.h"
#include "logger.h"

#include <stdio.h>
#include "Vector.hpp"

Algorithms::FusionAlgorithm Fusion{};
Algorithms::AHRSKalman Kalman{};

Mat::Matrix<3, 1> MagCal({0, 0, 0});
Mat::Matrix<3, 1> MagRaw({0, 0, 0});

Mat::Matrix<3, 1> GyroCal({0, 0, 0});
Mat::Matrix<3, 1> GyroRaw({0, 0, 0});

Mat::Matrix<3, 1> AccCal({0, 0, 0});
Mat::Matrix<3, 1> AccRaw({0, 0, 0});

Mat::Matrix<3, 1> GyroRawMean({0, 0, 0});
Mat::Matrix<3, 1> GyroCalMean({0, 0, 0});

Mat::Quaternion G{0.0f, 0.0f, 0.0f, 9.80655f};
Mat::Quaternion Q{};
Mat::Quaternion _acceleration{0.0f, 0.0f, 0.0f, 0.0f};
Mat::Quaternion _velocity{0.0f, 0.0f, 0.0f, 0.0f};
Mat::Quaternion _position{0.0f, 0.0f, 0.0f, 0.0f};

Mat::Matrix<3, 1> angle = 0.0f;
Mat::Matrix<3, 1> speed = 0.0f;
Mat::Matrix<3, 1> meanMag = 0.0f;

float al = 0.01f;

uint8_t InitAlgorithms(const AGMSensorData* sensorData)
{
	printf("InitAlgorithms\n\r");
	AccRaw(0,0) = sensorData->Acc.x;
	AccRaw(1,0) = sensorData->Acc.y;
	AccRaw(2,0) = sensorData->Acc.z;
	MagRaw(0,0) = sensorData->Mag.x;
	MagRaw(1,0) = sensorData->Mag.y;
	MagRaw(2,0) = sensorData->Mag.z;
	AccCal = CalibrateAcc(AccRaw);
	MagCal = CalibrateMag(MagRaw);

	Kalman.InitialiseKalman(AccCal, MagCal);

	return 1;
}

uint8_t ResetKinematics()
{
	Fusion.ResetKinematics();
	_acceleration = {0.0f, 0.0f, 0.0f, 0.0f};
	_velocity = {0.0f, 0.0f, 0.0f, 0.0f};
	_position = {0.0f, 0.0f, 0.0f, 0.0f};

	angle = 0.0f;
	//speed = 0.0f;

	al = al * 0.5f;
	return 1;
}

uint8_t MadgwickUpdate(const AGMSensorData* sensorData)
{
	if(sensorData != nullptr)
	{
		static float last_time{sensorData->SensorTime * 0.001f};
		float currTime = sensorData->SensorTime * 0.001f;
		float deltat = (currTime - last_time);
		last_time = currTime;

		AccRaw(0,0) = sensorData->Acc.x;
		AccRaw(1,0) = sensorData->Acc.y;
		AccRaw(2,0) = sensorData->Acc.z;
		MagRaw(0,0) = sensorData->Mag.x;
		MagRaw(1,0) = sensorData->Mag.y;
		MagRaw(2,0) = sensorData->Mag.z;
		GyroRaw(0,0) = sensorData->Gyro.x;
		GyroRaw(1,0) = sensorData->Gyro.y;
		GyroRaw(2,0) = sensorData->Gyro.z;
		AccCal = CalibrateAcc(AccRaw);
		MagCal = CalibrateMag(MagRaw);
		GyroCal = CalibrateGyro(GyroRaw);

		float mx = 0;
		float my = 0;
		float mz = 0;
		float md = 1.0f / MagCal.VecNorm();
		mx = MagCal(0,0) * md;
		my = MagCal(1,0) * md;
		mz = MagCal(2,0) * md;
		//printf("Raw:0,0,0,0,0,0,%f,%f,%f\n\r", MagCal(0,0), MagCal(1,0), MagCal(2,0));
		//printf("%f,	%f,	%f,	%f\n\r", mx, my, mz, md)
		//return 1;

		//angle += (speed + GyroCal) * 0.5f * deltat;
		//speed = GyroCal * al + speed * (1.0f - al);
		//speed = GyroCal;

		//meanMag = MagCal * al + meanMag * (1.0f - al);
		//float norm = sqrt(meanMag(0,0)*meanMag(0,0) + meanMag(1,0)*meanMag(1,0) + meanMag(2,0)*meanMag(2,0));
		//printf("%f \t%f \t%f \t%f\n\r", meanMag(0,0), meanMag(1,0), meanMag(2,0), norm);
		//return 1;

//		filterUpdate(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
//								AccCal(0,0), AccCal(1,0), AccCal(2,0),
//								MagCal(0,0), MagCal(1,0), MagCal(2,0),
//								sensorData->SensorTime / 1000.0f);
		Kalman.UpdateState(GyroCal, sensorData->SensorTime / 1000.0f);
		Kalman.CorrectStateAcc(AccCal, sensorData->SensorTime / 1000.0f);
		Kalman.CorrectStateMag(MagCal, sensorData->SensorTime / 1000.0f);
		Mat::Quaternion newAcceleration = {0, AccCal(0,0), AccCal(1,0), AccCal(2,0)};
//		Q.w = GetW();
//		Q.x = GetX();
//		Q.y = GetY();
//		Q.z = GetZ();
		Q = Kalman.GetState();
		newAcceleration = (Q * newAcceleration * Q.Conjugate()) - G;

		//Calculate new Velocity and Position
//		_position = _position + _velocity * deltat + ((newAcceleration + _acceleration * 2.0f) * (deltat * deltat / 6.0f));
//		_velocity = _velocity + ((_acceleration + newAcceleration) * (0.5f * deltat));
		_acceleration = newAcceleration;



		Fusion.UpdateIMU(*sensorData);
		//printf("My: %f, \t%f, \t%f, \tOrig: %f, \t%f, \t%f\n\r", Fusion.GetRoll(), Fusion.GetPitch(), Fusion.GetYaw(), getRoll(), getPitch(), getYaw());
		auto acc = Fusion.GetAcceleration();
		printf("Orig: %f, \t%f, \t%f, \tMy: %f, \t%f, \t%f\n\r",
				_acceleration.x, _acceleration.y, _acceleration.z,
				acc.x, acc.y, acc.z);

		return 1;
	}
	return 0;
}

Vec3 GetPosition()
{
	Vec3 result;
	auto pos = Fusion.GetPosition();
	result.x = pos.x;
	result.y = pos.y;
	result.z = pos.z;
	return result;
}

Vec3 GetVelocity()
{
	Vec3 result;
	auto pos = Fusion.GetVelocity();
	result.x = pos.x;
	result.y = pos.y;
	result.z = pos.z;
	return result;
}

Vec3 GetAcceleration()
{
	Vec3 result;
	auto pos = Fusion.GetAcceleration();
	result.x = pos.x;
	result.y = pos.y;
	result.z = pos.z;
	return result;
}
