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
#include "constants.h"
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
	Fusion.InitState(AccCal, MagCal, (float)(HAL_GetTick()) * ms2s);
	Kalman.InitialiseKalman(AccCal, MagCal, (float)(HAL_GetTick()) * ms2s);
	return 1;
}

uint8_t ResetKinematics()
{
	//Fusion.ResetKinematics();
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
		//GyroCal = {{0.0f, 0.0f, 0.0f}};
		AGMSensorData sensorDataCal;
		sensorDataCal.Acc.x = AccCal(0,0);
		sensorDataCal.Acc.y = AccCal(1,0);
		sensorDataCal.Acc.z = AccCal(2,0);
		sensorDataCal.Mag.x = MagCal(0,0);
		sensorDataCal.Mag.y = MagCal(1,0);
		sensorDataCal.Mag.z = MagCal(2,0);
		sensorDataCal.Gyro.x = GyroCal(0,0);
		sensorDataCal.Gyro.y = GyroCal(1,0);
		sensorDataCal.Gyro.z = GyroCal(2,0);
		currTime = (float)(HAL_GetTick()) * ms2s;
		sensorDataCal.SensorTime = currTime;
		Fusion.OnIMUData(sensorDataCal);
		const auto& rotMatrix = Fusion.GetRotationMatrix();
		auto acc = rotMatrix * AccCal;
//		float mx = 0;
//		float my = 0;
//		float mz = 0;
//		float md = 1.0f / MagCal.VecNorm();
//		mx = MagCal(0,0) * md;
//		my = MagCal(1,0) * md;
//		mz = MagCal(2,0) * md;
		//printf("Raw:0,0,0,0,0,0,%f,%f,%f\n\r", MagCal(0,0), MagCal(1,0), MagCal(2,0));
		printf("Raw:%f,%f,%f,%f,%f,%f,%f,%f,%f\n\r", AccRaw(0,0), AccRaw(1,0), AccRaw(2,0),
				MagRaw(0,0), MagRaw(1,0), MagRaw(2,0),
				GyroRaw(0,0), GyroRaw(1,0), GyroRaw(2,0));
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
		Kalman.UpdateState(GyroCal, currTime);
		Kalman.CorrectStateAcc(AccCal, currTime);
		Kalman.CorrectStateMag(MagCal, currTime);
		Q = Kalman.GetState();
		//LOG("AHRS: \t%f, \t%f, \t%f, \t\t\t%f, \t%f, \t%f", Kalman.GetRoll(), Kalman.GetPitch(), Kalman.GetYaw(), Fusion.GetRoll(), Fusion.GetPitch(), Fusion.GetYaw());
		Mat::Quaternion newAcceleration = {0, AccCal(0,0), AccCal(1,0), AccCal(2,0)};
//		Q.w = GetW();
//		Q.x = GetX();
//		Q.y = GetY();
//		Q.z = GetZ();
		Q = Kalman.GetState();
		newAcceleration = (Q * newAcceleration * Q.Conjugate());
		//LOG("Acceleration Vector: \t%f, \t%f, \t%f,  \t%f, \t%f, \t%f", acc(0,0), acc(1,0), acc(2,0), newAcceleration.x, newAcceleration.y, newAcceleration.z);

		//Calculate new Velocity and Position
//		_position = _position + _velocity * deltat + ((newAcceleration + _acceleration * 2.0f) * (deltat * deltat / 6.0f));
//		_velocity = _velocity + ((_acceleration + newAcceleration) * (0.5f * deltat));
//		_acceleration = newAcceleration;

		//printf("My: %f, \t%f, \t%f, \tOrig: %f, \t%f, \t%f\n\r", Fusion.GetRoll(), Fusion.GetPitch(), Fusion.GetYaw(), getRoll(), getPitch(), getYaw());
		//auto acc = Fusion.GetAcceleration();
//		printf("Orig: %f, \t%f, \t%f, \tMy: %f, \t%f, \t%f\n\r",
//				_acceleration.x, _acceleration.y, _acceleration.z,
//				acc.x, acc.y, acc.z);

		return 1;
	}
	return 0;
}

uint8_t OnGNSSData(const GNSS_StateHandle* GNSSData)
{
	Fusion.OnGNSSData(GNSSData);
}

Vec3 GetPosition()
{
	Vec3 result;
	const Algorithms::VelocityEKF::StateVec& position = Fusion.GetVelPos();
	result.x = position(3);
	result.y = position(4);
	result.z = position(5);
	return result;
}

Vec3 GetVelocity()
{
	Vec3 result;
	const Algorithms::VelocityEKF::StateVec& velocity = Fusion.GetVelPos();
	result.x = velocity(0);
	result.y = velocity(1);
	result.z = velocity(2);
	return result;
}

Vec3 GetAcceleration()
{
	Vec3 result;
	return result;
}
