/*
 * LibAPI.cpp
 *
 *  Created on: Apr 6, 2024
 *      Author: pawda
 */

#include "LibAPI.h"

#include "../Util/Algorithms/MadgwickOriginal.h"
#include "../Util/Algorithms/MyMadgwick.hpp"
#include "Quaternion.hpp"
#include "calibrate.h"
#include "logger.h"

#include <stdio.h>

#define gyroMeasError 3.14159265358979 * (10.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

Algorithms::MadgwickFilter madgwickFilter{beta};

Mat::Matrix<3, 1> MagCal({0, 0, 0});
Mat::Matrix<3, 1> MagRaw({0, 0, 0});

Mat::Matrix<3, 1> GyroCal({0, 0, 0});
Mat::Matrix<3, 1> GyroRaw({0, 0, 0});

Mat::Matrix<3, 1> AccCal({0, 0, 0});
Mat::Matrix<3, 1> AccRaw({0, 0, 0});

Mat::Matrix<3, 1> GyroRawMean({0, 0, 0});
Mat::Matrix<3, 1> GyroCalMean({0, 0, 0});

Mat::Quaternion G{0,0,0,9.80655f};
Mat::Quaternion Q{};
Mat::Quaternion Acc{};

uint8_t MadgwickInit()
{
	return 1;
}

uint8_t MadgwickUpdate(const AGMSensorData* sensorData)
{
	static float prev_time = sensorData->SensorTime;
	static float delta_t;
	float step = 0.0005;
	if(sensorData != nullptr)
	{
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

//		Acc = {0, AccCal(0,0), AccCal(1,0), AccCal(2,0)};
//		Q.w = GetW();
//		Q.x = GetX();
//		Q.y = GetY();
//		Q.z = GetZ();
//		Acc = (Q * Acc * Q.Conjugate()) - G;
//		printf("%f	%f	%f	%f\n\r", Acc.x, Acc.y, Acc.z, Acc.Norm());

		filterUpdate(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
						AccCal(0,0), AccCal(1,0), AccCal(2,0),
						MagCal(0,0), MagCal(1,0), MagCal(2,0),
						sensorData->SensorTime / 1000.0f);
		madgwickFilter.Update(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
								AccCal(0,0), AccCal(1,0), AccCal(2,0),
								MagCal(0,0), MagCal(1,0), MagCal(2,0),
								sensorData->SensorTime / 1000.0f);
		printf("Orig: %f,\t%f,\t%f\t\tMy: %f,\t%f,\t%f\n\r", getRoll(), getPitch(), getYaw(),
				madgwickFilter.GetRoll(), madgwickFilter.GetPitch(), madgwickFilter.GetYaw());
//		Quaternion MyQ = madgwickFilter.GetOrientation();
//		printf("Orig: %f,\t%f,\t%f,\t%f\tMy: %f,\t%f,\t%f,\t%f\n\r", GetW(), GetX(), GetY(), GetZ(),
//																	MyQ.w, MyQ.x, MyQ.y, MyQ.z);
		return 1;
	}
	return 0;
}
