/*
 * LibAPI.cpp
 *
 *  Created on: Apr 6, 2024
 *      Author: pawda
 */

#include "LibAPI.h"

#include "../Util/Algorithms/MadgwickAHRSclass.h"
#include "../Util/Algorithms/MadgwickOriginal.h"
#include "Quaternion.hpp"
#include "calibrate.h"
#include "logger.h"

#include <stdio.h>

constexpr float IMUFrequency = 200.0f;
constexpr float madgwickBeta = 0.1f;
Madgwick madgwick{IMUFrequency, madgwickBeta};

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

		Acc = {0, AccCal(0,0), AccCal(1,0), AccCal(2,0)};
		Q.w = GetW();
		Q.x = GetX();
		Q.y = GetY();
		Q.z = GetZ();
		Acc = (Q * Acc * Q.Conjugate()) - G;
		printf("%f	%f	%f	%f\n\r", Acc.x, Acc.y, Acc.z, Acc.Norm());
		//printf("MAG: %f, %f, %f\n\r",MagCal(0,0),MagCal(1,0),MagCal(2,0));
		//delta_t = sensorData->SensorTime - prev_time;
		//GyroCalMean += GyroCal * (delta_t / 1000.0f);
		//prev_time = sensorData->SensorTime;
		//GyroRawMean = GyroRawMean * (1.0f - step) + GyroRaw * step;
		//GyroCalMean = GyroCalMean * (1.0f - step) + GyroCal * step;
		//float norm = sqrt(MagCal(0,0) * MagCal(0,0) + MagCal(1,0) * MagCal(1,0) + MagCal(2,0) * MagCal(2,0));
		//printf("Mag: %f,	%f,	%f,	Norm: %f \n\r", MagCal(0,0), MagCal(1,0), MagCal(2,0), norm);
		//printf("gyro: %f,	%f,	%f,	%f,	%f,	%f\n\r", GyroRawMean(0,0), GyroRawMean(1,0), GyroRawMean(2,0), GyroCalMean(0,0), GyroCalMean(1,0), GyroCalMean(2,0));
		//printf("%f %f %f, D_t: %f\n\r", GyroCalMean(0,0), GyroCalMean(1,0), GyroCalMean(2,0), delta_t);
//		madgwick.update(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
//						AccCal(0,0), AccCal(1,0), AccCal(2,0),
//						MagCal(0,0), MagCal(1,0), MagCal(2,0));
		//printf("MAG: %f, %f, %f\n\r",MagCal(0,0),MagCal(1,0),MagCal(2,0));
		filterUpdate(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
						AccCal(0,0), AccCal(1,0), AccCal(2,0),
						MagCal(0,0), MagCal(1,0), MagCal(2,0),
						sensorData->SensorTime / 1000.0f);
		return 1;
	}
	return 0;
}

float MadgwickGetRoll()
{
	return madgwick.getRoll();
}

float MadgwickGetPitch()
{
	return madgwick.getPitch();
}

float MadgwickGetYaw()
{
	return madgwick.getYaw();
}
