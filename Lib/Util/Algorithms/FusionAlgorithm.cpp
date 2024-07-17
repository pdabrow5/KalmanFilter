/*
 * FusionAlgorithm.cpp
 *
 *  Created on: Apr 24, 2024
 *      Author: pawda
 */

#include "FusionAlgorithm.hpp"
#include "Vector.hpp"
#include "calibrate.h"

const float gyroMeasError = 3.14159265358979 * (2.0f / 180.0f); // gyroscope measurement error in rad/s (shown as 5 deg/s)
const float beta = sqrt(3.0f / 4.0f) * gyroMeasError; // compute beta
const float G = 9.80655f;
const float Mili2Sec = 1.0f / 1000.0f;

namespace Algorithms
{

using namespace Mat;

FusionAlgorithm::FusionAlgorithm(): _madgwickFilter{beta}{}

void FusionAlgorithm::InitState(const AGMSensorData& imuData)
{
	V_Vector<3> rawAcc{{imuData.Acc.x, imuData.Acc.y, imuData.Acc.z}};
	V_Vector<3> rawGyro{{imuData.Gyro.x, imuData.Gyro.y, imuData.Gyro.z}};
	V_Vector<3> rawMag{{imuData.Mag.x, imuData.Mag.y, imuData.Mag.z}};
	auto AccCal = CalibrateAcc(rawAcc);
	auto GyroCal = CalibrateGyro(rawGyro);
	auto MagCal = CalibrateMag(rawMag);
	_madgwickFilter.Update(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
							AccCal(0,0), AccCal(1,0), AccCal(2,0),
							MagCal(0,0), MagCal(1,0), MagCal(2,0),
							imuData.SensorTime * Mili2Sec);
	_acceleration = {0.0f, 0.0f, 0.0f, 0.0f};
	_velocity = {0.0f, 0.0f, 0.0f, 0.0f};
	_position = {0.0f, 0.0f, 0.0f, 0.0f};
}

void FusionAlgorithm::UpdateIMU(const AGMSensorData& imuData)
{
	static float last_time{imuData.SensorTime * Mili2Sec};
	float currTime = imuData.SensorTime * Mili2Sec;
	float deltat = (currTime - last_time);
	last_time = currTime;
	V_Vector<3> rawAcc{{imuData.Acc.x, imuData.Acc.y, imuData.Acc.z}};
	V_Vector<3> rawGyro{{imuData.Gyro.x, imuData.Gyro.y, imuData.Gyro.z}};
	V_Vector<3> rawMag{{imuData.Mag.x, imuData.Mag.y, imuData.Mag.z}};
	auto AccCal = CalibrateAcc(rawAcc);
	auto GyroCal = CalibrateGyro(rawGyro);
	auto MagCal = CalibrateMag(rawMag);

	//Update Orientation
	_madgwickFilter.Update(GyroCal(0,0), GyroCal(1,0), GyroCal(2,0),
							AccCal(0,0), AccCal(1,0), AccCal(2,0),
							MagCal(0,0), MagCal(1,0), MagCal(2,0),
							currTime);

	//Calcuate new Acceleration
	Quaternion newAcceleration = {0.0f, AccCal(0,0), AccCal(1,0), AccCal(2,0)};
	newAcceleration = (_madgwickFilter.GetOrientation() * newAcceleration * _madgwickFilter.GetOrientation().Inversed());
	newAcceleration.z -= G;

	//Calculate new Velocity and Position
	_position = _position + _velocity * deltat + ((newAcceleration + _acceleration * 2.0f) * (deltat * deltat / 6.0f));
	_velocity = _velocity + ((_acceleration + newAcceleration) * (0.5f * deltat));
	_acceleration = newAcceleration;

}

void FusionAlgorithm::ResetKinematics()
{
	_acceleration = {0.0f, 0.0f, 0.0f, 0.0f};
	_velocity = {0.0f, 0.0f, 0.0f, 0.0f};
	_position = {0.0f, 0.0f, 0.0f, 0.0f};
}

} //namespace Algorithms
