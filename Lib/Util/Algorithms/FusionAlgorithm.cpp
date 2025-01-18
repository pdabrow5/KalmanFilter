/*
 * FusionAlgorithm.cpp
 *
 *  Created on: Apr 24, 2024
 *      Author: pawda
 */

#include "FusionAlgorithm.hpp"
#include "constants.h"
#include "logger.h"

namespace Algorithms
{
using namespace Mat;

//FusionAlgorithm::FusionAlgorithm(
//		const OrientationEKF::StateVec& oriState, const OrientationEKF::StateCovarianceMatrix& oriCov,
//		const VelocityEKF::StateVec& velState, const VelocityEKF::StateCovarianceMatrix& velCov,
//		float time) : _orientationEKF{oriState, oriCov, time}, _velocityEKF{velState, velCov, time}{}

void FusionAlgorithm::InitState(const Matrix<3, 1>& acc, const Matrix<3, 1>& mag, float time)
{
	LOG("INIT");
//	_orientationEKF.Initialise(acc, mag, time);
//	const auto& _state = _orientationEKF.GetState();
//	LOG("State: %f, %f, %f, %f", _state(0), _state(1), _state(2), _state(3));
	_AHRSKalman.InitialiseKalman(acc, mag, time);
}

//void FusionAlgorithm::OnIMUData(const AGMSensorData& imuData)
void FusionAlgorithm::OnIMUData(const Matrix<3, 1>& acc, const Matrix<3, 1>& gyro, const Matrix<3, 1>& mag, float time)
{
//	_orientationControlVec(0) = imuData.Gyro.x; _orientationControlVec(1) = imuData.Gyro.y; _orientationControlVec(2) = imuData.Gyro.z;
//	_orientationEKF.Predict(_orientationControlVec, _orientationControlCov, imuData.SensorTime);
//	float a_factor = 1.0f / (sqrt(imuData.Acc.x*imuData.Acc.x + imuData.Acc.y*imuData.Acc.y + imuData.Acc.z*imuData.Acc.z));
//	float m_factor = 1.0f / (sqrt(imuData.Mag.x*imuData.Mag.x + imuData.Mag.y*imuData.Mag.y + imuData.Mag.z*imuData.Mag.z));
//	_orientationMeassurementVec(0) = imuData.Acc.x * a_factor;
//	_orientationMeassurementVec(1) = imuData.Acc.y * a_factor;
//	_orientationMeassurementVec(2) = imuData.Acc.z * a_factor;
//	_orientationMeassurementVec(3) = imuData.Mag.x * m_factor;
//	_orientationMeassurementVec(4) = imuData.Mag.y * m_factor;
//	_orientationMeassurementVec(5) = imuData.Mag.z * m_factor;
//	_orientationEKF.Update(_orientationMeassurementVec, _orientationMeassurementCov, imuData.SensorTime);
	//const OrientationEKF::StateVec& q = _orientationEKF.GetState();
	//LOG("EKF: \t%f, \t%f, \t%f", GetRoll(), GetPitch(), GetYaw());
	//LOG("ORIENTATION: \t%f, \t%f, \t%f, \t%f", q(0), q(1), q(2), q(3));
//AHRS
	_AHRSKalman.UpdateState(gyro, time);
	_AHRSKalman.CorrectStateAcc(acc, time);
	_AHRSKalman.CorrectStateMag(mag, time);
	const auto& Q = _AHRSKalman.GetState();
	Mat::Quaternion newAcceleration = {0, acc(0,0), acc(1,0), acc(2,0)};
	newAcceleration = (Q * newAcceleration * Q.Conjugate());
	V_Vector<3> accENU{{newAcceleration.x, newAcceleration.y, newAcceleration.z}};
	accENU(2) -= Gravity;
	_velocityEKF.Predict(static_cast<const V_Vector<3>&>(accENU), _GetGlobalAccCov(acc(0,0), acc(1,0), acc(2,0)), time);
	const auto& vel = _velocityEKF.GetState();
	//LOG("ENU: \t%f, \t%f, \t%f VEL: \t%f, \t%f, \t%f", accENU(0), accENU(1), accENU(2), vel(0), vel(1), vel(2));
//AHRS
	//V_Vector<3> accL{{imuData.Acc.x, imuData.Acc.y, imuData.Acc.z}};
	//auto accENU = GetRotationMatrix()*accL;
	//accENU(2,0) -= Gravity;
	//_velocityEKF.Predict(static_cast<const V_Vector<3>&>(accENU), _GetGlobalAccCov(accL(0), accL(1), accL(2)), imuData.SensorTime);
}

void FusionAlgorithm::OnGNSSData(const GNSS_StateHandle* GNSSData)
{
	static VelocityEKF::MeassurementVec measurementVec;
	static VelocityEKF::MeasurementCovarianceMatrix measurementCov = Eye<6>(1.0f);
	measurementVec(0) = ((float)GNSSData->velE) * mm2m;
	measurementVec(1) = ((float)GNSSData->velN) * mm2m;
	measurementVec(2) = -((float)GNSSData->velD) * mm2m;
	measurementVec(3) = GNSSData->fLat;
	measurementVec(4) = GNSSData->fLon;
	measurementVec(5) = ((float)GNSSData->hMSL) * mm2m;
	//LOG("measurementVec: \t%f, \t%f, \t%f, \t%f, \t%f, \t%f", measurementVec(0), measurementVec(1), measurementVec(2), measurementVec(3), measurementVec(4), measurementVec(5));

	measurementCov(0,0) = ((float)GNSSData->sAcc) * mm2m;
	measurementCov(1,1) = ((float)GNSSData->sAcc) * mm2m;
	measurementCov(2,2) = ((float)GNSSData->sAcc) * mm2m;
	measurementCov(3,3) = ((float)GNSSData->hAcc) * mm2m * _ER;
	measurementCov(4,4) = ((float)GNSSData->hAcc) * mm2m * _ER / cos(GNSSData->fLat * DEG_2_RAD);
	measurementCov(5,5) = ((float)GNSSData->vAcc) * mm2m;
	_velocityEKF.Update(measurementVec, measurementCov, _velocityEKF.GetTime());
}


float FusionAlgorithm::GetRoll() const
{
	//const OrientationEKF::StateVec& q = _orientationEKF.GetState();
//AHRS
	const auto& Q = _AHRSKalman.GetState();
	const OrientationEKF::StateVec q{{Q.w, Q.x, Q.y, Q.z}};
//AHRS
	float roll = atan2(2.0f * (q(0) * q(1) + q(2) * q(3)), q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3));
	return roll * 57.29578f;
}

float FusionAlgorithm::GetPitch() const
{
	//const OrientationEKF::StateVec& q = _orientationEKF.GetState();
//AHRS
	const auto& Q = _AHRSKalman.GetState();
	const OrientationEKF::StateVec q{{Q.w, Q.x, Q.y, Q.z}};
//AHRS
	float pitch = -asin(2.0f * (q(1) * q(3) - q(0) * q(2)));
	return pitch * 57.29578f;
}

float FusionAlgorithm::GetYaw() const
{
	//const OrientationEKF::StateVec& q = _orientationEKF.GetState();
//AHRS
	const auto& Q = _AHRSKalman.GetState();
	const OrientationEKF::StateVec q{{Q.w, Q.x, Q.y, Q.z}};
//AHRS
	float yaw = atan2(2.0f * (q(1) * q(2) + q(0) * q(3)), q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3));
	return yaw * 57.29578f + 180.0f;
}

const Matrix<3,3>& FusionAlgorithm::GetRotationMatrix() const
{
	static Matrix<3,3> rotationMatrix;
	//const OrientationEKF::StateVec& q = _orientationEKF.GetState();
//AHRS
	const auto& Q = _AHRSKalman.GetState();
	const OrientationEKF::StateVec q{{Q.w, Q.x, Q.y, Q.z}};
//AHRS
	rotationMatrix = {{
			q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 	2.0f*(q(1)*q(2)-q(0)*q(3)), 	2.0f*(q(1)*q(3)+q(0)*q(2)),
			2.0f*(q(1)*q(2)+q(0)*q(3)), 	q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3),	2.0f*(q(2)*q(3)-q(0)*q(1)),
			2.0f*(q(1)*q(3)-q(0)*q(2)),		2.0f*(q(0)*q(1)+q(2)*q(3)), 	q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3)
	}};
	return rotationMatrix;
}

const Matrix<3,3>& FusionAlgorithm::_GetGlobalAccCov(float x, float y, float z) const
{
	static Matrix<3,3> result;
	static const Matrix<3,3>& a_a = GetRotationMatrix();
	//const OrientationEKF::StateVec& q = _orientationEKF.GetState();
//AHRS
	const auto& Q = _AHRSKalman.GetState();
	const OrientationEKF::StateVec q{{Q.w, Q.x, Q.y, Q.z}};
//AHRS
	Matrix<3, 4> a_q{{
        -y*q(3) + z*q(2) , y*q(2) + z*q(3) , -2.0f*x*q(2) + y*q(1) + z*q(0) , -2.0f*x*q(3) - y*q(0) + z*q(1),
        x*q(3) - z*q(1) , x*q(2) -2.0f*y*q(1) - z*q(0) , x*q(1) + z*q(3) , x*q(0) - 2.0f*y*q(3) + z*q(2),
        -x*q(2) + y*q(1) , x*q(3) + y*q(0) -2.0f*z*q(1) , -x*q(0) + y*q(3) - 2.0f*z*q(2) , x*q(1) + y*q(2)
	}};
	result = a_a*_orientationControlCov*a_a.Transposed() + a_q*_AHRSKalman.GetNoiseCovariance()*a_q.Transposed();
	return result;
}

} //namespace Algorithms
