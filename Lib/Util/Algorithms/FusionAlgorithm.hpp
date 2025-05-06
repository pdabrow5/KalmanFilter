/*
 * FusionAlgorithm.hpp
 *
 *  Created on: Apr 22, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_FUSIONALGORITHM_HPP_
#define UTIL_ALGORITHMS_FUSIONALGORITHM_HPP_

#include "MyMadgwick.hpp"
#include "UtilTypes.h"
#include "ExtendedKalman/VelocityEKF.hpp"
#include "ExtendedKalman/OrientationEKF.hpp"
#include "../../../Peripherals/simpleRTK2B/GNSS.h"
#include "ExtendedKalman/AHRSKalman.hpp"


namespace Algorithms
{

constexpr float _gyroNoiseVariance = 0.05;
constexpr float _magNoiseVariance = 2.4;
constexpr float _accNoiseVariance = 0.10;

class FusionAlgorithm
{
public:
//	FusionAlgorithm(
//			const OrientationEKF::StateVec& oriState, const OrientationEKF::StateCovarianceMatrix& oriCov,
//			const VelocityEKF::StateVec& velState, const VelocityEKF::StateCovarianceMatrix& velCov,
//			float time);
	FusionAlgorithm(){
		_orientationControlCov = Eye<3>(_gyroNoiseVariance);
//		_orientationMeassurementCov(0,0) = _accNoiseVariance;
//		_orientationMeassurementCov(1,1) = _accNoiseVariance;
//		_orientationMeassurementCov(2,2) = _accNoiseVariance;
//		_orientationMeassurementCov(3,3) = _magNoiseVariance;
//		_orientationMeassurementCov(4,4) = _magNoiseVariance;
//		_orientationMeassurementCov(5,5) = _magNoiseVariance;
	};
	void InitState(const Matrix<3, 1>& acc, const Matrix<3, 1>& mag, float time);
	//void OnIMUData(const AGMSensorData& imuData);
	void OnIMUData(const Matrix<3, 1>& acc, const Matrix<3, 1>& gyro, const Matrix<3, 1>& mag, float time);
	void OnGNSSData(const GNSS_StateHandle* GNSSData);
	float GetRoll() const;
	float GetPitch() const;
	float GetYaw() const;
	const VelocityEKF::StateVec& GetVelPos() const {return _velocityEKF.GetState();}
private:
//	OrientationEKF _orientationEKF{OrientationEKF::StateVec{}, OrientationEKF::StateCovarianceMatrix{}, 0.0f};
	VelocityEKF _velocityEKF{VelocityEKF::StateVec{0.0f}, VelocityEKF::StateCovarianceMatrix{Eye<6>(1.0f)}, 0.0f};
//	OrientationEKF::ControlVec _orientationControlVec;
	OrientationEKF::ControlCovarianceMatrix _orientationControlCov;
//	OrientationEKF::MeassurementVec _orientationMeassurementVec;
//	OrientationEKF::MeasurementCovarianceMatrix _orientationMeassurementCov;
	VelocityEKF::ControlVec _velocityControlVec;
	VelocityEKF::ControlCovarianceMatrix _velocityControlCov;
	VelocityEKF::MeassurementVec _velocityMeassurementVec;
	VelocityEKF::MeasurementCovarianceMatrix _velocityMeassurementCov;

	AHRSKalman _AHRSKalman{};

//Helper methods
	const Matrix<3,3>& _GetGlobalAccCov(float x, float y, float z) const;
public:
	const Matrix<3,3>& GetRotationMatrix() const;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_FUSIONALGORITHM_HPP_ */
