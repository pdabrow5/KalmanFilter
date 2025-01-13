/*
 * ExtendedKalman.hpp
 *
 *  Created on: Jun 25, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_EXTENDEDKALMAN_AHRSKALMAN_HPP_
#define UTIL_ALGORITHMS_EXTENDEDKALMAN_AHRSKALMAN_HPP_

#include "Matrix.hpp"
#include "Quaternion.hpp"

using namespace Mat;

namespace Algorithms
{

class AHRSKalman
{
public:
	AHRSKalman(){};
	const Quaternion& GetState() const;
	const Matrix<4, 4>& GetNoiseCovariance() const;
	void InitialiseKalman(const Matrix<3, 1>& acc, const Matrix<3, 1>& mag, float time);
	void CorrectStateAcc(const Matrix<3, 1>& acc, float time);
	void CorrectStateMag(const Matrix<3, 1>& mag, float time);
	void UpdateState(const Matrix<3, 1>& U, float time);
	float GetRoll() const;
	float GetPitch() const;
	float GetYaw() const;
private:
	//Estimated State
	Quaternion _X;

	//Estimated Covariance
	Matrix<4,4> _P = Eye<4>();

//Helper variables
	float _lastUpdateTime;
	float _lastCorrectionTime;

	const float _gyroNoiseVariance{0.05};
	const float _magNoiseVariance{0.4};
	const float _accNoiseVariance{0.01};

	//Process Noise Covariancex
	Matrix<4, 4> _Q;

	//Predicted State
	Quaternion _pX;
};

} //namespace Algorithms
#endif /* UTIL_ALGORITHMS_EXTENDEDKALMAN_AHRSKALMAN_HPP_ */
