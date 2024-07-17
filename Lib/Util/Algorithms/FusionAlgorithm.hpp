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


namespace Algorithms
{

class FusionAlgorithm
{
public:
	FusionAlgorithm();
	void InitState(const AGMSensorData& imuData);
	void UpdateIMU(const AGMSensorData& imuData);
	//void UpdateGNSS(const GNSSSensorData& gnssData);
	void ResetKinematics();

	const Mat::Quaternion& GetAcceleration() const {return _acceleration;}
	const Mat::Quaternion& GetVelocity() const {return _velocity;}
	const Mat::Quaternion& GetPosition() const {return _position;}
	const Mat::Quaternion& GetOrientation() const {return _madgwickFilter.GetOrientation();}
	float GetRoll() const {return _madgwickFilter.GetRoll();}
	float GetPitch() const {return _madgwickFilter.GetPitch();}
	float GetYaw() const {return _madgwickFilter.GetYaw();}

private:
	Mat::Quaternion _acceleration{0.0f, 0.0f, 0.0f, 0.0f};
	Mat::Quaternion _velocity{0.0f, 0.0f, 0.0f, 0.0f};
	Mat::Quaternion _position{0.0f, 0.0f, 0.0f, 0.0f};
	MadgwickFilter _madgwickFilter;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_FUSIONALGORITHM_HPP_ */
