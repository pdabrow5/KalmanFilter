/*
 * MyMadgwick.hpp
 *
 *  Created on: Apr 14, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_MYMADGWICK_HPP_
#define UTIL_ALGORITHMS_MYMADGWICK_HPP_

#include "UtilTypes.h"
#include "Quaternion.hpp"
#include "Matrix.hpp"

namespace Algorithms
{

class MadgwickFilter
{
public:
	explicit MadgwickFilter(float beta);
	void Update(AGMSensorData_t* sensorData);
	float GetRoll() const;
	float GetPitch() const;
	float GetYaw() const;
private:
	Mat::Quaternion _q;
	float _beta;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_MYMADGWICK_HPP_ */
