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

	void UpdateIMU(const AGMSensorData& imuData);
	void UpdateGNSS(const GNSSSensorData& gnssData);

private:
	Mat::Quaternion _velocity{0.0f, 0.0f, 0.0f, 0.0f};
	Mat::Quaternion _position{0.0f, 0.0f, 0.0f, 0.0f};
	Mat::Quaternion _pose{1.0f, 0.0f, 0.0f, 0.0f};
	MadgwickFilter _madgwickFilter;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_FUSIONALGORITHM_HPP_ */
