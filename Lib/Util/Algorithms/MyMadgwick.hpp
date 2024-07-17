/*
 * MyMadgwick.hpp
 *
 *  Created on: Apr 14, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_MYMADGWICK_HPP_
#define UTIL_ALGORITHMS_MYMADGWICK_HPP_

#include "Quaternion.hpp"
#include "Matrix.hpp"

namespace Algorithms
{

class MadgwickFilter
{
public:
	explicit MadgwickFilter(float beta);
	void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float time);
	void SetBeta(float);

	const Mat::Quaternion& GetOrientation() const;
	float GetRoll() const;
	float GetPitch() const;
	float GetYaw() const;
private:
	Mat::Quaternion _q;
	float _beta;
};

} //namespace Algorithms

#endif /* UTIL_ALGORITHMS_MYMADGWICK_HPP_ */
