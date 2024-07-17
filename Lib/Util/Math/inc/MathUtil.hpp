/*
 * Util.hpp
 *
 *  Created on: Jul 2, 2024
 *      Author: pawda
 */

#ifndef UTIL_MATH_INC_MATHUTIL_HPP_
#define UTIL_MATH_INC_MATHUTIL_HPP_

#include "Matrix.hpp"
#include "Quaternion.hpp"

namespace Mat
{

float RollFromQuaterion(const Quaternion& q);
float PitchFromQuaterion(const Quaternion& q);
float YawFromQuaterion(const Quaternion& q);

}
#endif /* UTIL_MATH_INC_MATHUTIL_HPP_ */
