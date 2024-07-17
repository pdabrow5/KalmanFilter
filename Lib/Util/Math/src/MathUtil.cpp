/*
 * MathUtil.cpp
 *
 *  Created on: Jul 2, 2024
 *      Author: pawda
 */

#include "MathUtil.hpp"

#include <cmath>

namespace Mat
{

float RollFromQuaterion(const Quaternion& q)
{
	float roll = atan2(2.0f * (q.w*q.x + q.y*q.z), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
	return roll * 57.29578f;
}

float PitchFromQuaterion(const Quaternion& q)
{
	float pitch = -asin(2.0f * (q.x*q.z - q.w*q.y));
	return pitch * 57.29578f;
}

float YawFromQuaterion(const Quaternion& q)
{
	float yaw = atan2(2.0f * (q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
	return yaw * 57.29578f + 180.0f;
}

} //namespace Mat

