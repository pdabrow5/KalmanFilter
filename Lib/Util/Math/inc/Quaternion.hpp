/*
 * Quaternion.hpp
 *
 *  Created on: Apr 13, 2024
 *      Author: pawda
 */

#ifndef UTIL_MATH_INC_QUATERNION_HPP_
#define UTIL_MATH_INC_QUATERNION_HPP_

namespace Mat
{

struct Quaternion
{
public:
	float w, x, y, z;
	Quaternion& operator+=(const Quaternion& other);
	Quaternion& operator-=(const Quaternion& other);
	Quaternion& operator*=(const Quaternion& other);
	Quaternion& operator*=(float f);

	Quaternion& Normalise();

	Quaternion operator+(const Quaternion& other) const;
	Quaternion operator-(const Quaternion& other) const;
	Quaternion operator*(const Quaternion& other) const;
	Quaternion operator*(float f) const;

	Quaternion Conjugate() const;
	Quaternion Normalised() const;
	Quaternion Inversed() const;
	float Norm() const;
};

inline Quaternion operator*(float f, const Quaternion& q)
{
	return q * f;
}

} //namespace Mat
#endif /* UTIL_MATH_INC_QUATERNION_HPP_ */
