/*
 * Quaternion.cpp
 *
 *  Created on: Apr 13, 2024
 *      Author: pawda
 */

#include "../inc/Quaternion.hpp"

#include <math.h>

namespace Mat
{

Quaternion& Quaternion::operator+=(const Quaternion& other)
{
	w += other.w;
	x += other.x;
	y += other.y;
	z += other.z;
	return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other)
{
	w -= other.w;
	x -= other.x;
	y -= other.y;
	z -= other.z;
	return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& other)
{
	*this = operator*(other);
	return *this;
}

Quaternion& Quaternion::operator*=(float f)
{
	w *= f;
	x *= f;
	y *= f;
	z *= f;
	return *this;
}

Quaternion& Quaternion::Normalise()
{
	float norm = Norm();
	if(norm != 0.0f) this->operator *=((1.0f / norm));
	return *this;
}

Quaternion Quaternion::operator+(const Quaternion& other) const
{
	Quaternion result = *this;
	result += other;
	return result;
}

Quaternion Quaternion::operator-(const Quaternion& other) const
{
	Quaternion result = *this;
	result -= other;
	return result;
}

Quaternion Quaternion::operator*(const Quaternion& other) const
{
	Quaternion result{};
	result.w = this->w*other.w - this->x*other.x - this->y*other.y - this->z*other.z;
	result.x = this->w*other.x + this->x*other.w + this->y*other.z - this->z*other.y;
	result.y = this->w*other.y - this->x*other.z + this->y*other.w + this->z*other.x;
	result.z = this->w*other.z + this->x*other.y - this->y*other.x + this->z*other.w;
	return result;
}

Quaternion Quaternion::operator*(float f) const
{
	Quaternion result = *this;
	result *= f;
	return result;
}

Quaternion Quaternion::Conjugate() const
{
	Quaternion result = *this;
	result.x*= -1.0f;
	result.y*= -1.0f;
	result.z*= -1.0f;
	return result;
}

Quaternion Quaternion::Normalised() const
{
	Quaternion normalised = *this;
	normalised.Normalise();
	return normalised;
}

Quaternion Quaternion::Inversed() const
{
	Quaternion conjugate = Conjugate();
	conjugate.Normalise();
	return conjugate;
}

float Quaternion::Norm() const
{
	return sqrt(w*w + x*x + y*y + z*z);
}

} //namespace Mat
