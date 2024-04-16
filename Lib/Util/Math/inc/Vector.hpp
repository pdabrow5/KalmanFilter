#ifndef UTIL_INC_VECTOR_HPP_
#define UTIL_INC_VECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>

namespace Mat
{

using u_int = unsigned short;

template <u_int length>
class Vector
{
public:
	Vector();
	explicit Vector(float val);
	explicit Vector(const std::array<float, length>& values);

	inline float& operator()(u_int index);
	inline const float& operator()(u_int index) const;

	inline bool operator== (const Vector<length>& other) const;
	inline bool operator!= (const Vector<length>& other) const;
	inline bool Equals(const Vector<length>& other, float epsilon = 0.000001f) const;

	inline Vector<length>& operator+= (const Vector<length>& other);
	inline Vector<length>& operator+= (float val);

	inline Vector<length>& operator-= (const Vector<length>& other);
	inline Vector<length>& operator-= (float val);

	inline Vector<length> operator+ (const Vector<length>& other) const;
	inline Vector<length> operator+ (float val) const;

	inline Vector<length> operator- (const Vector<length>& other) const;
	inline Vector<length> operator- (float val) const;

	inline Vector<length>& operator*= (float val);

	Vector<length> operator*(float val) const;
	inline float operator*(const Vector<length>& other) const;

	inline u_int Size() const {return length;}

	inline float Norm() const {float result = 0.0f; std::for_each(_values.begin(), _values.end(), [&result](float n){result += n * n;}); return std::sqrt(result);}

protected:
	std::array<float, length> _values;
};

//IMPLEMENTATION

template <u_int length>
Vector<length>::Vector() {static_assert(length > 0, "Vector length must be positive!");}

template <u_int length>
Vector<length>::Vector(float val)
{
	static_assert(length > 0, "Vector length must be positive!");
	for(auto &v : _values) v = val;
}

template <u_int length>
Vector<length>::Vector(const std::array<float, length>& values)
{
	static_assert(length > 0, "Vector length must be positive!");
	 _values = values;
}

template <u_int length>
float& Vector<length>::operator()(u_int index)
{
	return _values[index];
}

template <u_int length>
const float& Vector<length>::operator()(u_int index) const
{
	return _values[index];
}

template <u_int length>
bool Vector<length>::operator==(const Vector<length>& other) const
{
	return _values == other._values;
}

template <u_int length>
bool Vector<length>::operator!=(const Vector<length>& other) const
{
	return _values != other._values;
}

template <u_int length>
bool Vector<length>::Equals(const Vector<length>& other, float epsilon) const
{
	bool result = true;
	for(u_int i = 0; i < length && result; ++i)
	{
		result = result && (fabsf(_values[i] - other._values[i]) < epsilon);
	}
	return result;
}

template <u_int length>
Vector<length>& Vector<length>::operator+=(const Vector<length>& other)
{
	for(u_int i = 0; i < _values.size(); ++i) _values[i] += other._values[i];
	return *this;
}

template <u_int length>
Vector<length>& Vector<length>::operator-=(const Vector<length>& other)
{
	for(u_int i = 0; i < _values.size(); ++i) _values[i] -= other._values[i];
	return *this;
}

template <u_int length>
Vector<length>& Vector<length>::operator+=(float val)
{
	for(u_int i = 0; i < _values.size(); ++i) _values[i] += val;
	return *this;
}

template <u_int length>
Vector<length>& Vector<length>::operator-=(float val)
{
	for(u_int i = 0; i < _values.size(); ++i) _values[i] -= val;
	return *this;
}

template <u_int length>
Vector<length> Vector<length>::operator+(const Vector<length>& other) const
{
	Vector<length> result = *this;
	result += other;
	return result;
}

template <u_int length>
Vector<length> Vector<length>::operator-(const Vector<length>& other) const
{
	Vector<length> result = *this;
	result -= other;
	return result;
}

template <u_int length>
Vector<length> Vector<length>::operator+(float val) const
{
	Vector<length> result = *this;
	result += val;
	return result;
}

template <u_int length>
Vector<length> Vector<length>::operator-(float val) const
{
	Vector<length> result = *this;
	result -= val;
	return result;
}

template <u_int length>
Vector<length>& Vector<length>::operator*=(float val)
{
	for(u_int i = 0; i < _values.size(); ++i) _values[i] *= val;
	return *this;
}

template <u_int length>
Vector<length> Vector<length>::operator*(float val) const
{
	Vector<length> result = *this;
	result *= val;
	return result;
}

template <u_int length>
float Vector<length>::operator*(const Vector<length>& other) const
{
	float result = 0.0f;
	for(u_int i = 0; i < _values.size(); ++i) result += _values[i] * other._values[i];
	return result;
}

} //namespace Mat

#endif /* UTIL_INC_VECTOR_HPP_ */
