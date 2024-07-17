#ifndef UTIL_INC_VECTOR_HPP_
#define UTIL_INC_VECTOR_HPP_

#include <algorithm>
#include <array>
#include <cmath>

#include "Matrix.hpp"

namespace Mat
{

template<u_short length>
class V_Vector;

template<u_short length>
class H_Vector: public Matrix<1, length>
{
public:
	using Matrix<1, length>::_values;
	H_Vector();
	H_Vector(float val);
	H_Vector(const std::array<float, length>& values);

	inline float& operator()(u_short index) {return _values[index];}
	inline float operator()(u_short index) const {return _values[index];}
	inline float operator*(const V_Vector<length>& other) const;

	inline V_Vector<length> Transpose() const;

	float Norm() const;
};

template<u_short length>
class V_Vector: public Matrix<length, 1>
{
public:
	using Matrix<length, 1>::_values;
	V_Vector();
	V_Vector(float val);
	V_Vector(const std::array<float, length>& values);

	inline float& operator()(u_short index) {return _values[index];}
	inline float operator()(u_short index) const {return _values[index];}

	inline H_Vector<length> Transpose() const;

	float Norm() const;
};

template <u_short length>
H_Vector<length>::H_Vector() : Matrix<1, length>() {}

template <u_short length>
H_Vector<length>::H_Vector(float val) : Matrix<1, length>(val) {}

template <u_short length>
H_Vector<length>::H_Vector(const std::array<float, length>& values) : Matrix<1, length>(values) {}

template<u_short length>
float H_Vector<length>::operator*(const V_Vector<length>& other) const
{
	float result = 0.0f;
	for(u_short i = 0; i < length; ++i){result += _values[i] * other(i);}
	return result;
}

template<u_short length>
V_Vector<length> H_Vector<length>::Transpose() const
{
	return V_Vector<length>(_values);
}

template <u_short length>
float H_Vector<length>::Norm() const
{
	float result = 0.0f;
	std::for_each(_values.begin(), _values.end(), [&result](float n){result += n * n;});
	return std::sqrt(result);
}

template <u_short length>
V_Vector<length>::V_Vector() : Matrix<length, 1>() {}

template <u_short length>
V_Vector<length>::V_Vector(float val) : Matrix<length, 1>(val) {}

template <u_short length>
V_Vector<length>::V_Vector(const std::array<float, length>& values) : Matrix<length, 1>(values) {}

template<u_short length>
H_Vector<length> V_Vector<length>::Transpose() const
{
	return H_Vector<length>(_values);
}

template <u_short length>
float V_Vector<length>::Norm() const
{
	float result = 0.0f;
	std::for_each(_values.begin(), _values.end(), [&result](float n){result += n * n;});
	return std::sqrt(result);
}

} //namespace Mat

#endif /* UTIL_INC_VECTOR_HPP_ */
