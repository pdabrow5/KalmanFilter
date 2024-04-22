#ifndef UTIL_INC_MATRIX_HPP_
#define UTIL_INC_MATRIX_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

namespace Mat
{

using u_short = unsigned char;
using u_int = unsigned int;

/// @brief Pair contatining Matrix dimensions: Heigth and Width
using MatShape = std::pair<u_short, u_short>;

template <u_short heigth, u_short width>
class Matrix
{
public:
	using Row = std::array<float, width>;
	Matrix();
	Matrix(float val);
	Matrix(const std::array<float, heigth*width>& values);

	inline float& operator()(u_short row, u_short col);
	inline const float& operator()(u_short row, u_short col) const;

	inline bool operator== (const Matrix<heigth, width>& other) const;
	inline bool operator!= (const Matrix<heigth, width>& other) const;
	inline bool Equals(const Matrix<heigth, width>& other, float epsilon = 0.000001f) const;

	inline Matrix<heigth, width>& operator+= (const Matrix<heigth, width>& other);
	inline Matrix<heigth, width>& operator+= (float val);

	inline Matrix<heigth, width>& operator-= (const Matrix<heigth, width>& other);
	inline Matrix<heigth, width>& operator-= (float val);

	inline Matrix<heigth, width> operator+ (const Matrix<heigth, width>& other) const;
	inline Matrix<heigth, width> operator+ (float val) const;

	inline Matrix<heigth, width> operator- (const Matrix<heigth, width>& other) const;
	inline Matrix<heigth, width> operator- (float val) const;

	inline Matrix<heigth, width>& operator*= (float val);
	inline Matrix<heigth, width> operator* (float val) const;

	template <u_short other_width>
	Matrix<heigth, other_width> operator*(const Matrix<width, other_width>& other) const;

	template <u_short other_width>
	void Multiply(const Matrix<width, other_width>& other, Matrix<heigth, other_width>& result) const;

	inline u_int Size() {return heigth * width;}
	inline const MatShape& Shape() const;
	static constexpr MatShape shape{heigth, width};
private:
	std::array<float, heigth*width> _values;
};

template<u_int length>
float Multiply_rows(const std::array<float, length>& A, const std::array<float, length>& B)
{
    float result = 0.0f;
	for(u_int i = 0; i < length; ++i) result += A[i]*B[i];
	return result;
}

template <u_short heigth, u_short width>
Matrix<heigth, width>::Matrix() {static_assert(heigth > 0 && width > 0, "Matrix width and heigth must be positive!");}

template <u_short heigth, u_short width>
Matrix<heigth, width>::Matrix(float val)
{
	static_assert(heigth > 0 && width > 0, "Matrix width and heigth must be positive!");
	std::for_each(_values.begin(), _values.end(), [val](float& x){x = val;});
}

template <u_short heigth, u_short width>
Matrix<heigth, width>::Matrix(const std::array<float, heigth*width>& values): _values(values) {static_assert(heigth > 0 && width > 0, "Matrix width and heigth must be positive!");}

template <u_short heigth, u_short width>
float& Matrix<heigth, width>::operator()(u_short row, u_short col)
{
	u_int index = width * row + col;
	return _values[index];
}

template <u_short heigth, u_short width>
const float& Matrix<heigth, width>::operator()(u_short row, u_short col) const
{
	u_int index = width * row + col;
	return _values[index];
}

template <u_short heigth, u_short width>
inline bool Matrix<heigth, width>::operator==(const Matrix<heigth, width> &other) const
{
    return _values==other._values;
}

template <u_short heigth, u_short width>
inline bool Matrix<heigth, width>::operator!=(const Matrix<heigth, width> &other) const
{
    return _values!=other._values;
}

template <u_short heigth, u_short width>
inline bool Matrix<heigth, width>::Equals(const Matrix<heigth, width> &other, float epsilon) const
{
    bool result = true;
	for(u_int i = 0; i < _values.size() && result; ++i)
	{
		result = result && (fabsf(_values[i] - other._values[i]) < epsilon);
	}
	return result;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> &Matrix<heigth, width>::operator+=(const Matrix<heigth, width> &other)
{
    for(u_int i = 0; i < _values.size(); ++i) _values[i] += other._values[i];
	return *this;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> &Matrix<heigth, width>::operator+=(float val)
{
    for(u_int i = 0; i < _values.size(); ++i) _values[i] += val;
	return *this;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> &Matrix<heigth, width>::operator-=(const Matrix<heigth, width> &other)
{
    for(u_int i = 0; i < _values.size(); ++i) _values[i] -= other._values[i];
	return *this;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> &Matrix<heigth, width>::operator-=(float val)
{
    for(u_int i = 0; i < _values.size(); ++i) _values[i] -= val;
	return *this;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> Matrix<heigth, width>::operator+(const Matrix<heigth, width> &other) const
{
    Matrix<heigth, width> result = *this;
	result += other;
	return result;

}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> Matrix<heigth, width>::operator+(float val) const
{
    Matrix<heigth, width> result = *this;
	result += val;
	return result;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> Matrix<heigth, width>::operator-(const Matrix<heigth, width> &other) const
{
    Matrix<heigth, width> result = *this;
	result -= other;
	return result;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> Matrix<heigth, width>::operator-(float val) const
{
    Matrix<heigth, width> result = *this;
	result -= val;
	return result;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> &Matrix<heigth, width>::operator*=(float val)
{
    for(u_int i = 0; i < _values.size(); ++i) _values[i] *= val;
	return *this;
}

template <u_short heigth, u_short width>
inline Matrix<heigth, width> Matrix<heigth, width>::operator*(float val) const
{
    Matrix<heigth, width> result = *this;
	result *= val;
	return result;
}

template <u_short heigth, u_short width> template <u_short other_width>
Matrix<heigth, other_width> Matrix<heigth, width>::operator*(const Matrix<width, other_width>& other) const
{
	Matrix<heigth, other_width> result;
	Multiply(other, result);
	return result;
}

template <u_short heigth, u_short width> template <u_short other_width>
void Matrix<heigth, width>::Multiply(const Matrix<width, other_width>& other, Matrix<heigth, other_width>& result) const
{
	std::array<Row, other_width> other_columns;
	std::array<Row, heigth> this_rows;
	for(u_short row = 0; row < heigth; ++row)
		for(u_short col = 0; col < width; ++col)
			this_rows[row][col] = this->operator()(row, col);

	for(u_short col = 0; col < other_width; ++col)
		for(u_short row = 0; row < width; ++row)
			other_columns[col][row] = other(row, col);

	for(u_short row = 0; row < heigth; ++row)
		for(u_short col = 0; col < other_width; ++col)
			result(row, col) = Multiply_rows(this_rows[row], other_columns[col]);
}

template <u_short heigth, u_short width>
const MatShape& Matrix<heigth, width>::Shape() const
{
	return Matrix<heigth, width>::shape;
}

template <u_short size>
Matrix <size, size> Eye(float val)
{
	Matrix <size, size> result{};
	for(u_short i = 0; i < size; ++i)
		result(i, i) = val;
	return result;
}

} //namespace Mat
#endif /* UTIL_INC_MATRIX_HPP_ */
