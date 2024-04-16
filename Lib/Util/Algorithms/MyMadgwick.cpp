/*
 * MyMadgwick.cpp
 *
 *  Created on: Apr 14, 2024
 *      Author: pawda
 */

#include "MyMadgwick.hpp"

using namespace Mat;

namespace Algorithms
{

MadgwickFilter::MadgwickFilter(float beta): _beta(beta)
{
	_q = {0.0f, 0.0f, 0.0f, 0.0f};
}

void MadgwickFilter::Update(AGMSensorData_t* sensorData)
{

}

Matrix<3,1> F_q_d_s(const Quaternion& q, const Quaternion& d, const Quaternion& s)
{

}

} //namespace Algorithms
