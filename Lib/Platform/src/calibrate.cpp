/*
 * calibrate.cpp
 *
 *  Created on: Apr 8, 2024
 *      Author: pawda
 */
#include "calibrate.h"

#include "UtilTypes.h"

using namespace Mat;

const Matrix<3,3> MagA({1.527812, -0.015809, 0.017590,
						-0.015809, 1.524591, 0.005030,
						0.017590, 0.005030, 2.722998});
const Matrix<3, 1> MagB({-44.070419, 26.891034, -12.777197});

const Matrix<3,3> AccA({0.994382, 0.002869, -0.000884,
						0.002869, 1.000491, 0.000648,
						-0.000884, 0.000648, 0.998789});

const Matrix<3, 1> AccB({0.364931, 0.282141, -0.210647});

const Matrix<3,3> GyroA({8.144 * DEG_2_RAD, 0.0, 0.0,
						0.0, 8.78 * DEG_2_RAD, 0.0,
						0.0, 0.0, 8.16 * DEG_2_RAD});
const Matrix<3, 1> GyroB({-0.0533, 0.0459, -0.0038});

Matrix<3, 1> CalibrateMag(const Matrix<3, 1>& MagVec)
{
	return (MagA * (MagVec - MagB));
}
Matrix<3, 1> CalibrateAcc(const Matrix<3, 1>& AccVec)
{
	return (AccA * (AccVec - AccB));
}

Matrix<3, 1> CalibrateGyro(const Matrix<3, 1>& GyroVec)
{
	return (GyroA * (GyroVec - GyroB));
}
