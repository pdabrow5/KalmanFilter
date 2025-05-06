/*
 * calibrate.cpp
 *
 *  Created on: Apr 8, 2024
 *      Author: pawda
 */
#include "calibrate.h"

#include "UtilTypes.h"

using namespace Mat;

const Matrix<3,3> MagA({0.116923f, 0.0f, 0.0f,
						0.0f, 0.116923f, 0.0f,
						0.0f, 0.0f, 0.13103f});
const Matrix<3, 1> MagB({26.0f, 23.0f, -146.5f});

//const Matrix<3,3> MagA({0.621f, 0.002f, 0.0f,
//						0.002f, 0.620f, 0.005f,
//						0.0f, 0.005f, 2.597f});
//const Matrix<3, 1> MagB({8.77f, 2.90f, -11.92f});

//const Matrix<3,3> AccA({0.994382, 0.002869, -0.000884,
//						0.002869, 1.000491, 0.000648,
//						-0.000884, 0.000648, 0.998789});
//const Matrix<3, 1> AccB({0.364931, 0.282141, -0.210647});

//const Matrix<3,3> AccA({0.995866, 0.012699, -0.001166,
//						0.012699, 0.996956, 0.000580,
//						-0.001166, 0.000580, 0.996647});
//const Matrix<3, 1> AccB({0.349030, 0.113962, -0.205569});

const Matrix<3,3> AccA({0.984501, 0.012699, -0.001166,
						0.012699, 0.988835, 0.000580,
						-0.001166, 0.000580, 0.980272 });
const Matrix<3, 1> AccB({0.357455, 0.192043, -0.190166});


const Matrix<3,3> GyroA({8.133f * DEG_2_RAD, 0.0, 0.0,
						0.0, 8.238f * DEG_2_RAD, 0.0,
						0.0, 0.0, 8.193f * DEG_2_RAD});
const Matrix<3, 1> GyroB({-0.045665, 0.047779, -0.005830});
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
