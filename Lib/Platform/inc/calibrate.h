/*
 * calibrate.h
 *
 *  Created on: Apr 8, 2024
 *      Author: pawda
 */

#ifndef PLATFORM_INC_CALIBRATE_H_
#define PLATFORM_INC_CALIBRATE_H_

#include <Math/inc/Matrix.hpp>

using namespace Mat;

Matrix<3, 1> CalibrateMag(const Matrix<3, 1>& MagVec);
Matrix<3, 1> CalibrateAcc(const Matrix<3, 1>& AccVec);
Matrix<3, 1> CalibrateGyro(const Matrix<3, 1>& GyroVec);

#endif /* PLATFORM_INC_CALIBRATE_H_ */
