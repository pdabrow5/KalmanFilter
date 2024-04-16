/*
 * UtilTypes.h
 *
 *  Created on: Apr 6, 2024
 *      Author: pawda
 */

#ifndef UTIL_UTILTYPES_H_
#define UTIL_UTILTYPES_H_

#define DEG_2_RAD 0.01744444f

typedef struct Vec3_t
{
	float x, y, z;
} Vec3;

typedef struct AGMSensorData_t
{
	Vec3 Mag;
	Vec3 Acc;
	Vec3 Gyro;
	float SensorTime;
} AGMSensorData;


#endif /* UTIL_UTILTYPES_H_ */
