/*
 * constants.h
 *
 *  Created on: Apr 8, 2024
 *      Author: pawda
 */

#ifndef PLATFORM_INC_CONSTANTS_H_
#define PLATFORM_INC_CONSTANTS_H_

#define Gravity 9.80655f
#define PI 3.14159f

constexpr float EarthRadius = 6371000; //meters
constexpr float _ER = 1.0f / EarthRadius;
constexpr float D2R = PI / 180.0f;
constexpr float R2D = 180.0f / PI;
constexpr float ms2s = 1.0f / 1000.0f;
constexpr float mm2m = 1.0f / 1000.0f;


#endif /* PLATFORM_INC_CONSTANTS_H_ */
