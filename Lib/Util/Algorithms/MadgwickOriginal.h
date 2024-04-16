/*
 * MadgwickOriginal.h
 *
 *  Created on: Apr 14, 2024
 *      Author: pawda
 */

#ifndef UTIL_ALGORITHMS_MADGWICKORIGINAL_H_
#define UTIL_ALGORITHMS_MADGWICKORIGINAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

void filterUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, const float time);
float getRoll();
float getPitch();
float getYaw();

float GetW();
float GetX();
float GetY();
float GetZ();

#ifdef __cplusplus
}
#endif

#endif /* UTIL_ALGORITHMS_MADGWICKORIGINAL_H_ */
