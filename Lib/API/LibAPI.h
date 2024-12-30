/*
 * LibAPI.h
 *
 *  Created on: Apr 6, 2024
 *      Author: pawda
 */

#ifndef API_LIBAPI_H_
#define API_LIBAPI_H_

#include <stdint.h>

#include "../../Peripherals/simpleRTK2B/GNSS.h"
#include "UtilTypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

uint8_t InitAlgorithms(const AGMSensorData* sensorData);
uint8_t ResetKinematics();
uint8_t MadgwickUpdate(const AGMSensorData* sensorData);
uint8_t OnGNSSData(const GNSS_StateHandle* GNSSData);

Vec3 GetPosition();
Vec3 GetVelocity();
Vec3 GetAcceleration();

#ifdef __cplusplus
}
#endif

#endif /* API_LIBAPI_H_ */
