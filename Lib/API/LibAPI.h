/*
 * LibAPI.h
 *
 *  Created on: Apr 6, 2024
 *      Author: pawda
 */

#ifndef API_LIBAPI_H_
#define API_LIBAPI_H_

#include <stdint.h>

#include "UtilTypes.h"

#ifdef __cplusplus
extern "C"
{
#endif

uint8_t MadgwickInit();
uint8_t MadgwickUpdate(const AGMSensorData* sensorData);

#ifdef __cplusplus
}
#endif

#endif /* API_LIBAPI_H_ */
