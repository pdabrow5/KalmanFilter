/*
 * logger.h
 *
 *  Created on: Feb 11, 2024
 *      Author: pawda
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include <stdio.h>
#include <stm32h743xx.h>
#include "stm32h7xx_hal.h"

#ifdef DEBUG

#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define LOG(...) \
	  {printf("%lu	%s(%u):	", HAL_GetTick(),__func__, __LINE__); \
	  printf(__VA_ARGS__); printf("\n\r");}
#else
#define LOG(...)

#endif

#endif /* INC_LOGGER_H_ */