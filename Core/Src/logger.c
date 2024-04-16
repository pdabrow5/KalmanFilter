/*
 * logger.c
 *
 *  Created on: Feb 22, 2024
 *      Author: pawda
 */
#include "logger.h"
#include "usart.h"

#ifdef DEBUG

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#endif
