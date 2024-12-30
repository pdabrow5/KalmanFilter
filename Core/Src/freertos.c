/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "logger.h"
#include "usart.h"
#include "semphr.h"

#include "../../Peripherals/BMX160/inc/DFRobot_BMX160.h"
#include "../../Peripherals/simpleRTK2B/GNSS.h"
#include "LibAPI.h"
#include "../../Lib/Util/Algorithms/MadgwickOriginal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
sBmx160SensorData_t Omagn;
sBmx160SensorData_t Oaccel;
sBmx160SensorData_t Ogyro;
AGMSensorData SensorData;

sBmx160SensorData_t MeanGyro = {0.0f, 0.0f, 0.0f, 0};
float meanAlpha = 0.1;
int MaxSteps = 250;
int currStep = 0;
volatile int inter_flag = 0;
volatile int INTERRUPTS = 0;
BaseType_t GiveSemaphoreResult = 0;

HAL_StatusTypeDef uart_result;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for AccMeassure */
osThreadId_t AccMeassureHandle;
uint32_t AccMeassureBuffer[ 2048 ];
osStaticThreadDef_t AccMeassureControlBlock;
const osThreadAttr_t AccMeassure_attributes = {
  .name = "AccMeassure",
  .cb_mem = &AccMeassureControlBlock,
  .cb_size = sizeof(AccMeassureControlBlock),
  .stack_mem = &AccMeassureBuffer[0],
  .stack_size = sizeof(AccMeassureBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for ReceiveGNSSData */
osThreadId_t ReceiveGNSSDataHandle;
uint32_t ReceiveGNSSDataBuffer[ 2048 ];
osStaticThreadDef_t ReceiveGNSSDataControlBlock;
const osThreadAttr_t ReceiveGNSSData_attributes = {
  .name = "ReceiveGNSSData",
  .cb_mem = &ReceiveGNSSDataControlBlock,
  .cb_size = sizeof(ReceiveGNSSDataControlBlock),
  .stack_mem = &ReceiveGNSSDataBuffer[0],
  .stack_size = sizeof(ReceiveGNSSDataBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for GNNS_UART_INTERRUPT */
osSemaphoreId_t GNNS_UART_INTERRUPTHandle;
osStaticSemaphoreDef_t GNNS_UART_INTERRUPTControlBlock;
const osSemaphoreAttr_t GNNS_UART_INTERRUPT_attributes = {
  .name = "GNNS_UART_INTERRUPT",
  .cb_mem = &GNNS_UART_INTERRUPTControlBlock,
  .cb_size = sizeof(GNNS_UART_INTERRUPTControlBlock),
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//void GetGNSS()
//{
//	GNSS_GetUniqID(&GNSS_Handle);
//	GNSS_ParseBuffer(&GNSS_Handle);
//	osDelay(250);
//	GNSS_GetPVTData(&GNSS_Handle);
//	GNSS_ParseBuffer(&GNSS_Handle);
//	LOG("Buffer: %d", GNSS_Handle.uartWorkingBuffer[0]);
//	LOG("Day: %d-%d-%d \r\n", GNSS_Handle.day, GNSS_Handle.month,GNSS_Handle.year);
//	LOG("Time: %d:%d:%d \r\n", GNSS_Handle.hour, GNSS_Handle.min,GNSS_Handle.sec);
//	LOG("Status of fix: %d \r\n", GNSS_Handle.fixType);
//	LOG("Latitude: %f \r\n", GNSS_Handle.fLat);
//	LOG("Longitude: %f \r\n",(float) GNSS_Handle.lon / 10000000.0);
//	LOG("Height above ellipsoid: %d \r\n", GNSS_Handle.height);
//	LOG("Height above mean sea level: %d \r\n", GNSS_Handle.hMSL);
//	LOG("Ground Speed (2-D): %d \r\n", GNSS_Handle.gSpeed);
//	LOG("Unique ID: %04X %04X %04X %04X %04X %04X\n\r",
//	GNSS_Handle.uniqueID[0], GNSS_Handle.uniqueID[1],
//	GNSS_Handle.uniqueID[2], GNSS_Handle.uniqueID[3],
//	GNSS_Handle.uniqueID[4], GNSS_Handle.uniqueID[5]);
//}

void DelayFunction(uint16_t ms)
{
	osDelay(ms);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	inter_flag = 0;
	++INTERRUPTS;
	//HAL_UART_Receive_DMA(&huart2, GNSS_Handle.uartWorkingBuffer, 10);
	//xTaskResumeFromISR(ReceiveGNSSDataHandle);
	xSemaphoreGiveFromISR(GNNS_UART_INTERRUPTHandle, &GiveSemaphoreResult);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == USART1)
	{
		inter_flag = Size;
		++INTERRUPTS;
		xSemaphoreGiveFromISR(GNNS_UART_INTERRUPTHandle, &GiveSemaphoreResult);
	}
}

void Receive_IT()
{
	uart_result = HAL_UART_Receive_IT(GNSS_Handle.huart, GNSS_Handle.uartWorkingBuffer, 421);
	if(uart_result != 0) LOG("Receive_IT Status: %d", uart_result);
}

void Receive_IT_IDLE()
{
	uart_result = HAL_UARTEx_ReceiveToIdle_IT(GNSS_Handle.huart, GNSS_Handle.uartWorkingBuffer, sizeof(GNSS_Handle.uartWorkingBuffer));
	if(uart_result != 0) LOG("Receive_IT_IDLE Status: %d", uart_result);
}

void AskPvtReceive_IT_IDLE()
{
	LOG("Send PVT querry");
	uart_result = HAL_UART_Transmit(GNSS_Handle.huart, getPVTData, sizeof(getPVTData) / sizeof(uint8_t), 500);
	if(uart_result != 0) LOG("HAL_UART_Transmit Status: %d", uart_result);
	LOG("Waiting for response");
	uart_result = HAL_UARTEx_ReceiveToIdle_IT(GNSS_Handle.huart, GNSS_Handle.uartWorkingBuffer, 100);
	if(uart_result != 0) LOG("Receive_IT_IDLE Status: %d", uart_result);
}

void Receive_DMA()
{
	uart_result = HAL_UART_Receive_DMA(GNSS_Handle.huart, GNSS_Handle.uartWorkingBuffer, 10);
	if(uart_result != 0) LOG("Receive_DMA Status: %d", uart_result);
}

void Receive_DMA_IDLE()
{
	uart_result = 	HAL_UARTEx_ReceiveToIdle_DMA(GNSS_Handle.huart, GNSS_Handle.uartWorkingBuffer, 10);
	if(uart_result != 0) LOG("Receive_DMA_IDLE Status: %d", uart_result);
}

void Ask_For_PVT()
{
	uart_result = HAL_UART_Transmit(GNSS_Handle.huart, getPVTData, sizeof(getPVTData) / sizeof(uint8_t), 500);
	if(uart_result != 0) LOG("HAL_UART_Transmit Status: %d", uart_result);
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAccMeassureTask(void *argument);
void StartReceiveGNSSDataTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
//		Bmx160_init();
	LOG("BMX INIT");
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of GNNS_UART_INTERRUPT */
  GNNS_UART_INTERRUPTHandle = osSemaphoreNew(1, 1, &GNNS_UART_INTERRUPT_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of AccMeassure */
  AccMeassureHandle = osThreadNew(StartAccMeassureTask, NULL, &AccMeassure_attributes);

  /* creation of ReceiveGNSSData */
  ReceiveGNSSDataHandle = osThreadNew(StartReceiveGNSSDataTask, NULL, &ReceiveGNSSData_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	Vec3 Pos, Acc, Vel;
	int flag = 0;
  /* Infinite loop */
  for(;;)
  {
	if(HAL_GPIO_ReadPin(UserButton_GPIO_Port, UserButton_Pin))
	{
		LOG("PUSHED BUTTON");
		//GNSS_GetUniqID(&GNSS_Handle);
		if(flag == 0)
		{
			LOG("Receive DMA");
			Receive_DMA();
			flag = 1;
		}
		else
		{
			LOG("Receive IT");
			Receive_IT();
			flag = 0;
		}
		//ResetKinematics();
		//LOG("RESET STATE!!!!");
//		float mx = 0.0f;
//		float my = 0.0f;
//		float mz = 0.0f;
//		for(int i = 0; i < 1000; ++i)
//		{
//			Bmx160_getAllData(&Omagn, &Ogyro, &Oaccel);
//			mx += Oaccel.x;
//			my += Oaccel.y;
//			mz += Oaccel.z;
//			osDelay(1);
//		}
//		mx = mx / 1000.0f;
//		my = my / 1000.0f;
//		mz = mz / 1000.0f;
//		int imx = 0;
//		int imy = 0;
//		int imz = 0;
//		imx = mx * 1000.0f;
//		imy = my * 1000.0f;
//		imz = mz * 1000.0f;
//		//printf("Raw:0,0,0,0,0,0,%d,%d,%d\n\r", imx, imy, imz);
//		printf("%f %f %f\n\r", mx, my, mz);
	}
//	Pos = GetPosition();
//	Acc = GetAcceleration();
//	Vel = GetVelocity();
	//LOG("Vel: %f, \t%f, \t%f\tPos: %f, \t%f, \t%f", Vel.x, Vel.y, Vel.z, Pos.x, Pos.y, Pos.z);
	//LOG("INTERRUPTS: %d, inter_flag: %d, UART state: %d", INTERRUPTS, inter_flag, GNSS_Handle.huart->RxState);
	//GNSS_GetPVTData(&GNSS_Handle);
	//GNSS_GetPVTData(&GNSS_Handle);
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartAccMeassureTask */
/**
* @brief Function implementing the AccMeassure thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAccMeassureTask */
void StartAccMeassureTask(void *argument)
{
  /* USER CODE BEGIN StartAccMeassureTask */
  /* Infinite loop */
	Bmx160_init();
	LOG("After INIT");
	Bmx160_getAllData(&Omagn, &Ogyro, &Oaccel);
	SensorData.Acc.x = Oaccel.x;
	SensorData.Acc.y = Oaccel.y;
	SensorData.Acc.z = Oaccel.z;
	SensorData.Gyro.x = Ogyro.x;
	SensorData.Gyro.y = Ogyro.y;
	SensorData.Gyro.z = Ogyro.z;
	SensorData.Mag.x = Omagn.x;
	SensorData.Mag.y = Omagn.y;
	SensorData.Mag.z = Omagn.z;
	SensorData.SensorTime = Omagn.sensortime;
	InitAlgorithms(&SensorData);
  for(;;)
  {
	//GetGNSS();
	//GNSS_GetUniqID(&GNSS_Handle);
	//GNSS_GetPVTData(&GNSS_Handle);
//	if(HAL_GPIO_ReadPin(UserButton_GPIO_Port, UserButton_Pin))
//	{
//		for(int i = 0; i < steps; ++i)
//		{
//			Bmx160_getAllData(&Omagn, &Ogyro, &Oaccel);
//			sum_x += Oaccel.x;
//			sum_y += Oaccel.y;
//			sum_z += Oaccel.z;
//			osDelay(5);
//		}
//		printf("%f %f %f\n\r", (sum_x/((float)(steps))), (sum_y/((float)(steps))), (sum_z/((float)(steps))));
//		sum_x = 0.0f; sum_y = 0.0f; sum_z = 0.0f;
//	}
	Bmx160_getAllData(&Omagn, &Ogyro, &Oaccel);
	SensorData.Acc.x = Oaccel.x;
	SensorData.Acc.y = Oaccel.y;
	SensorData.Acc.z = Oaccel.z;
	SensorData.Gyro.x = Ogyro.x;
	SensorData.Gyro.y = Ogyro.y;
	SensorData.Gyro.z = Ogyro.z;
	SensorData.Mag.x = Omagn.x;
	SensorData.Mag.y = Omagn.y;
	SensorData.Mag.z = Omagn.z;
	SensorData.SensorTime = Omagn.sensortime;
	MadgwickUpdate(&SensorData);
//	int mx = 0;
//	int my = 0;
//	int mz = 0;
//	mx = Omagn.x * 10.0f;
//	my = Omagn.y * 10.0f;
//	mz = Omagn.z * 10.0f;
//	//LOG("Raw:0,0,0,0,0,0,%d,%d,%d\n\r", mx, my, mz);
//	//LOG("Raw:%f,%f,%f,%f,%f,%f,%f,%f,%f\n\r",
//			SensorData.Acc.x, SensorData.Acc.y, SensorData.Acc.z,
//			SensorData.Gyro.x, SensorData.Gyro.y, SensorData.Gyro.z,
//			SensorData.Mag.x, SensorData.Mag.y, SensorData.Mag.z);
	osDelay(1);

  }
  /* USER CODE END StartAccMeassureTask */
}

/* USER CODE BEGIN Header_StartReceiveGNSSDataTask */
/**
* @brief Function implementing the ReceiveGNSSData thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveGNSSDataTask */
void StartReceiveGNSSDataTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveGNSSDataTask */
  /* Infinite loop */
	LOG("GNSS INIT");
	GNSS_Init(&GNSS_Handle, &huart1, DelayFunction);
	osDelay(1000);
	LOG("GNSS LOAD CONFIG");
	//GNSS_LoadConfig(&GNSS_Handle);
	LOG("GNSS CONFIG LOADED!!!");
	xSemaphoreTake(GNNS_UART_INTERRUPTHandle, portMAX_DELAY);
  for(;;)
  {
	HAL_UART_Transmit_DMA(GNSS_Handle.huart, getPVTData,
			sizeof(getPVTData) / sizeof(uint8_t));
    Receive_IT_IDLE();
	//AskPvtReceive_IT_IDLE();
	LOG("Waiting for semaphore...");
	xSemaphoreTake(GNNS_UART_INTERRUPTHandle, portMAX_DELAY);
	LOG("Got into loop, no.%d", INTERRUPTS);
	if(inter_flag == 0)
	{
		LOG("NORMAL GNSS INTERRUPT[%d]!!!", INTERRUPTS);
	}
	else
	{
		LOG("IDLE GNSS INTERRUPT[%d], SIZE: %d", INTERRUPTS, inter_flag);
	}

//	printf("\n\n");
//	uint16_t sizeofTable = inter_flag == 0 ? sizeof(GNSS_Handle.uartWorkingBuffer) : inter_flag;
//	for(uint16_t size = 0; size < sizeofTable; ++size)
//		printf("%x ", GNSS_Handle.uartWorkingBuffer[size]);
//	printf("\n\n\r");
    //printf("%s \n\r", GNSS_Handle.uartWorkingBuffer);
	//GNSS_ParsePVTData(&GNSS_Handle);
	GNSS_ParsePVTDataPTR(&GNSS_Handle);
	LOG("Buffer: %d", GNSS_Handle.uartWorkingBuffer[0]);
	LOG("Day: %d-%d-%d \r\n", GNSS_Handle.day, GNSS_Handle.month,GNSS_Handle.year);
	LOG("Time: %d:%d:%d \r\n", GNSS_Handle.hour, GNSS_Handle.min,GNSS_Handle.sec);
	LOG("Status of fix: %d \r\n", GNSS_Handle.fixType);
	LOG("Latitude: %f \r\n", GNSS_Handle.fLat);
	LOG("Longitude: %f \r\n",(float) GNSS_Handle.lon / 10000000.0);
	LOG("Height above ellipsoid: %d \r\n", GNSS_Handle.height);
	LOG("Height above mean sea level: %d \r\n", GNSS_Handle.hMSL);
	LOG("Ground Speed (2-D): %d \r\n", GNSS_Handle.gSpeed);
	LOG("");
	OnGNSSData(&GNSS_Handle);
  }
  /* USER CODE END StartReceiveGNSSDataTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

