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

#include "../../Peripherals/BMX160/inc/DFRobot_BMX160.h"
#include "LibAPI.h"
#include "../../Lib/Util/Algorithms/MadgwickOriginal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
const osThreadAttr_t AccMeassure_attributes = {
  .name = "AccMeassure",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartAccMeassureTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
	float roll, pitch, yaw;
  /* Infinite loop */
  for(;;)
  {
//	roll = MadgwickGetRoll();
//	pitch = MadgwickGetPitch();
//	yaw = MadgwickGetYaw();
	//LOG("1st: %f,	%f,	%f		2nd: %f,	%f,	%f", roll, pitch, yaw, getRoll(), getPitch(), getYaw());
    osDelay(50);
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
	int steps = 1000;
	float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
  for(;;)
  {
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
	//LOG("sensor time: %f, Mag: %f %f %f", SensorData.SensorTime, Omagn.x, Omagn.y, Omagn.z);
	MadgwickUpdate(&SensorData);
	//printf("%f %f %f\n\r", Omagn.x, Omagn.y, Omagn.z);
	osDelay(1);

  }
  /* USER CODE END StartAccMeassureTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

