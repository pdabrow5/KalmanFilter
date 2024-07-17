################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripherals/BMX160/src/DFRobot_BMX160.c \
../Peripherals/BMX160/src/common_porting.c 

C_DEPS += \
./Peripherals/BMX160/src/DFRobot_BMX160.d \
./Peripherals/BMX160/src/common_porting.d 

OBJS += \
./Peripherals/BMX160/src/DFRobot_BMX160.o \
./Peripherals/BMX160/src/common_porting.o 


# Each subdirectory must supply rules for building sources it contributes
Peripherals/BMX160/src/%.o Peripherals/BMX160/src/%.su Peripherals/BMX160/src/%.cyclo: ../Peripherals/BMX160/src/%.c Peripherals/BMX160/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Platform/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Math/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Algorithms" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripherals-2f-BMX160-2f-src

clean-Peripherals-2f-BMX160-2f-src:
	-$(RM) ./Peripherals/BMX160/src/DFRobot_BMX160.cyclo ./Peripherals/BMX160/src/DFRobot_BMX160.d ./Peripherals/BMX160/src/DFRobot_BMX160.o ./Peripherals/BMX160/src/DFRobot_BMX160.su ./Peripherals/BMX160/src/common_porting.cyclo ./Peripherals/BMX160/src/common_porting.d ./Peripherals/BMX160/src/common_porting.o ./Peripherals/BMX160/src/common_porting.su

.PHONY: clean-Peripherals-2f-BMX160-2f-src

