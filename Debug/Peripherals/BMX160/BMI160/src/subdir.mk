################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripherals/BMX160/BMI160/src/bmi160.c 

C_DEPS += \
./Peripherals/BMX160/BMI160/src/bmi160.d 

OBJS += \
./Peripherals/BMX160/BMI160/src/bmi160.o 


# Each subdirectory must supply rules for building sources it contributes
Peripherals/BMX160/BMI160/src/%.o Peripherals/BMX160/BMI160/src/%.su Peripherals/BMX160/BMI160/src/%.cyclo: ../Peripherals/BMX160/BMI160/src/%.c Peripherals/BMX160/BMI160/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Peripherals/BMI160/inc -I../Lib/Util/inc -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/BMM150/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/BMI160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripherals-2f-BMX160-2f-BMI160-2f-src

clean-Peripherals-2f-BMX160-2f-BMI160-2f-src:
	-$(RM) ./Peripherals/BMX160/BMI160/src/bmi160.cyclo ./Peripherals/BMX160/BMI160/src/bmi160.d ./Peripherals/BMX160/BMI160/src/bmi160.o ./Peripherals/BMX160/BMI160/src/bmi160.su

.PHONY: clean-Peripherals-2f-BMX160-2f-BMI160-2f-src

