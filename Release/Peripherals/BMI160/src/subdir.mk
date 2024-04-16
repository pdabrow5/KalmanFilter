################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripherals/BMI160/src/bmi160.c \
../Peripherals/BMI160/src/bmi160_wrapper.c \
../Peripherals/BMI160/src/common_porting.c 

C_DEPS += \
./Peripherals/BMI160/src/bmi160.d \
./Peripherals/BMI160/src/bmi160_wrapper.d \
./Peripherals/BMI160/src/common_porting.d 

OBJS += \
./Peripherals/BMI160/src/bmi160.o \
./Peripherals/BMI160/src/bmi160_wrapper.o \
./Peripherals/BMI160/src/common_porting.o 


# Each subdirectory must supply rules for building sources it contributes
Peripherals/BMI160/src/%.o Peripherals/BMI160/src/%.su Peripherals/BMI160/src/%.cyclo: ../Peripherals/BMI160/src/%.c Peripherals/BMI160/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMI160" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMI160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/inc" -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripherals-2f-BMI160-2f-src

clean-Peripherals-2f-BMI160-2f-src:
	-$(RM) ./Peripherals/BMI160/src/bmi160.cyclo ./Peripherals/BMI160/src/bmi160.d ./Peripherals/BMI160/src/bmi160.o ./Peripherals/BMI160/src/bmi160.su ./Peripherals/BMI160/src/bmi160_wrapper.cyclo ./Peripherals/BMI160/src/bmi160_wrapper.d ./Peripherals/BMI160/src/bmi160_wrapper.o ./Peripherals/BMI160/src/bmi160_wrapper.su ./Peripherals/BMI160/src/common_porting.cyclo ./Peripherals/BMI160/src/common_porting.d ./Peripherals/BMI160/src/common_porting.o ./Peripherals/BMI160/src/common_porting.su

.PHONY: clean-Peripherals-2f-BMI160-2f-src

