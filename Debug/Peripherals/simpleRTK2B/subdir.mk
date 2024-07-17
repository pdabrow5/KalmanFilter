################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Peripherals/simpleRTK2B/GNSS.c 

C_DEPS += \
./Peripherals/simpleRTK2B/GNSS.d 

OBJS += \
./Peripherals/simpleRTK2B/GNSS.o 


# Each subdirectory must supply rules for building sources it contributes
Peripherals/simpleRTK2B/%.o Peripherals/simpleRTK2B/%.su Peripherals/simpleRTK2B/%.cyclo: ../Peripherals/simpleRTK2B/%.c Peripherals/simpleRTK2B/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Peripherals/BMI160/inc -I../Lib/Util/inc -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/API" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Platform/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Math/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Algorithms" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Peripherals-2f-simpleRTK2B

clean-Peripherals-2f-simpleRTK2B:
	-$(RM) ./Peripherals/simpleRTK2B/GNSS.cyclo ./Peripherals/simpleRTK2B/GNSS.d ./Peripherals/simpleRTK2B/GNSS.o ./Peripherals/simpleRTK2B/GNSS.su

.PHONY: clean-Peripherals-2f-simpleRTK2B

