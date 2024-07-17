################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Lib/Util/Math/src/Quaternion.cpp 

OBJS += \
./Lib/Util/Math/src/Quaternion.o 

CPP_DEPS += \
./Lib/Util/Math/src/Quaternion.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/Util/Math/src/%.o Lib/Util/Math/src/%.su Lib/Util/Math/src/%.cyclo: ../Lib/Util/Math/src/%.cpp Lib/Util/Math/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Platform/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Math/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Algorithms" -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-Util-2f-Math-2f-src

clean-Lib-2f-Util-2f-Math-2f-src:
	-$(RM) ./Lib/Util/Math/src/Quaternion.cyclo ./Lib/Util/Math/src/Quaternion.d ./Lib/Util/Math/src/Quaternion.o ./Lib/Util/Math/src/Quaternion.su

.PHONY: clean-Lib-2f-Util-2f-Math-2f-src

