################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.cpp 

OBJS += \
./Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.o 

CPP_DEPS += \
./Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/Util/Algorithms/ExtendedKalman/%.o Lib/Util/Algorithms/ExtendedKalman/%.su Lib/Util/Algorithms/ExtendedKalman/%.cyclo: ../Lib/Util/Algorithms/ExtendedKalman/%.cpp Lib/Util/Algorithms/ExtendedKalman/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Lib/Util/inc -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/API" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Platform/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Math/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Algorithms" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-Util-2f-Algorithms-2f-ExtendedKalman

clean-Lib-2f-Util-2f-Algorithms-2f-ExtendedKalman:
	-$(RM) ./Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.cyclo ./Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.d ./Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.o ./Lib/Util/Algorithms/ExtendedKalman/AHRSKalman.su

.PHONY: clean-Lib-2f-Util-2f-Algorithms-2f-ExtendedKalman

