################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Lib/Util/src/API.cpp \
../Lib/Util/src/Matrix.cpp 

OBJS += \
./Lib/Util/src/API.o \
./Lib/Util/src/Matrix.o 

CPP_DEPS += \
./Lib/Util/src/API.d \
./Lib/Util/src/Matrix.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/Util/src/%.o Lib/Util/src/%.su Lib/Util/src/%.cyclo: ../Lib/Util/src/%.cpp Lib/Util/src/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMI160" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMI160/inc" -I../Lib/Util/inc -Os -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-Util-2f-src

clean-Lib-2f-Util-2f-src:
	-$(RM) ./Lib/Util/src/API.cyclo ./Lib/Util/src/API.d ./Lib/Util/src/API.o ./Lib/Util/src/API.su ./Lib/Util/src/Matrix.cyclo ./Lib/Util/src/Matrix.d ./Lib/Util/src/Matrix.o ./Lib/Util/src/Matrix.su

.PHONY: clean-Lib-2f-Util-2f-src

