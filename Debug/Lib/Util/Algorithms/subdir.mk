################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Lib/Util/Algorithms/MadgwickOriginal.c 

CPP_SRCS += \
../Lib/Util/Algorithms/MadgwickAHRSclass.cpp \
../Lib/Util/Algorithms/MyMadgwick.cpp 

C_DEPS += \
./Lib/Util/Algorithms/MadgwickOriginal.d 

OBJS += \
./Lib/Util/Algorithms/MadgwickAHRSclass.o \
./Lib/Util/Algorithms/MadgwickOriginal.o \
./Lib/Util/Algorithms/MyMadgwick.o 

CPP_DEPS += \
./Lib/Util/Algorithms/MadgwickAHRSclass.d \
./Lib/Util/Algorithms/MyMadgwick.d 


# Each subdirectory must supply rules for building sources it contributes
Lib/Util/Algorithms/%.o Lib/Util/Algorithms/%.su Lib/Util/Algorithms/%.cyclo: ../Lib/Util/Algorithms/%.cpp Lib/Util/Algorithms/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m7 -std=gnu++14 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Lib/Util/inc -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/API" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Platform/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Math/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Algorithms" -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"
Lib/Util/Algorithms/%.o Lib/Util/Algorithms/%.su Lib/Util/Algorithms/%.cyclo: ../Lib/Util/Algorithms/%.c Lib/Util/Algorithms/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -I../Peripherals/BMI160/inc -I../Lib/Util/inc -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Peripherals/BMX160/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/API" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Platform/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Math/inc" -I"C:/Users/pawda/STM32CubeIDE/workspace_1.13.2/KalmanFilter/Lib/Util/Algorithms" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Lib-2f-Util-2f-Algorithms

clean-Lib-2f-Util-2f-Algorithms:
	-$(RM) ./Lib/Util/Algorithms/MadgwickAHRSclass.cyclo ./Lib/Util/Algorithms/MadgwickAHRSclass.d ./Lib/Util/Algorithms/MadgwickAHRSclass.o ./Lib/Util/Algorithms/MadgwickAHRSclass.su ./Lib/Util/Algorithms/MadgwickOriginal.cyclo ./Lib/Util/Algorithms/MadgwickOriginal.d ./Lib/Util/Algorithms/MadgwickOriginal.o ./Lib/Util/Algorithms/MadgwickOriginal.su ./Lib/Util/Algorithms/MyMadgwick.cyclo ./Lib/Util/Algorithms/MyMadgwick.d ./Lib/Util/Algorithms/MyMadgwick.o ./Lib/Util/Algorithms/MyMadgwick.su

.PHONY: clean-Lib-2f-Util-2f-Algorithms

