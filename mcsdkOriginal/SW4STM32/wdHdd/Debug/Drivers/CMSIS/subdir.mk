################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/system_stm32f3xx.c 

OBJS += \
./Drivers/CMSIS/system_stm32f3xx.o 

C_DEPS += \
./Drivers/CMSIS/system_stm32f3xx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/system_stm32f3xx.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/system_stm32f3xx.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


