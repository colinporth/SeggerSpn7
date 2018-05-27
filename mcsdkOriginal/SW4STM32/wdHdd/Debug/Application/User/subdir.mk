################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/main.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_api.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_config.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_parameters.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_tasks.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/motor_control_protocol.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/motorcontrol.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/stm32f30x_mc_it.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/stm32f3xx_hal_msp.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/stm32f3xx_it.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/ui_task.c \
C:/Users/colin/Desktop/wdHDD/wdHdd/Src/user_interface.c 

OBJS += \
./Application/User/main.o \
./Application/User/mc_api.o \
./Application/User/mc_config.o \
./Application/User/mc_parameters.o \
./Application/User/mc_tasks.o \
./Application/User/motor_control_protocol.o \
./Application/User/motorcontrol.o \
./Application/User/stm32f30x_mc_it.o \
./Application/User/stm32f3xx_hal_msp.o \
./Application/User/stm32f3xx_it.o \
./Application/User/ui_task.o \
./Application/User/user_interface.o 

C_DEPS += \
./Application/User/main.d \
./Application/User/mc_api.d \
./Application/User/mc_config.d \
./Application/User/mc_parameters.d \
./Application/User/mc_tasks.d \
./Application/User/motor_control_protocol.d \
./Application/User/motorcontrol.d \
./Application/User/stm32f30x_mc_it.d \
./Application/User/stm32f3xx_hal_msp.d \
./Application/User/stm32f3xx_it.d \
./Application/User/ui_task.d \
./Application/User/user_interface.d 


# Each subdirectory must supply rules for building sources it contributes
Application/User/main.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/mc_api.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_api.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/mc_config.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_config.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/mc_parameters.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_parameters.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/mc_tasks.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/mc_tasks.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/motor_control_protocol.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/motor_control_protocol.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/motorcontrol.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/motorcontrol.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f30x_mc_it.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/stm32f30x_mc_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f3xx_hal_msp.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/stm32f3xx_hal_msp.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/stm32f3xx_it.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/stm32f3xx_it.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/ui_task.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/ui_task.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Application/User/user_interface.o: C:/Users/colin/Desktop/wdHDD/wdHdd/Src/user_interface.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DARM_MATH_CM4 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F303xE -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/STM32F3xx_HAL_Driver/Inc/Legacy" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Legacy/Cube" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/Applications/Test_MotorApp/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/Any/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/MCLib/F3xx/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/UILibrary/Inc" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/MCSDK_v5.0.3/MotorControl/MCSDK/SystemDriveParams" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Device/ST/STM32F3xx/Include" -I"C:/Users/colin/Desktop/wdHDD/wdHdd/Drivers/CMSIS/Include"  -O3 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


