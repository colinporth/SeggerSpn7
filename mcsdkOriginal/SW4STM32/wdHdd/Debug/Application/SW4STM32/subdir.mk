################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
C:/Users/colin/Desktop/wdHDD/wdHdd/SW4STM32/startup_stm32f303xe.s 

OBJS += \
./Application/SW4STM32/startup_stm32f303xe.o 


# Each subdirectory must supply rules for building sources it contributes
Application/SW4STM32/startup_stm32f303xe.o: C:/Users/colin/Desktop/wdHDD/wdHdd/SW4STM32/startup_stm32f303xe.s
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Assembler'
	@echo $(PWD)
	arm-none-eabi-as -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


