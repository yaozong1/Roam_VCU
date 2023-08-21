################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../RTT/SEGGER_RTT.c \
../RTT/SEGGER_RTT_printf.c 

OBJS += \
./RTT/SEGGER_RTT.o \
./RTT/SEGGER_RTT_printf.o 

C_DEPS += \
./RTT/SEGGER_RTT.d \
./RTT/SEGGER_RTT_printf.d 


# Each subdirectory must supply rules for building sources it contributes
RTT/%.o RTT/%.su RTT/%.cyclo: ../RTT/%.c RTT/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F105xC -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/h1576/Desktop/Roam/Roam_VCU/RTT" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-RTT

clean-RTT:
	-$(RM) ./RTT/SEGGER_RTT.cyclo ./RTT/SEGGER_RTT.d ./RTT/SEGGER_RTT.o ./RTT/SEGGER_RTT.su ./RTT/SEGGER_RTT_printf.cyclo ./RTT/SEGGER_RTT_printf.d ./RTT/SEGGER_RTT_printf.o ./RTT/SEGGER_RTT_printf.su

.PHONY: clean-RTT

