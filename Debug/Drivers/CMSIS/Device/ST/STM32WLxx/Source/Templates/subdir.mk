################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.c 

OBJS += \
./Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.o 

C_DEPS += \
./Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/%.o Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/%.su Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/%.cyclo: ../Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/%.c Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32WLxx-2f-Source-2f-Templates

clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32WLxx-2f-Source-2f-Templates:
	-$(RM) ./Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.cyclo ./Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.d ./Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.o ./Drivers/CMSIS/Device/ST/STM32WLxx/Source/Templates/system_stm32wlxx.su

.PHONY: clean-Drivers-2f-CMSIS-2f-Device-2f-ST-2f-STM32WLxx-2f-Source-2f-Templates

