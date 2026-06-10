################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/bmi270_config.c \
../Core/Src/driver_bmp280.c \
../Core/Src/driver_bmp280_interface.c \
../Core/Src/main.c \
../Core/Src/ms5611.c \
../Core/Src/stm32wlxx_hal_msp.c \
../Core/Src/stm32wlxx_it.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32wlxx.c 

OBJS += \
./Core/Src/bmi270_config.o \
./Core/Src/driver_bmp280.o \
./Core/Src/driver_bmp280_interface.o \
./Core/Src/main.o \
./Core/Src/ms5611.o \
./Core/Src/stm32wlxx_hal_msp.o \
./Core/Src/stm32wlxx_it.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32wlxx.o 

C_DEPS += \
./Core/Src/bmi270_config.d \
./Core/Src/driver_bmp280.d \
./Core/Src/driver_bmp280_interface.d \
./Core/Src/main.d \
./Core/Src/ms5611.d \
./Core/Src/stm32wlxx_hal_msp.d \
./Core/Src/stm32wlxx_it.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32wlxx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DCORE_CM4 -DUSE_HAL_DRIVER -DSTM32WL55xx -c -I../Core/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc -I../Drivers/STM32WLxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WLxx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/bmi270_config.cyclo ./Core/Src/bmi270_config.d ./Core/Src/bmi270_config.o ./Core/Src/bmi270_config.su ./Core/Src/driver_bmp280.cyclo ./Core/Src/driver_bmp280.d ./Core/Src/driver_bmp280.o ./Core/Src/driver_bmp280.su ./Core/Src/driver_bmp280_interface.cyclo ./Core/Src/driver_bmp280_interface.d ./Core/Src/driver_bmp280_interface.o ./Core/Src/driver_bmp280_interface.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/ms5611.cyclo ./Core/Src/ms5611.d ./Core/Src/ms5611.o ./Core/Src/ms5611.su ./Core/Src/stm32wlxx_hal_msp.cyclo ./Core/Src/stm32wlxx_hal_msp.d ./Core/Src/stm32wlxx_hal_msp.o ./Core/Src/stm32wlxx_hal_msp.su ./Core/Src/stm32wlxx_it.cyclo ./Core/Src/stm32wlxx_it.d ./Core/Src/stm32wlxx_it.o ./Core/Src/stm32wlxx_it.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32wlxx.cyclo ./Core/Src/system_stm32wlxx.d ./Core/Src/system_stm32wlxx.o ./Core/Src/system_stm32wlxx.su

.PHONY: clean-Core-2f-Src

