################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Misc/Src/bme280.c 

OBJS += \
./Misc/Src/bme280.o 

C_DEPS += \
./Misc/Src/bme280.d 


# Each subdirectory must supply rules for building sources it contributes
Misc/Src/%.o Misc/Src/%.su: ../Misc/Src/%.c Misc/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F446xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Otavi/Documents/Embed/STM32/Projects/BME280_I2C/Misc/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Misc-2f-Src

clean-Misc-2f-Src:
	-$(RM) ./Misc/Src/bme280.d ./Misc/Src/bme280.o ./Misc/Src/bme280.su

.PHONY: clean-Misc-2f-Src

