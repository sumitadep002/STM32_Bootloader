################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Bootloader_Cmd/BL_Cmd.c 

OBJS += \
./Bootloader_Cmd/BL_Cmd.o 

C_DEPS += \
./Bootloader_Cmd/BL_Cmd.d 


# Each subdirectory must supply rules for building sources it contributes
Bootloader_Cmd/%.o Bootloader_Cmd/%.su Bootloader_Cmd/%.cyclo: ../Bootloader_Cmd/%.c Bootloader_Cmd/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F334x8 -c -I../Core/Inc -I../Drivers/STM32F3xx_HAL_Driver/Inc -I"/home/scaledgetechnology/Desktop/GitHub/STM32_Bootloader/workspace/02_Bootloader/Bootloader_Cmd" -I../Drivers/STM32F3xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F3xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Bootloader_Cmd

clean-Bootloader_Cmd:
	-$(RM) ./Bootloader_Cmd/BL_Cmd.cyclo ./Bootloader_Cmd/BL_Cmd.d ./Bootloader_Cmd/BL_Cmd.o ./Bootloader_Cmd/BL_Cmd.su

.PHONY: clean-Bootloader_Cmd

