################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/MY_CS43L22.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/MY_CS43L22.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/MY_CS43L22.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/damit/Desktop/STM32F/STM32F407/STM32F4-Discovery_FW_V1.1.0/My Projects/Music/Inc" -I"C:/Users/damit/Desktop/STM32F/STM32F407/STM32F4-Discovery_FW_V1.1.0/My Projects/Music/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/damit/Desktop/STM32F/STM32F407/STM32F4-Discovery_FW_V1.1.0/My Projects/Music/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/damit/Desktop/STM32F/STM32F407/STM32F4-Discovery_FW_V1.1.0/My Projects/Music/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/damit/Desktop/STM32F/STM32F407/STM32F4-Discovery_FW_V1.1.0/My Projects/Music/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


