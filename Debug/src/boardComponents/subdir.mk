################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/boardComponents/cs43l22.c \
../src/boardComponents/lis302dl.c \
../src/boardComponents/lis3dsh.c 

OBJS += \
./src/boardComponents/cs43l22.o \
./src/boardComponents/lis302dl.o \
./src/boardComponents/lis3dsh.o 

C_DEPS += \
./src/boardComponents/cs43l22.d \
./src/boardComponents/lis302dl.d \
./src/boardComponents/lis3dsh.d 


# Each subdirectory must supply rules for building sources it contributes
src/boardComponents/%.o: ../src/boardComponents/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross ARM C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mlittle-endian -mfloat-abi=hard -mfpu=fpv4-sp-d16 -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -ffreestanding -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DUSE_FULL_ASSERT -DTRACE -DOS_USE_TRACE_ITM -DSTM32F407xx -DUSE_HAL_DRIVER -DHSE_VALUE=8000000 -I"../include" -I"../system/include" -I"../system/include/CMSIS" -I"../system/include/stm32f4-hal" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


