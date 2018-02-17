################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../system/src/cortexm/_initialize_hardware.c \
../system/src/cortexm/_reset_hardware.c \
../system/src/cortexm/exception_handlers.c 

OBJS += \
./system/src/cortexm/_initialize_hardware.o \
./system/src/cortexm/_reset_hardware.o \
./system/src/cortexm/exception_handlers.o 

C_DEPS += \
./system/src/cortexm/_initialize_hardware.d \
./system/src/cortexm/_reset_hardware.d \
./system/src/cortexm/exception_handlers.d 


# Each subdirectory must supply rules for building sources it contributes
system/src/cortexm/%.o: ../system/src/cortexm/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DARM_MATH_CM7 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DOS_USE_SEMIHOSTING -DSTM32F746xx -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f7xx" -I"../system/include/cmsis/device" -I"C:\STM32ToolChain\CMSIS\ARM.CMSIS.5.2.0\CMSIS\Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


