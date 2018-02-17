################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dsp.c \
../src/main.c \
../src/stm32f7xx_hal_msp.c \
../src/stm32f7xx_it.c 

OBJS += \
./src/dsp.o \
./src/main.o \
./src/stm32f7xx_hal_msp.o \
./src/stm32f7xx_it.o 

C_DEPS += \
./src/dsp.d \
./src/main.d \
./src/stm32f7xx_hal_msp.d \
./src/stm32f7xx_it.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DARM_MATH_CM7 -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DOS_USE_SEMIHOSTING -DSTM32F746xx -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f7xx" -I"../system/include/cmsis/device" -I"C:\STM32ToolChain\CMSIS\ARM.CMSIS.5.2.0\CMSIS\Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

src/main.o: ../src/main.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM Cross C Compiler'
	arm-none-eabi-gcc -mcpu=cortex-m7 -mthumb -Og -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-move-loop-invariants -Wall -Wextra  -g3 -DDEBUG -DOS_USE_SEMIHOSTING -DTRACE -DOS_USE_TRACE_SEMIHOSTING_DEBUG -DSTM32F746xx -I"../include" -I"../system/include" -I"../system/include/cmsis" -I"../system/include/stm32f7xx" -I"../system/include/cmsis/device" -I"C:\STM32ToolChain\CMSIS\ARM.CMSIS.5.2.0\CMSIS\Include" -std=gnu11 -MMD -MP -MF"$(@:%.o=%.d)" -MT"src/main.d" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


