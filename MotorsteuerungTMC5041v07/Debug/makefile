################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
MotorsteuerungTMC5041v07.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
MotorsteuerungTMC5041v07.list \

OBJCOPY_BIN += \
MotorsteuerungTMC5041v07.bin \


# All Target
all: MotorsteuerungTMC5041v07.elf secondary-outputs

# Tool invocations
MotorsteuerungTMC5041v07.elf: $(OBJS) $(USER_OBJS) E:\OneDrive\arbeit\Elektronik\STM\CubeIDE\MotorsteuerungTMC5041v07\STM32F446RETX_FLASH.ld
	arm-none-eabi-gcc -o "MotorsteuerungTMC5041v07.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"E:\OneDrive\arbeit\Elektronik\STM\CubeIDE\MotorsteuerungTMC5041v07\STM32F446RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="MotorsteuerungTMC5041v07.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

MotorsteuerungTMC5041v07.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "MotorsteuerungTMC5041v07.list"
	@echo 'Finished building: $@'
	@echo ' '

MotorsteuerungTMC5041v07.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "MotorsteuerungTMC5041v07.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
