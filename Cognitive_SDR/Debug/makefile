################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/X-NUCLEO-S2868A1/Src/subdir.mk
-include Drivers/BSP/Components/S2LP/S2LP_Library/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
Cognitive_SDR.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Cognitive_SDR.list \

OBJCOPY_BIN += \
Cognitive_SDR.bin \


# All Target
all: Cognitive_SDR.elf secondary-outputs

# Tool invocations
Cognitive_SDR.elf: $(OBJS) $(USER_OBJS) D:\Usuarios\Documents\Universidad\TFM\Cognitive_SDR\STM32F401RBTX_FLASH.ld
	arm-none-eabi-gcc -o "Cognitive_SDR.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Usuarios\Documents\Universidad\TFM\Cognitive_SDR\STM32F401RBTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Cognitive_SDR.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Cognitive_SDR.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Cognitive_SDR.list"
	@echo 'Finished building: $@'
	@echo ' '

Cognitive_SDR.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "Cognitive_SDR.bin"
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
