################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/X-NUCLEO-IDS01Ax/Src/subdir.mk
-include Drivers/BSP/STM32L0xx_Nucleo/subdir.mk
-include Drivers/BSP/Components/spirit1/SPIRIT1_Util/Src/subdir.mk
-include Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
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
radio_eval.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
radio_eval.list \

OBJCOPY_BIN += \
radio_eval.bin \


# All Target
all: radio_eval.elf secondary-outputs

# Tool invocations
radio_eval.elf: $(OBJS) $(USER_OBJS) D:\Usuarios\Documents\Universidad\TFM\SDR_Mutliband_SW\TFM_JVP\radio_eval\STM32L053R8TX_FLASH.ld
	arm-none-eabi-gcc -o "radio_eval.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"D:\Usuarios\Documents\Universidad\TFM\SDR_Mutliband_SW\TFM_JVP\radio_eval\STM32L053R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="radio_eval.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

radio_eval.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "radio_eval.list"
	@echo 'Finished building: $@'
	@echo ' '

radio_eval.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "radio_eval.bin"
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
