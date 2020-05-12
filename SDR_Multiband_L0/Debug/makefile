################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Driver/STM32L0xx_HAL_Driver/Src/subdir.mk
-include Driver/CMSIS/Include/subdir.mk
-include Driver/BSP/X-NUCLEO-IDS01Ax/Src/subdir.mk
-include Driver/BSP/STM32L0xx_Nucleo/subdir.mk
-include Driver/BSP/Components/spirit1/SPIRIT1_Util/Src/subdir.mk
-include Driver/BSP/Components/spirit1/SPIRIT1_Library/Src/subdir.mk
-include Application/USER/subdir.mk
-include Application/SW4STM32/subdir.mk
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
SDR_Multiband_L0.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SDR_Multiband_L0.list \

OBJCOPY_BIN += \
SDR_Multiband_L0.bin \


# All Target
all: SDR_Multiband_L0.elf secondary-outputs

# Tool invocations
SDR_Multiband_L0.elf: $(OBJS) $(USER_OBJS) ../STM32L053R8Tx_FLASH.ld
	arm-none-eabi-gcc -o "SDR_Multiband_L0.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0plus -T"../STM32L053R8Tx_FLASH.ld" --specs=nosys.specs -Wl,-Map="SDR_Multiband_L0.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SDR_Multiband_L0.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SDR_Multiband_L0.list"
	@echo 'Finished building: $@'
	@echo ' '

SDR_Multiband_L0.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SDR_Multiband_L0.bin"
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
