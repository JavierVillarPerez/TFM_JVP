################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Drivers/BSP/X-NUCLEO-IDS01Ax/Src/subdir.mk
-include Drivers/BSP/STM32F4xx_Nucleo/subdir.mk
-include Drivers/BSP/Components/spirit1/SPIRIT1_Util/Src/subdir.mk
-include Drivers/BSP/Components/spirit1/SPIRIT1_Library/Src/subdir.mk
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
SDR_eval_board_F4.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
SDR_eval_board_F4.list \

OBJCOPY_BIN += \
SDR_eval_board_F4.bin \


# All Target
all: SDR_eval_board_F4.elf secondary-outputs

# Tool invocations
SDR_eval_board_F4.elf: $(OBJS) $(USER_OBJS) D:\Usuarios\Documents\Universidad\TFM\SDR_Mutliband_SW\TFM_JVP\SDR_eval_board_F4\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "SDR_eval_board_F4.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"D:\Usuarios\Documents\Universidad\TFM\SDR_Mutliband_SW\TFM_JVP\SDR_eval_board_F4\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="SDR_eval_board_F4.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

SDR_eval_board_F4.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "SDR_eval_board_F4.list"
	@echo 'Finished building: $@'
	@echo ' '

SDR_eval_board_F4.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "SDR_eval_board_F4.bin"
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
