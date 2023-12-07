################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe
CCS_PATH?=C:/ti/ccs1200/ccs/ccs_base/pru/include
CG_TOOL_ROOT := C:/ti/ti-cgt-pru_2.3.3

#Set the default mode
CHANNELS ?= SINGLE

ifeq ($(CHANNELS),MULTI)
MULTI_CHANNEL=ENABLE_MULTI_CHANNEL
endif

ifeq ($(CHANNELS),SINGLE)
MULTI_CHANNEL=0
endif

GEN_OPTS__FLAG :=
GEN_CMDS__FLAG :=

ORDERED_OBJS += \
"bissc_main.obj" \
"bissc_diagnostic.cmd" \
$(GEN_CMDS__FLAG) \
-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

SUBDIRS := \
. \

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
bissc_diagnostic.cmd

ASM_SRCS += \
$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/position_sense/bissc/firmware/bissc_main.asm

OBJS += \
./bissc_main.obj

ASM_DEPS += \
./bissc_main.d

OBJS__QUOTED += \
"bissc_main.obj"

ASM_DEPS__QUOTED += \
"bissc_main.d"

ASM_SRCS__QUOTED += \
"$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/position_sense/bissc/firmware/bissc_main.asm"

# Each subdirectory must supply rules for building sources it contributes
bissc_main.obj: $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/position_sense/bissc/firmware/bissc_main.asm $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" --include_path="$(CCS_PATH)" --include_path="$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/position_sense/bissc/firmware" --include_path="$(CG_TOOL_ROOT)/include" --define=PRU1 --define=$(MULTI_CHANNEL) --define=icssg1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo '

USER_OBJS :=

LIBS := -llibc.a

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
bissc_master.out \

EXE_OUTPUTS__QUOTED += \
"bissc_master.out" \

BIN_OUTPUTS += \
bissc_master.hex \

BIN_OUTPUTS__QUOTED += \
"bissc_master.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "bissc_master.out"

# Tool invocations
bissc_master.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" --define=PRU1 --define=ENABLE_MULTI_CHANNEL --define=icssg1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"bissc_master.map" --heap_size=0 --stack_size=0 -i"$(CG_TOOL_ROOT)/lib" -i"$(CG_TOOL_ROOT)/include" --reread_libs --diag_wrap=off --display_error_number --warn_sections --xml_link_info="bissc_master_linkInfo.xml" -o "bissc_master.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

bissc_master.hex: $(EXE_OUTPUTS)
	@echo 'Building files: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru"  -o "bissc_master.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $(strip $(EXE_OUTPUTS__QUOTED))'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "bissc_main.obj"
	-$(RM) "bissc_main.d"
	-@echo 'Finished clean'
	-@echo ' '

post-build:
ifeq ($(CHANNELS),SINGLE)
	$(CG_TOOL_ROOT)/bin/hexpru.exe bissc_master_hexpru.cmd bissc_master.out && $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/bin2header/bin2header.exe bissc_master.b00 bissc_master_bin.h BiSSFirmware 4
endif
ifeq ($(CHANNELS),MULTI)
	$(CG_TOOL_ROOT)/bin/hexpru.exe bissc_master_hexpru.cmd bissc_master.out && $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/bin2header/bin2header.exe bissc_master.b00 bissc_master_multi_bin.h BiSSFirmwareMulti 4
endif
	-
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

