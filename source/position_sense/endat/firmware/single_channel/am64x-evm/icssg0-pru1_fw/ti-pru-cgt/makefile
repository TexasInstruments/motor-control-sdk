################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe
export MOTOR_CONTROL_SDK_PATH?=$(abspath ../../../../../../../..)
CCS_PATH?=$(CCS_INSTALL_DIR)/ccs_base/pru/include
CG_TOOL_ROOT := C:/ti/ti-cgt-pru_2.3.3
SYSCFG_DIR := $(CCS_PROJECT_DEBUG)/syscfg

GEN_OPTS__FLAG :=
GEN_CMDS__FLAG :=

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
linker.cmd \

ASM_SRCS += \
main.asm

OBJS += \
main.obj

ASM_DEPS += \
main.d

OBJDIR := .

FILES_PATH_common = \
	.. \
	../../.. \
	../../../.. \

FILES_PATH := $(FILES_PATH_common)

vpath %.asm $(FILES_PATH)
vpath %.obj $(OBJDIR)
vpath %.cmd $(FILES_PATH)

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/$(OBJS): $(ASM_SRCS) 
	@echo 'Building file: "$<"'
	@echo 'Invoking: PRU Compiler'
	"$(CG_TOOL_ROOT)/bin/clpru" -DICSSG0 -DPRU1 -DSLICE1 -v4 --define=SOC_AM64X --include_path="$(CCS_PATH)" --include_path="$(SYSCFG_DIR)" --include_path="${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $(GEN_OPTS__FLAG) "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.hex" \


# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out"

# Tool invocations
endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CG_TOOL_ROOT)/bin/clpru" -DICSSG0 -DPRU1 -DSLICE1 -v4 --define=SOC_AM64X  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt_linkInfo.xml" --rom_model -o "endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out" $^
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.hex: $(EXE_OUTPUTS)
	@echo 'Building secondary target: "$@"'
	@echo 'Invoking: PRU Hex Utility'
	"$(CG_TOOL_ROOT)/bin/hexpru" -o "endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building secondary target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# Other Targets
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "main.obj"
	-$(RM) "main.d"
	-@echo 'Finished clean'
	-@echo ' '

post-build:
		-$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=EnDatFirmware -o endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.h endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out
		-  move endat_peripheral_interface_single_ch_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware/endat_master_bin.h
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
