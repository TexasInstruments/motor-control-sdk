################################################################################
# Automatically-generated file. Do not edit!
################################################################################
#   Required input arguments :
#   CCS_PROJECT_DEBUG=${CWD}   Use this input argument to include files from external paths 

export MOTOR_CONTROL_SDK_PATH?=$(abspath ../../../../../../../..)
include $(MOTOR_CONTROL_SDK_PATH)/imports.mak
SYSCFG_DIR := $(CCS_PROJECT_DEBUG)/syscfg
CG_TOOL_ROOT := $(CGT_TI_PRU_PATH)
RM := DEL /F
RMDIR := RMDIR /S/Q

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
	icssg_pru.cmd \

ASM_SRCS += \
	sdfm.asm \

OBJS += \
	sdfm.obj \

ASM_DEPS += \
	sdfm.d \

OBJDIR := .

FILES_PATH_common = \
	.. \
	../.. \
	../../../.. \

FILES_PATH := $(FILES_PATH_common)

vpath %.asm $(FILES_PATH)
vpath %.obj $(OBJDIR)
vpath %.cmd $(FILES_PATH)


# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.hex" \

# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out"

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/$(OBJS) %.obj: $(ASM_SRCS) 
	@echo 'Building file: "$^"'
	@echo 'Invoking: PRU Compiler'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DPRU0 -DSLICE0 -v4 -v4 -o2 --display_error_number --hardware_mac=on --define=SOC_AM243X --define=SDFM_PRU_CORE --define=SDFM_LOAD_SHARE_MODE --include_path="$(SYSCFG_DIR)" --include_path="${CG_TOOL_ROOT}/include" --include_path="${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source" --include_path="${MOTOR_CONTROL_SDK_PATH}/source" --include_path="${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $^
	@echo 'Finished building: "$^"'
	@echo ' '

# Tool invocations
sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DPRU0 -DSLICE0 -v4 -v4 -o2 --display_error_number --hardware_mac=on --define=SOC_AM243X --define=SDFM_PRU_CORE --define=SDFM_LOAD_SHARE_MODE  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt_linkInfo.xml" --warn_sections --entry_point=SDFM_ENTRY --zero_init=off --disable_auto_rts --define=SDFM_LOAD_SHARE_MODE=1  --rom_model -o "sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out" $^	
	@echo 'Finished building target: "$@"'
	@echo ' '
	@$(MAKE) --no-print-directory post-build

# To clean generated files
clean:
	-$(RM) $(BIN_OUTPUTS__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) $(OBJS)
	-$(RM) $(ASM_DEPS)
	-@echo 'Finished clean'
	-@echo ' '

post-build:
		-$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=pru_SDFM_PRU0_image -o sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.h sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.out
		-  move sdfm_firmware_multi_axis_load_share_am243x-evm_icssg0-pru0_fw_ti-pru-cgt.h ${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware/sdfm_pru_bin.h
	-@echo ' '

.PHONY: all clean
