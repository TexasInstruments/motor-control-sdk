################################################################################
# Automatically-generated file. Do not edit!
################################################################################
#   Required input arguments :
#   CCS_PROJECT_DEBUG=${CWD}   Use this input argument to include files from external paths

export MOTOR_CONTROL_SDK_PATH?=$(abspath ../../../../../../../..)
include $(MOTOR_CONTROL_SDK_PATH)/imports.mak
SYSCFG_DIR := $(CCS_PROJECT_DEBUG)/syscfg
CG_TOOL_ROOT := $(CGT_TI_PRU_PATH)

# Add inputs and outputs from these tool invocations to the build variables
CMD_SRCS += \
	bissc_diagnostic.cmd \

ASM_SRCS += \
	bissc_main.asm \

OBJS += \
	bissc_main.obj \

ASM_DEPS += \
	bissc_main.d \

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
bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.hex" \

# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out"

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/%.obj %.obj: %.asm
	@echo 'Building file: "$^"'
	@echo 'Invoking: PRU Compiler'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DRTU_PRU1 -DSLICE1 -v4 -v4 --define=SOC_AM243X --define=ENABLE_MULTI_MAKE_RTU --include_path="${CG_TOOL_ROOT}/include" --include_path="${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source" --include_path="${MOTOR_CONTROL_SDK_PATH}/source" --include_path="${MOTOR_CONTROL_SDK_PATH}/source/position_sense/bissc/firmware"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $^
	@echo 'Finished building: "$^"'
	@echo ' '

# Tool invocations
bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DRTU_PRU1 -DSLICE1 -v4 -v4 --define=SOC_AM243X --define=ENABLE_MULTI_MAKE_RTU  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt_linkInfo.xml" --disable_auto_rts --diag_suppress=10063-D --entry_point=BISSC_INIT --disable_auto_rts  --rom_model -o "bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out" $^
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
		-$(CG_TOOL_ROOT)/bin/hexpru --diag_wrap=off --array --array:name_prefix=BiSSFirmwareMultiMakeRTU -o bissc_receiver_multi_RTU_bin.h  bissc_peripheral_interface_multi_ch_load_share_am243x-evm_icssg0-rtupru1_fw_ti-pru-cgt.out
		-$(CAT) ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/pru_io/firmware/pru_load_bin_copyright.h bissc_receiver_multi_RTU_bin.h > ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/bissc/firmware/bissc_receiver_multi_RTU_bin.h
		-$(RM)  bissc_receiver_multi_RTU_bin.h
	-@echo ' '

.PHONY: all clean
