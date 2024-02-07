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
	tamagawa_diagnostic.cmd \
	tamagawa_master_hexpru.cmd \

ASM_SRCS += \
	tamagawa_main.asm \

OBJS += \
	tamagawa_main.obj \

ASM_DEPS += \
	tamagawa_main.d \

OBJDIR := .

FILES_PATH_common = \
	.. \
	../../../.. \

FILES_PATH := $(FILES_PATH_common)

vpath %.asm $(FILES_PATH)
vpath %.obj $(OBJDIR)
vpath %.cmd $(FILES_PATH)


# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.hex" \

# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out"

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/$(OBJS) %.obj: $(ASM_SRCS) 
	@echo 'Building file: "$^"'
	@echo 'Invoking: PRU Compiler'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DPRU1 -DSLICE1 -v4 --define=SOC_AM64X --include_path="$(SYSCFG_DIR)" --include_path="${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $^
	@echo 'Finished building: "$^"'
	@echo ' '

# Tool invocations
tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DPRU1 -DSLICE1 -v4 --define=SOC_AM64X  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt_linkInfo.xml" --entry_point=TAMAGAWA_INIT --disable_auto_rts  --rom_model -o "tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out" $^	
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
		-$(CG_TOOL_ROOT)/bin/hexpru.exe ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_hexpru.cmd tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.out
		- ${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/tools/bin2header/bin2header.exe tamagawa_single_channel_am64x-evm_icssg0-pru1_fw_ti-pru-cgt.b00 tamagawa_master_single_channel_bin.h TamagawaFirmware 4
		- $(COPY) tamagawa_master_single_channel_bin.h ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/tamagawa/firmware/tamagawa_master_single_channel_bin.h 
	-@echo ' '

.PHONY: all clean
