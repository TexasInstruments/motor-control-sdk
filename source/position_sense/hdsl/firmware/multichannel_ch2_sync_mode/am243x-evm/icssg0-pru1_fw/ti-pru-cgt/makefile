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
	hdsl_master_icssg_txpru.cmd \

ASM_SRCS += \
	main.asm \
	datalink.asm \
	datalink_init.asm \
	transport.asm \
	utils.asm \

OBJS += \
	main.obj \
	datalink.obj \
	datalink_init.obj \
	transport.obj \
	utils.obj \

ASM_DEPS += \
	main.d \
	datalink.d \
	datalink_init.d \
	transport.d \
	utils.d \

OBJDIR := .

FILES_PATH_common = \
	.. \
	../../.. \
	../../../.. \

FILES_PATH := $(FILES_PATH_common)

vpath %.asm $(FILES_PATH)
vpath %.obj $(OBJDIR)
vpath %.cmd $(FILES_PATH)


# Add inputs and outputs from these tool invocations to the build variables
EXE_OUTPUTS += \
hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out \

EXE_OUTPUTS__QUOTED += \
"hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out" \

BIN_OUTPUTS += \
hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.hex \

BIN_OUTPUTS__QUOTED += \
"hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.hex" \

# All Target
all: $(OBJS) $(CMD_SRCS)
	@$(MAKE) --no-print-directory -Onone "hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out"

# Each subdirectory must supply rules for building sources it contributes
$(OBJDIR)/$(OBJS) %.obj: $(ASM_SRCS) 
	@echo 'Building file: "$^"'
	@echo 'Invoking: PRU Compiler'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DPRU1 -DSLICE1 --define=SOC_AM243X --define=PRU1 --define=CHANNEL_2 --define=ICSS_G_V_1_0 --define=SYNC_300_MHZ --define=HDSL_MULTICHANNEL --define=EXT_SYNC_ENABLE --include_path="$(SYSCFG_DIR)" --include_path="${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware"  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little --preproc_with_compile --preproc_dependency="$(basename $(<F)).d_raw" $^
	@echo 'Finished building: "$^"'
	@echo ' '

# Tool invocations
hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out: $(OBJS) $(CMD_SRCS)
	@echo 'Building target: "$@"'
	@echo 'Invoking: PRU Linker'
	"$(CGT_TI_PRU_PATH)/bin/clpru" -DICSSG0 -DPRU1 -DSLICE1 --define=SOC_AM243X --define=PRU1 --define=CHANNEL_2 --define=ICSS_G_V_1_0 --define=SYNC_300_MHZ --define=HDSL_MULTICHANNEL --define=EXT_SYNC_ENABLE  --define=_DEBUG_=1 -g --diag_warning=225 --diag_wrap=off --display_error_number --endian=little -z -m"hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.map" --disable_auto_rts   --diag_wrap=off --display_error_number --warn_sections --xml_link_info="hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt_linkInfo.xml" --disable_auto_rts --entry_point=main  --rom_model -o "hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out" $^	
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
		-$(CG_TOOL_ROOT)/bin/hexpru.exe --diag_wrap=off --array --array:name_prefix=Hiperface_DSL_SYNC2_0_TX_PRU -o hdsl_master_icssg_multichannel_ch2_sync_mode_bin.h hdsl_master_multichannel_ch2_sync_mode_am243x-evm_icssg0-pru1_fw_ti-pru-cgt.out
		-  $(COPY)  hdsl_master_icssg_multichannel_ch2_sync_mode_bin.h  ${MOTOR_CONTROL_SDK_PATH}/source/position_sense/hdsl/firmware/hdsl_master_icssg_multichannel_ch2_sync_mode_bin.h
	-@echo ' '

.PHONY: all clean
