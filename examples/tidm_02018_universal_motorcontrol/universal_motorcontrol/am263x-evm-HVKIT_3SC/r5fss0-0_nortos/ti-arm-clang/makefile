#
# Auto generated makefile
#

export MOTOR_CONTROL_SDK_PATH?=$(abspath ../../../../../..)
include $(MOTOR_CONTROL_SDK_PATH)/imports.mak
include $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)

CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy
COV=$(CG_TOOL_ROOT)/bin/tiarmcov
PROFDATA=$(CG_TOOL_ROOT)/bin/tiarmprofdata
COVERAGE_PATH=$(abspath .)
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).appimage
BOOTIMAGE_NAME_XIP:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).appimage_xip
BOOTIMAGE_NAME_SIGNED:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).appimage.signed
BOOTIMAGE_RPRC_NAME:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).rprc
BOOTIMAGE_RPRC_NAME_XIP:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).rprc_xip
BOOTIMAGE_RPRC_NAME_TMP:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).rprc_tmp
BOOTIMAGE_NAME_HS:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).appimage.hs
BOOTIMAGE_NAME_HS_FS:=universal_motorcontrol_HVKIT_3SC.$(PROFILE).appimage.hs_fs
TARGETS := $(BOOTIMAGE_NAME)

FILES_common := \
	main.c \
	sys_main.c \
	motor1_drive.c \
	motor_common.c \
	tca6416_config.c \
	mcan_transceiver.c \
	can_msg.c \
	user_mtr1.c \
	hal.c \
	traj.c \
	step_response.c \
	datalog.c \
	angle_gen.c \
	volt_recons.c \
	svgen_current.c \
	sfra.c \
	speedfr.c \
	speedcalc.c \
	hall.c \
	esmo.c \
	encoder.c \
	filter_fo.c \
	vs_freq.c \
	pi.c \
	mtpa.c \
	fwc.c \
	fastrts.c \
	fastrts_repl.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../../.. \
	../../../sfra_setting \
	../../../src_board \
	../../../src_control \
	../../../src_control/CAN \
	${MOTOR_CONTROL_SDK_PATH}/source/utilities/traj/source \
	${MOTOR_CONTROL_SDK_PATH}/source/utilities/step_response/source \
	${MOTOR_CONTROL_SDK_PATH}/source/utilities/datalog/source \
	${MOTOR_CONTROL_SDK_PATH}/source/utilities/angle_gen/source \
	${MOTOR_CONTROL_SDK_PATH}/source/transforms/volts/source \
	${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen \
	${MOTOR_CONTROL_SDK_PATH}/source/observers/speedfr/source \
	${MOTOR_CONTROL_SDK_PATH}/source/observers/speedcalc/source \
	${MOTOR_CONTROL_SDK_PATH}/source/observers/hall/source \
	${MOTOR_CONTROL_SDK_PATH}/source/observers/esmo/source \
	${MOTOR_CONTROL_SDK_PATH}/source/observers/encoder/source \
	${MOTOR_CONTROL_SDK_PATH}/source/control/filter/filter_fo/source \
	${MOTOR_CONTROL_SDK_PATH}/source/control/vs_freq/source \
	${MOTOR_CONTROL_SDK_PATH}/source/control/pi/source \
	${MOTOR_CONTROL_SDK_PATH}/source/control/mtpa/source \
	${MOTOR_CONTROL_SDK_PATH}/source/control/fwc/source \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/mathlib/fastrts \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source \
	-I${MOTOR_CONTROL_SDK_PATH}/source \
	-I${CG_TOOL_ROOT}/include/c \
    -I${MOTOR_CONTROL_SDK_PATH}/source/control/fwc/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/control/mtpa/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/control/pi/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/control/vs_freq/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/control/filter/filter_fo/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/dcl \
    -I${MOTOR_CONTROL_SDK_PATH}/source/observers/encoder/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/observers/esmo/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/observers/est/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/observers/hall/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/observers/speedfr/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/observers/speedcalc/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/sfra \
    -I${MOTOR_CONTROL_SDK_PATH}/source/transforms/clarke \
    -I${MOTOR_CONTROL_SDK_PATH}/source/transforms/ipark \
    -I${MOTOR_CONTROL_SDK_PATH}/source/transforms/park \
    -I${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen \
    -I${MOTOR_CONTROL_SDK_PATH}/source/transforms/volts/include \
	-I${MOTOR_CONTROL_SDK_PATH}/source/utilities \
    -I${MOTOR_CONTROL_SDK_PATH}/source/utilities/angle_gen/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/utilities/datalog/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/utilities/step_response/include \
    -I${MOTOR_CONTROL_SDK_PATH}/source/utilities/traj/include \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/mathlib/fastrts \
	-I${MOTOR_CONTROL_SDK_PATH}/examples/tidm_02018_universal_motorcontrol/universal_motorcontrol \
    -I${MOTOR_CONTROL_SDK_PATH}/examples/tidm_02018_universal_motorcontrol/universal_motorcontrol/src_board \
    -I${MOTOR_CONTROL_SDK_PATH}/examples/tidm_02018_universal_motorcontrol/universal_motorcontrol/src_control \
    -I${MOTOR_CONTROL_SDK_PATH}/examples/tidm_02018_universal_motorcontrol/universal_motorcontrol/src_control/CAN \
    -I${MOTOR_CONTROL_SDK_PATH}/examples/tidm_02018_universal_motorcontrol/universal_motorcontrol/src_control/PWMDAC \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM263X \
	-DSYSCONFIG_EN \
	-D_INLINE \
	-D_AM263x \
	-DAM263_CC \
	-DHVMTRPFC_REV1P1 \
	-DMOTOR1_ESMO \
	-DMOTOR1_ENC \
	-DMOTOR1_HALL_N \
	-DHALL_CAL \
	-DMOTOR1_OVM_N \
	-DMOTOR1_FWC_N \
	-DMOTOR1_MTPA_N \
	-DMOTOR1_PI_TUNE_N \
	-DMOTOR1_RPM_CMD_N \
	-DMOTOR1_FILTERIS_N \
	-DMOTOR1_FILTERVS_N \
	-DBRAKE_ENABLE_N \
	-DDATALOG_EN \
	-DSFRA_ENABLE_N \
	-DSTEP_RP_EN_N \
	-DCPUTIME_ENABLE_N \
	-DEPWMDAC_MODE_N \
	-DCMD_CAN_N \

CFLAGS_common := \
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-mthumb \
	-Wall \
	-Werror \
	-g \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-deprecated-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-x c++ \

CFLAGS_debug := \
	-D_DEBUG_=1 \

CFLAGS_release := \
	-Os \

LNK_FILES_common = \
	linker.cmd \

LIBS_PATH_common = \
	-Wl,-i${CG_TOOL_ROOT}/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/source/sfra/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-lnortos.am263x.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lboard.am263x.r5f.ti-arm-clang.${ConfigName}.lib \
	-llibc.a \
	-llibsysbm.a \
	-lsfra.am263x.r5f.ti-arm-clang.${ConfigName}.lib \

LFLAGS_common = \
	-Wl,--diag_suppress=10063 \
	-Wl,--ram_model \
	-Wl,--reread_libs \


LIBS_NAME = \
	nortos.am263x.r5f.ti-arm-clang.${ConfigName}.lib \
	drivers.am263x.r5f.ti-arm-clang.${ConfigName}.lib \
	board.am263x.r5f.ti-arm-clang.${ConfigName}.lib \
	libc.a \
	libsysbm.a \
	sfra.am263x.r5f.ti-arm-clang.${ConfigName}.lib \

LIBS_PATH_NAME = \
	${CG_TOOL_ROOT}/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib \
	${MOTOR_CONTROL_SDK_PATH}/source/sfra/lib \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
ifeq ($(INSTRUMENTATION_MODE), yes)
CFLAGS += -fprofile-instr-generate -fcoverage-mapping
endif
DEFINES := $(DEFINES_common) $(DEFINES_$(PROFILE))
INCLUDES := $(INCLUDES_common) $(INCLUDE_$(PROFILE))
LIBS := $(LIBS_common) $(LIBS_$(PROFILE))
LIBS_PATH := $(LIBS_PATH_common) $(LIBS_PATH_$(PROFILE))
LFLAGS := $(LFLAGS_common) $(LFLAGS_$(PROFILE))
LNKOPTFLAGS := $(LNKOPTFLAGS_common) $(LNKOPTFLAGS_$(PROFILE))
LNK_FILES := $(LNK_FILES_common) $(LNK_FILES_$(PROFILE))

OBJDIR := obj/$(PROFILE)/
OBJS := $(FILES:%.c=%.obj)
OBJS += $(ASMFILES:%.S=%.obj)
DEPS := $(FILES:%.c=%.d)

vpath %.obj $(OBJDIR)
vpath %.c $(FILES_PATH)
vpath %.S $(FILES_PATH)
vpath %.lib $(LIBS_PATH_NAME)
vpath %.a $(LIBS_PATH_NAME)

$(OBJDIR)/%.obj %.obj: %.c
	@echo  Compiling: am263x:r5fss0-0:nortos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am263x:r5fss0-0:nortos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) -o $(OBJDIR)/$@ $<

all: $(TARGETS)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

SYSTEM_FLAG ?= false

ifeq ($(SYSTEM_FLAG), false)
	SYSTEM_COMMAND := syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
else
	SYSTEM_COMMAND := $(OBJS) $(LNK_FILES) $(LIBS_NAME)
endif

$(OUTNAME):  $(SYSTEM_COMMAND)
	@echo  .
	@echo  Linking: am263x:r5fss0-0:nortos:ti-arm-clang $@ ...
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	@echo  Linking: am263x:r5fss0-0:nortos:ti-arm-clang $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: am263x:r5fss0-0:nortos:ti-arm-clang $(OUTNAME) ...
	$(RMDIR) $(OBJDIR)
	$(RM) $(OUTNAME)
	$(RM) $(BOOTIMAGE_NAME)
	$(RM) $(BOOTIMAGE_NAME_XIP)
	$(RM) $(BOOTIMAGE_NAME_SIGNED)
	$(RM) $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(RM) $(BOOTIMAGE_RPRC_NAME_XIP)
	$(RMDIR) generated/

scrub:
	@echo  Scrubing: am263x:r5fss0-0:nortos:ti-arm-clang universal_motorcontrol_HVKIT_3SC ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
endif
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@


.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

syscfg: ../AM263_HVMTRPFC.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part AM263x --package ZCZ --output generated/ ../AM263_HVMTRPFC.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM263x_beta --context r5fss0-0 --part AM263x --package ZCZ --output generated/  ../AM263_HVMTRPFC.syscfg

#
# Generation of boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
ifeq ($(OS),Windows_NT)
  BOOTIMAGE_CERT_GEN_CMD=powershell -executionpolicy unrestricted -command $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/signing/x509CertificateGen.ps1
else
  BOOTIMAGE_CERT_GEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/signing/x509CertificateGen.sh
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt


BOOTIMAGE_CORE_ID_r5fss0-0 = 0
BOOTIMAGE_CORE_ID_r5fss0-1 = 1
BOOTIMAGE_CORE_ID_r5fss1-0 = 2
BOOTIMAGE_CORE_ID_r5fss1-1 = 3
SBL_RUN_ADDRESS=0x70002000
SBL_DEV_ID=55

MULTI_CORE_IMAGE_GEN = $(SYSCFG_NODE) $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/multicoreImageGen/multicoreImageGen.js
OUTRPRC_CMD = $(SYSCFG_NODE) $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/out2rprc/elf2rprc.js
APP_IMAGE_SIGN_CMD = $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/signing/mcu_appimage_x509_cert_gen.py

ifeq ($(OS),Windows_NT)
  XIPGEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/xipGen/xipGen.exe
else
  XIPGEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/xipGen/xipGen.out
endif

MULTI_CORE_IMAGE_PARAMS = \
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_r5fss0-0) \

MULTI_CORE_IMAGE_PARAMS_XIP = \
	$(BOOTIMAGE_RPRC_NAME_XIP)@$(BOOTIMAGE_CORE_ID_r5fss0-0) \

$(BOOTIMAGE_NAME): $(OUTNAME)
	@echo  Boot image: am263x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ ...
ifneq ($(OS),Windows_NT)
	$(CHMOD) a+x $(XIPGEN_CMD)
endif
	$(OUTRPRC_CMD) $(OUTNAME) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(COPY) $(BOOTIMAGE_RPRC_NAME) $(BOOTIMAGE_RPRC_NAME_TMP)
	$(RM) $(BOOTIMAGE_RPRC_NAME)
	$(XIPGEN_CMD) -i $(BOOTIMAGE_RPRC_NAME_TMP) -o $(BOOTIMAGE_RPRC_NAME) -x $(BOOTIMAGE_RPRC_NAME_XIP) --flash-start-addr 0x60000000 -v > $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(BOOTIMAGE_NAME) $(MULTI_CORE_IMAGE_PARAMS) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(MULTI_CORE_IMAGE_GEN) --devID $(SBL_DEV_ID) --out $(BOOTIMAGE_NAME_XIP) $(MULTI_CORE_IMAGE_PARAMS_XIP) >> $(BOOTIMAGE_TEMP_OUT_FILE)
	$(RM) $(BOOTIMAGE_RPRC_NAME_TMP)
	$(RM) $(BOOTIMAGE_TEMP_OUT_FILE)
	@echo  Boot image: am263x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .

$(BOOTIMAGE_NAME_HS): $(BOOTIMAGE_NAME)
ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
ifeq ($(RSASSAPSS_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --key $(APP_SIGNING_KEY) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --output $(BOOTIMAGE_NAME_HS)
else
	@echo Boot image signing: Encryption is disabled. RSASSAPSS is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --key $(APP_SIGNING_KEY) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --pss_saltlen $(APP_SIGNING_SALT_LENGTH) --output $(BOOTIMAGE_NAME_HS) --rsassa_pss
endif
else
ifeq ($(RSASSAPSS_ENABLED),no)
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --kd-salt $(KD_SALT) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --enc_key_id $(APP_ENCRYPTION_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --output $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME)-enc
else
	@echo Boot image signing: Encryption is enabled. RSASSAPSS is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --kd-salt $(KD_SALT) --sign_key_id $(APP_SIGNING_KEY_KEYRING_ID) --enc_key_id $(APP_ENCRYPTION_KEY_KEYRING_ID) --hash_algo $(APP_SIGNING_HASH_ALGO) --pss_saltlen $(APP_SIGNING_SALT_LENGTH) --output $(BOOTIMAGE_NAME_HS) --rsassa_pss
	$(RM) $(BOOTIMAGE_NAME)-enc
endif
endif
	@echo  Boot image: am263x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS) Done !!!
	@echo  .
endif
.PHONY: coverage
coverage:
	@echo Creating Coverage Report for universal_motorcontrol_HVKIT_3SC.$(PROFILE) ...
	$(MKDIR) coverage
	$(PROFDATA) merge -sparse -obj-file=$(OUTNAME) $(OUTNAME).cnt -o universal_motorcontrol_HVKIT_3SC.$(PROFILE).profdata
	$(COV) show --format=html --show-expansions --show-instantiations --show-branches=count --object=$(OUTNAME) -instr-profile=universal_motorcontrol_HVKIT_3SC.$(PROFILE).profdata --output-dir=$(COVERAGE_PATH)/coverage --ignore-filename-regex=build_jenkins
	$(COV) export --format=text --object=$(OUTNAME) --instr-profile=universal_motorcontrol_HVKIT_3SC.$(PROFILE).profdata > coverage/universal_motorcontrol_HVKIT_3SC.$(PROFILE).profdata.json
	node $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/smart_placement/clang_coverage_analyse.js --input=coverage/universal_motorcontrol_HVKIT_3SC.$(PROFILE).profdata.json --output-json=coverage/universal_motorcontrol_HVKIT_3SC.$(PROFILE).analysis.json --output=../universal_motorcontrol_HVKIT_3SC.annotations.$(PROFILE).S --top-function-count=500
	@echo Coverage Report Generated at $(COVERAGE_PATH)/coverage folder !!!

-include $(addprefix $(OBJDIR)/, $(DEPS))
