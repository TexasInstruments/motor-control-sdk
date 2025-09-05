
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

OUTNAME:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).out
COREOUTNAME:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).optishare.out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).appimage
BOOTIMAGE_NAME_XIP:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).appimage_xip
BOOTIMAGE_NAME_SIGNED:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).appimage.signed
BOOTIMAGE_RPRC_NAME:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).rprc
BOOTIMAGE_RPRC_NAME_XIP:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).rprc_xip
BOOTIMAGE_RPRC_NAME_TMP:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).rprc_tmp
BOOTIMAGE_NAME_HS:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).appimage.hs
BOOTIMAGE_NAME_HS_FS:=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).appimage.hs_fs
TARGETS := $(BOOTIMAGE_NAME)


FILES_common := \
	fwc.c \
	mtpa.c \
	pi.c \
	vs_freq.c \
	filter_fo.c \
	fastrts.c \
	fastrts_repl.c \
	main.c \
	encoder.c \
	esmo.c \
	hall.c \
	speedcalc.c \
	speedfr.c \
	svgen_current.c \
	volt_recons.c \
	angle_gen.c \
	datalog.c \
	step_response.c \
	traj.c \
	sys_main.c \
	app_epwm.c \
	hal.c \
	user_mtr1.c \
	motor_common.c \
	motor1_drive.c \
	ti_drivers_config.c \
	ti_drivers_open_close.c \
	ti_board_config.c \
	ti_board_open_close.c \
	ti_dpl_config.c \
	ti_pinmux_config.c \
	ti_power_clock_config.c \

FILES_PATH_common = \
	.. \
	../.. \
	../../.. \
	../../../.. \
	../../../src_board \
	../../../src_control \
	../../../../../../source \
	../../../../../../mcu_plus_sdk/ \
	../../../../../../mcu_plus_sdk/source \
	$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/control/fwc/source \
	../../../../../../source/rtlibs/control/mtpa/source \
	../../../../../../source/rtlibs/control/pi/source \
	../../../../../../source/rtlibs/control/vs_freq/source \
	../../../../../../source/rtlibs/control/filter/filter_fo/source \
	../../../../../../source/rtlibs/utilities \
	../../../../../../source/rtlibs/observers/encoder/source \
	../../../../../../source/rtlibs/observers/esmo/source \
	../../../../../../source/rtlibs/observers/est/source \
	../../../../../../source/rtlibs/observers/hall/source \
	../../../../../../source/rtlibs/observers/speedcalc/source \
	../../../../../../source/rtlibs/observers/speedfr/source \
	../../../../../../source/rtlibs/transforms/svgen \
	../../../../../../source/rtlibs/transforms/volts/source \
	../../../../../../source/rtlibs/utilities/angle_gen/source \
	../../../../../../source/rtlibs/utilities/datalog/source \
	../../../../../../source/rtlibs/utilities/step_response/source \
	../../../../../../source/rtlibs/utilities/traj/source \
	../../../../../../mcu_plus_sdk/source/mathlib/fastrts \
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source \
	-I${MOTOR_CONTROL_SDK_PATH}/source \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board \
	-I${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware \
	-I${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/include \
	-I${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware \
	-I${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/control/fwc/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/control/mtpa/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/control/pi/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/control/vs_freq/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/control/filter/filter_fo/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/observers/encoder/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/observers/esmo/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/observers/est/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/observers/hall/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/observers/speedcalc/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/observers/speedfr/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/transforms/volts/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/utilities/angle_gen/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/utilities/datalog/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/utilities/step_response/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/utilities/traj/include \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/dcl \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/utilities \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/transforms/clarke \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/transforms/ipark \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/transforms/park \
	-I$(MOTOR_CONTROL_SDK_PATH)/source/rtlibs/transforms/svgen \
	-I$(MOTOR_CONTROL_SDK_PATH)/examples/tidep_01032_dual_motor_drive/universal_single_chip_servo/src_board \
	-I$(MOTOR_CONTROL_SDK_PATH)/examples/tidep_01032_dual_motor_drive/universal_single_chip_servo/src_control \
	-I$(MOTOR_CONTROL_SDK_PATH)/examples/tidep_01032_dual_motor_drive/universal_single_chip_servo/am243x-lp-BP-AM2BLDCSERVO/r5fss0-0_nortos \
	-I$(MOTOR_CONTROL_SDK_PATH)/examples/tidep_01032_dual_motor_drive/universal_single_chip_servo \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM243X \
	-DSYSCONFIG_EN \
	-D_INLINE \
	-DBP_AM2BLDCSERVO \
	-DCLOSED_LOOP_CIA402 \
	-DMOTOR1_ABS_ENC \
	-DMOTOR1_ENC \
	-DMOTOR1_INLINE_SDFM \
	-DMOTOR2_ABS_ENC \
	-DMOTOR2_ENC \
	-DMOTOR2_INLINE_SDFM \
	-DHALL_CAL_N \
	-DMOTOR1_OVM_N \
	-DMOTOR1_FWC_N \
	-DMOTOR1_MTPA_N \
	-DMOTOR1_PI_TUNE_N \
	-DMOTOR1_RPM_CMD \
	-DMOTOR2_RPM_CMD \
	-DMOTOR1_FILTERIS_N \
	-DMOTOR1_FILTERVS_N \
	-DBRAKE_ENABLE_N \
	-DDATALOG_EN_N \
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
	-mcpu=cortex-r5 \
	-mfloat-abi=hard \
	-mfpu=vfpv3-d16 \
	-mthumb \
	-Wall \
	-Werror \
	-Wno-gnu-variable-sized-type-not-at-end \
	-Wno-unused-function \
	-O2 \

CFLAGS_debug := \
	-D_DEBUG_=1 \
	-g \

CFLAGS_cpp_common := \
	-Wno-c99-designator \
	-Wno-extern-c-compat \
	-Wno-c++11-narrowing \
	-Wno-reorder-init-list \
	-Wno-register \
	-Wno-writable-strings \
	-Wno-enum-compare \
	-Wno-reserved-user-defined-literal \
	-Wno-unused-const-variable \
	-Wno-vla-cxx-extension \
	-x c++ \

CFLAGS_release := \
	-Os \
	-ffast-math \
	-flto \
	-finline-functions \


LNK_FILES_common = \
	generated/linker.cmd \

LIBS_PATH_common = \
	-Wl,-i${CG_TOOL_ROOT}/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-llibc.a \
	-lnortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lboard.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lmotorcontrol_sdfm.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lmotorcontrol_endat.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-llibc.a \
	-llibsysbm.a \

LFLAGS_common = \
	-Wl,--diag_suppress=10063 \
	-Wl,--ram_model \
	-Wl,--reread_libs \
	-Wl,--gen_xml_func_hash \
	

LIBS_NAME = \
	libc.a \
	nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	board.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	motorcontrol_sdfm.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	motorcontrol_endat.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	libc.a \
	libsysbm.a \

LIBS_PATH_NAME = \
	${CG_TOOL_ROOT}/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib \
	${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/lib \
	${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/lib \
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
	@echo  Compiling: am243x:r5fss0-0:nortos:ti-arm-clang $(OUTNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -MMD -o $(OBJDIR)/$@ $<

$(OBJDIR)/%.obj %.obj: %.S
	@echo  Compiling: am243x:r5fss0-0:nortos:ti-arm-clang $(LIBNAME): $<
	$(CC) -c $(CFLAGS) $(INCLUDES) $(DEFINES) -o $(OBJDIR)/$@ $<

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
	@echo  Linking: am243x:r5fss0-0:nortos:ti-arm-clang $@ ...
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES) -Wl,--xml_link_info=$(basename $@).lnkxml
	@echo  Linking: am243x:r5fss0-0:nortos:ti-arm-clang $@ Done !!!
	@echo  .

coreout: $(COREOUTNAME)
$(COREOUTNAME):  $(SYSTEM_COMMAND)
	@echo  .
	@echo  Relinking Linking: am243x:r5fss0-0:nortos:ti-arm-clang $@ with OptiShare SSO..
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES) -Wl,--xml_link_info=$(basename $@).lnkxml -Wl,--import_sso=$(SSO_PATH)
	@echo  Relinking with optishare: am243x:r5fss0-0:freertos:ti-arm-clang $@ Done !!!
	@echo  .

clean:
	@echo  Cleaning: am243x:r5fss0-0:nortos:ti-arm-clang $(OUTNAME) ...
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
	@echo  Scrubing: am243x:r5fss0-0:nortos:ti-arm-clang universal_single_chip_servo ...
	$(RMDIR) obj
ifeq ($(OS),Windows_NT)
	$(RM) \*.out
	$(RM) \*.map
	$(RM) \*.appimage*
	$(RM) \*.rprc*
	$(RM) \*.tiimage*
	$(RM) \*.bin
	$(RM) \*.lnkxml
	$(RM) \*.ossr
else
	$(RM) *.out
	$(RM) *.map
	$(RM) *.appimage*
	$(RM) *.rprc*
	$(RM) *.tiimage*
	$(RM) *.bin
	$(RM) *.lnkxml
	$(RM) *.ossr
endif
	$(RMDIR) generated

$(OBJS): | $(OBJDIR)

$(OBJDIR):
	$(MKDIR) $@


.NOTPARALLEL:

.INTERMEDIATE: syscfg
$(SYSCFG_GEN_FILES): syscfg

ifeq ($(SYSTEM_FLAG), false)
syscfg: ../AM243_BP-AM2BLDCSERVO.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part ALX --package ALX --output generated/ ../AM243_BP-AM2BLDCSERVO.syscfg
endif

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM243x_ALX_beta --context r5fss0-0 --part ALX --package ALX --output generated/  ../AM243_BP-AM2BLDCSERVO.syscfg

#
# Generation of boot image which can be loaded by Secondary Boot Loader (SBL)
#
ifeq ($(OS),Windows_NT)
EXE_EXT=.exe
endif
ifeq ($(OS),Windows_NT)
  BOOTIMAGE_CERT_GEN_CMD=powershell -executionpolicy unrestricted -command $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/security/security_common/tools/boot/signing/x509CertificateGen.ps1
else
  BOOTIMAGE_CERT_GEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/security/security_common/tools/boot/signing/x509CertificateGen.sh
endif
BOOTIMAGE_TEMP_OUT_FILE=temp_stdout_$(PROFILE).txt

BOOTIMAGE_CERT_KEY=$(APP_SIGNING_KEY)

BOOTIMAGE_CORE_ID_r5fss0-0 = 4
BOOTIMAGE_CORE_ID_r5fss0-1 = 5
BOOTIMAGE_CORE_ID_r5fss1-0 = 6
BOOTIMAGE_CORE_ID_r5fss1-1 = 7
BOOTIMAGE_CORE_ID_m4fss0-0 = 14
SBL_RUN_ADDRESS=0x70000000
SBL_DEV_ID=55

MULTI_CORE_IMAGE_GEN = $(SYSCFG_NODE) $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/multicoreImageGen/multicoreImageGen.js
OUTRPRC_CMD = $(SYSCFG_NODE) $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/out2rprc/elf2rprc.js
APP_IMAGE_SIGN_CMD = $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/source/security/security_common/tools/boot/signing/appimage_x509_cert_gen.py

ifeq ($(OS),Windows_NT)
  XIPGEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/xipGen/xipGen.exe
else
  UNAME_S = $(shell uname -s)
  ifeq ($(UNAME_S), Darwin)
    XIPGEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/xipGen/xipGen.out.mac
  else
    XIPGEN_CMD=$(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/boot/xipGen/xipGen.out
endif
endif

MULTI_CORE_IMAGE_PARAMS = \
	$(BOOTIMAGE_RPRC_NAME)@$(BOOTIMAGE_CORE_ID_r5fss0-0) \

MULTI_CORE_IMAGE_PARAMS_XIP = \
	$(BOOTIMAGE_RPRC_NAME_XIP)@$(BOOTIMAGE_CORE_ID_r5fss0-0) \

$(BOOTIMAGE_NAME): $(OUTNAME)
	@echo  Boot image: am243x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ ...
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
ifeq ($(DEVICE_TYPE), HS)
# Sign the appimage using appimage signing script
ifeq ($(ENC_ENABLED),no)
	@echo Boot image signing: Encryption is disabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS)
else
	@echo Boot image signing: Encryption is enabled.
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --enc y --enckey $(APP_ENCRYPTION_KEY) --output $(BOOTIMAGE_NAME_HS)
	$(RM) $(BOOTIMAGE_NAME)-enc
endif
	$(RM) $(BOOTIMAGE_NAME)
	@echo  Boot image: am243x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS) Done !!!
	@echo  .
else
# Sign the appimage for HS-FS using appimage signing script
	$(PYTHON) $(APP_IMAGE_SIGN_CMD) --bin $(BOOTIMAGE_NAME) --authtype 1 --key $(APP_SIGNING_KEY) --output $(BOOTIMAGE_NAME_HS_FS)
	$(RM) $(BOOTIMAGE_NAME)
	@echo  Boot image: am243x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$(BOOTIMAGE_NAME_HS_FS) Done !!!
	@echo  .
endif



.PHONY: coverage
coverage:
	@echo Creating Coverage Report for universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE) ...
	$(MKDIR) coverage
	$(PROFDATA) merge -sparse -obj-file=$(OUTNAME) $(OUTNAME).cnt -o universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).profdata
	$(COV) show --format=html --show-expansions --show-instantiations --show-branches=count --object=$(OUTNAME) -instr-profile=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).profdata --output-dir=$(COVERAGE_PATH)/coverage --ignore-filename-regex=build_jenkins
	$(COV) export --format=text --object=$(OUTNAME) --instr-profile=universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).profdata > coverage/universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).profdata.json
	node $(MOTOR_CONTROL_SDK_PATH)/mcu_plus_sdk/tools/smart_placement/clang_coverage_analyse.js --input=coverage/universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).profdata.json --output-json=coverage/universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.$(PROFILE).analysis.json --output=../universal_single_chip_servo_am243x-lp_r5fss0-0_nortos_ti-arm-clang.annotations.$(PROFILE).S --top-function-count=500
	@echo Coverage Report Generated at $(COVERAGE_PATH)/coverage folder !!!

-include $(addprefix $(OBJDIR)/, $(DEPS))
