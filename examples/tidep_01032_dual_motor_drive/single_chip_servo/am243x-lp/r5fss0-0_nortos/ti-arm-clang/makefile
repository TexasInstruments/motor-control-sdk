#
# Auto generated makefile
#

export MOTOR_CONTROL_SDK_PATH?=$(abspath ../../../../../..)
include $(MOTOR_CONTROL_SDK_PATH)/imports.mak
include $(MOTOR_CONTROL_SDK_PATH)/devconfig/devconfig.mak

CG_TOOL_ROOT=$(CGT_TI_ARM_CLANG_PATH)

CC=$(CG_TOOL_ROOT)/bin/tiarmclang
LNK=$(CG_TOOL_ROOT)/bin/tiarmclang
STRIP=$(CG_TOOL_ROOT)/bin/tiarmstrip
OBJCOPY=$(CG_TOOL_ROOT)/bin/tiarmobjcopy
ifeq ($(OS), Windows_NT)
	PYTHON=python
else
	PYTHON=python3
endif

PROFILE?=release
ConfigName:=$(PROFILE)

OUTNAME:=single_chip_servo.$(PROFILE).out

BOOTIMAGE_PATH=$(abspath .)
BOOTIMAGE_NAME:=single_chip_servo.$(PROFILE).appimage
BOOTIMAGE_NAME_XIP:=single_chip_servo.$(PROFILE).appimage_xip
BOOTIMAGE_NAME_SIGNED:=single_chip_servo.$(PROFILE).appimage.signed
BOOTIMAGE_RPRC_NAME:=single_chip_servo.$(PROFILE).rprc
BOOTIMAGE_RPRC_NAME_XIP:=single_chip_servo.$(PROFILE).rprc_xip
BOOTIMAGE_RPRC_NAME_TMP:=single_chip_servo.$(PROFILE).rprc_tmp

FILES_common := \
	endat_periodic_trigger.c \
	epwm_drv_aux.c \
	mclk_iep_sync.c \
	pwm.c \
	sddf.c \
	ti_r5fmath_trig.c \
	single_chip_servo.c \
	main.c \
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
	generated \

INCLUDES_common := \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source \
	-I${MOTOR_CONTROL_SDK_PATH}/source \
	-I${CG_TOOL_ROOT}/include/c \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/DSP/Include \
	-I${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/Core/Include \
	-I${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/firmware \
	-I${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/include \
	-I${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/firmware \
	-I${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/include \
	-I${MOTOR_CONTROL_SDK_PATH}/source/transforms/clarke \
	-I${MOTOR_CONTROL_SDK_PATH}/source/transforms/ipark \
	-I${MOTOR_CONTROL_SDK_PATH}/source/transforms/park \
	-I${MOTOR_CONTROL_SDK_PATH}/source/transforms/svgen \
	-Igenerated \

DEFINES_common := \
	-DSOC_AM243X \

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
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/lib \
	-Wl,-i${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/lib \
	-Wl,-i${CG_TOOL_ROOT}/lib \

LIBS_common = \
	-llibc.a \
	-lnortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-ldrivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lboard.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lmotorcontrol_sdfm.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lmotorcontrol_endat.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-lcmsis.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	-llibc.a \
	-llibsysbm.a \

LFLAGS_common = \
	-Wl,--diag_suppress=10063 \
	-Wl,--ram_model \
	-Wl,--reread_libs \


LIBS_NAME = \
	libc.a \
	nortos.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	drivers.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	board.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	motorcontrol_sdfm.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	motorcontrol_endat.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	cmsis.am243x.r5f.ti-arm-clang.${ConfigName}.lib \
	libc.a \
	libsysbm.a \

LIBS_PATH_NAME = \
	${CG_TOOL_ROOT}/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/kernel/nortos/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/drivers/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/board/lib \
	${MOTOR_CONTROL_SDK_PATH}/source/position_sense/endat/lib \
	${MOTOR_CONTROL_SDK_PATH}/source/current_sense/sdfm/lib \
	${MOTOR_CONTROL_SDK_PATH}/mcu_plus_sdk/source/cmsis/lib \
	${CG_TOOL_ROOT}/lib \

FILES := $(FILES_common) $(FILES_$(PROFILE))
ASMFILES := $(ASMFILES_common) $(ASMFILES_$(PROFILE))
FILES_PATH := $(FILES_PATH_common) $(FILES_PATH_$(PROFILE))
CFLAGS := $(CFLAGS_common) $(CFLAGS_$(PROFILE))
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
	$(CC) -c $(CFLAGS) -o $(OBJDIR)/$@ $<

all: $(BOOTIMAGE_NAME)

SYSCFG_GEN_FILES=generated/ti_drivers_config.c generated/ti_drivers_config.h
SYSCFG_GEN_FILES+=generated/ti_drivers_open_close.c generated/ti_drivers_open_close.h
SYSCFG_GEN_FILES+=generated/ti_dpl_config.c generated/ti_dpl_config.h
SYSCFG_GEN_FILES+=generated/ti_pinmux_config.c generated/ti_power_clock_config.c
SYSCFG_GEN_FILES+=generated/ti_board_config.c generated/ti_board_config.h
SYSCFG_GEN_FILES+=generated/ti_board_open_close.c generated/ti_board_open_close.h

$(OUTNAME): syscfg $(SYSCFG_GEN_FILES) $(OBJS) $(LNK_FILES) $(LIBS_NAME)
	@echo  .
	@echo  Linking: am243x:r5fss0-0:nortos:ti-arm-clang $@ ...
	$(LNK) $(LNKOPTFLAGS) $(LFLAGS) $(LIBS_PATH) -Wl,-m=$(basename $@).map -o $@ $(addprefix $(OBJDIR), $(OBJS)) $(LIBS) $(LNK_FILES)
	@echo  Linking: am243x:r5fss0-0:nortos:ti-arm-clang $@ Done !!!
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
	@echo  Scrubing: am243x:r5fss0-0:nortos:ti-arm-clang single_chip_servo ...
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

syscfg: ../example.syscfg
	@echo Generating SysConfig files ...
	$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/dist/cli.js --product $(SYSCFG_SDKPRODUCT) --context r5fss0-0 --part ALX --package ALX --output generated/ ../example.syscfg

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device AM243x_ALX_beta --context r5fss0-0 --part ALX --package ALX --output generated/  ../example.syscfg

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
	@echo  Boot image: am243x:r5fss0-0:nortos:ti-arm-clang $(BOOTIMAGE_PATH)/$@ Done !!!
	@echo  .

-include $(addprefix $(OBJDIR)/, $(DEPS))
