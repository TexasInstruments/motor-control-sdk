MOTOR_CONTROL_SDK_PATH?=$(abspath .)
include imports.mak

# Default device
DEVICE ?= am64x

# debug, release
PROFILE?=release

# GP, HS
DEVICE_TYPE?=GP

ifeq ($(DEVICE),$(filter $(DEVICE), am64x))
  SYSCFG_DEVICE = AM64x_beta
  # default syscfg CPU to use,
  # options on am64x are r5fss0-0, r5fss0-1, r5fss1-0, r5fss1-1, m4fss0-0
  SYSCFG_CPU = r5fss0-0
endif
ifeq ($(DEVICE),$(filter $(DEVICE), am243x))
  SYSCFG_DEVICE = AM243x_ALV_beta
  # default syscfg CPU to use,
  # options on am64x are r5fss0-0, r5fss0-1, r5fss1-0, r5fss1-1, m4fss0-0
  SYSCFG_CPU = r5fss0-0
endif
ifeq ($(DEVICE),$(filter $(DEVICE), am263x))
  SYSCFG_DEVICE = AM263x_beta
  # default syscfg CPU to use,
  # options on am263x are r5fss0-0, r5fss0-1, r5fss1-0, r5fss1-1
  SYSCFG_CPU = r5fss0-0
endif

all:
	$(MAKE) -C . -f makefile.$(DEVICE) all PROFILE=$(PROFILE)

clean:
	$(MAKE) -C . -f makefile.$(DEVICE) clean PROFILE=$(PROFILE)

scrub:
	$(MAKE) -C . -f makefile.$(DEVICE) scrub PROFILE=$(PROFILE)

libs:
	$(MAKE) -C . -f makefile.$(DEVICE) libs PROFILE=$(PROFILE) DEVICE_TYPE=$(DEVICE_TYPE)

libs-clean:
	$(MAKE) -C . -f makefile.$(DEVICE) libs-clean PROFILE=$(PROFILE)

libs-scrub:
	$(MAKE) -C . -f makefile.$(DEVICE) libs-scrub PROFILE=$(PROFILE)

examples:
	$(MAKE) -C . -f makefile.$(DEVICE) examples PROFILE=$(PROFILE)

examples-clean:
	$(MAKE) -C . -f makefile.$(DEVICE) examples-clean PROFILE=$(PROFILE)

examples-scrub:
	$(MAKE) -C . -f makefile.$(DEVICE) examples-scrub PROFILE=$(PROFILE)

help:
	$(MAKE) -C . -f makefile.$(DEVICE) -s help PROFILE=$(PROFILE)

syscfg-gui:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(SYSCFG_SDKPRODUCT) --device $(SYSCFG_DEVICE) --context $(SYSCFG_CPU)

devconfig:
	$(SYSCFG_NWJS) $(SYSCFG_PATH) --product $(MOTOR_CONTROL_SDK_PATH)/devconfig/devconfig.json --device $(SYSCFG_DEVICE) --context $(SYSCFG_CPU) --output devconfig/ $(MOTOR_CONTROL_SDK_PATH)/devconfig/devconfig.syscfg

.PHONY: all clean scrub
.PHONY: libs libs-clean libs-scrub
.PHONY: examples examples-clean examples-scrub
.PHONY: help
.PHONY: syscfg-gui
.PHONY: devconfig


################ Internal make targets - not to be used by customers ################
NODE=node

GEN_BUILDFILES_TARGET?=development
INSTRUMENTATION_MODE?=disable

DOC_COMBO = r5f.ti-arm-clang
# default combo for doc generation
ifeq ($(DEVICE),$(filter $(DEVICE), am62x))
  DOC_COMBO = m4f.ti-arm-clang
endif

projectspec-help:
	$(MAKE) -C . -f makefile_projectspec.$(DEVICE) -s help PROFILE=$(PROFILE)

docs:
	$(MAKE) -C docs_src/docs/api_guide all DEVICE=$(DEVICE) DOC_COMBO=$(DOC_COMBO)

docs-clean:
	$(MAKE) -C docs_src/docs/api_guide clean DEVICE=$(DEVICE) DOC_COMBO=$(DOC_COMBO)

gen-buildfiles:
	$(NODE) ./.project/project.js --device $(DEVICE) --target $(GEN_BUILDFILES_TARGET) --instrumentation $(INSTRUMENTATION_MODE)

gen-buildfiles-clean:
	$(NODE) ./.project/project.js --device $(DEVICE) --target clean

syscfg-tests:
ifeq ($(DEVICE),$(filter $(DEVICE), am64x))
	-$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/tests/sanityTests.js -s $(SYSCFG_SDKPRODUCT) -d $(SYSCFG_DEVICE) -c a53ss0-0
endif
ifeq ($(DEVICE),$(filter $(DEVICE), am64x am243x am62x))
	-$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/tests/sanityTests.js -s $(SYSCFG_SDKPRODUCT) -d $(SYSCFG_DEVICE) -c m4fss0-0
endif
	-$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/tests/sanityTests.js -s $(SYSCFG_SDKPRODUCT) -d $(SYSCFG_DEVICE) -c r5fss0-0
ifeq ($(DEVICE),$(filter $(DEVICE), am273x awr294x))
	-$(SYSCFG_NODE) $(SYSCFG_CLI_PATH)/tests/sanityTests.js -s $(SYSCFG_SDKPRODUCT) -d $(SYSCFG_DEVICE) -c c66ss0
endif

.PHONY: projectspec-help docs docs-clean
.PHONY: gen-buildfiles gen-buildfiles-clean
.PHONY: syscfg-tests
