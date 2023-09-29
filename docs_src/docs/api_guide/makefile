
export MOTOR_CONTROL_SDK_PATH=$(abspath ../../..)
include $(MOTOR_CONTROL_SDK_PATH)/imports.mak

DOXYGEN ?= doxygen
export DEVICE ?= am64x
export API_GUIDE_OUT_DIR=$(MOTOR_CONTROL_SDK_PATH)/docs/api_guide_$(DEVICE)

all:
	$(DOXYGEN) doxygen.cfg 2> doxy_warnings_$(DEVICE).txt
	cd theme && $(COPY) *.png $(API_GUIDE_OUT_DIR)


clean:
	$(MAKE) -C $(MOTOR_CONTROL_SDK_PATH)/test -f makefile.$(DEVICE) doxy_samples_$(DOC_COMBO)_clean
	$(RMDIR) $(API_GUIDE_OUT_DIR)

help:
	@echo "# Supported build targets:"
	@echo "# ========================"
	@echo "# $(MAKE) -s all DEVICE=<device>"
	@echo "# $(MAKE) -s clean DEVICE=<device>"
	@echo "# $(MAKE) -s help"
	@echo "#"

.PHONY: all clean help
