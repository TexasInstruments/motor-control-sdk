
export MOTOR_CONTROL_SDK_PATH=$(abspath ../../..)
include $(MOTOR_CONTROL_SDK_PATH)/imports.mak

DOXYGEN ?= doxygen
export DEVICE ?= am243x
export API_GUIDE_OUT_DIR=$(MOTOR_CONTROL_SDK_PATH)/docs/api_guide_$(DEVICE)

all:
	$(MKDIR) $(API_GUIDE_OUT_DIR)
	$(DOXYGEN) doxygen.cfg 2> doxy_warnings_$(DEVICE).txt
	cd theme && $(COPY) *.png $(API_GUIDE_OUT_DIR)

clean:
	$(RMDIR) $(API_GUIDE_OUT_DIR)

help:
	@echo "# Supported build targets:"
	@echo "# ========================"
	@echo "# $(MAKE) -s all DEVICE=<device>"
	@echo "# $(MAKE) -s clean DEVICE=<device>"
	@echo "# $(MAKE) -s help"
	@echo "#"

.PHONY: all clean help
