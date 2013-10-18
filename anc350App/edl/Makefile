TOP=../..
include $(TOP)/configure/CONFIG

DATA += ancStepModule.edl

install:
	mkdir -p $(TOP)/data
	cp -f $(TOP)/anc350App/edl/ancStepModule.edl $(TOP)/data/ancStepModule.edl
	cp -f $(TOP)/anc350App/edl/ancController.edl $(TOP)/data/ancController.edl

clean::
	rm -rf $(TOP)/data

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

