TOP = ../..
include $(TOP)/configure/CONFIG

USR_INCLUDES += -I$(ASYN)/include

LIBRARY += anc350

LIB_LIBS += asyn
anc350_LIBS += $(EPICS_BASE_IOC_LIBS)

DBD += devAnc350.dbd

anc350_INC += devAnc350.h

anc350_SRCS += devAnc350.c

include $(TOP)/configure/RULES

