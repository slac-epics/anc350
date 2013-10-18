TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application
PROD_IOC = ancTest350

DBD += ancTest350.dbd
# ancTest350.dbd will be made up from these files:
ancTest350_DBD += base.dbd
ancTest350_DBD += asyn.dbd
ancTest350_DBD += drvAsynIPPort.dbd
ancTest350_DBD += motorSupport.dbd
ancTest350_DBD += motorSimSupport.dbd
ancTest350_DBD += devSoftMotor.dbd
ancTest350_DBD += devAnc350.dbd
ancTest350_DBD += anc350AsynMotor.dbd
#ancTest350_DBD += motorRecord.dbd
#ancTest350_DBD += motorSupport.dbd
#ancTest350_DBD += devSoftMotor.dbd

# Include the following libraries in the build
ancTest350_LIBS += asyn
ancTest350_LIBS += motor
ancTest350_LIBS += softMotor
ancTest350_LIBS += motorSimSupport
ancTest350_LIBS += anc350
ancTest350_LIBS += anc350AsynMotor
ancTest350_LIBS += $(EPICS_BASE_IOC_LIBS)



# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
ancTest350_SRCS += ancTest350_registerRecordDeviceDriver.cpp
ancTest350_SRCS_DEFAULT += ancTest350Main.cpp
ancTest350_SRCS_vxWorks += -nil-

#The following adds support from base/src/vxWorks
ancTest350_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary


#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

