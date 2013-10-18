#!../../bin/linux-x86/ancTest350

< envPaths

cd ${TOP}

## Register all support components
dbLoadDatabase("dbd/ancTest350.dbd",0,0)
ancTest350_registerRecordDeviceDriver(pdbbase)

#drvAsynIPPortConfigure("IP1","172.27.13.14",0,0,0)
drvAsynIPPortConfigure("IP1","localhost:2101",0,0,0)

#=========================================================================
#  int anc350AsynMotorCreate(
#	    char port,    /* String name of asyn port */
#           int  addr,    /* Address value (axis number) */
#           int  card,    /* Number representing the motor controller */
#           int  nAxes,   /* Number of axes present on the controller */ )
##=========================================================================

anc350AsynMotorCreate("IP1","0","0","4")

#=========================================================================
#  drvAsynMotorConfigure(
#           char name,    /* String name assigned to device */
#           char driver,  /* String name of asyn driver to use */
#           int  card,    /* Number representing the motor controller */
#           int  nAxes,   /* Number of axes present on the controller */ )
##=========================================================================

drvAsynMotorConfigure("ANC1", "anc350AsynMotor","0","4")
#drvAsynMotorConfigure("ANC1", "drvANC150Asyn","0","1")

## Load record instances
dbLoadRecords("db/ancTest.db", "")
#dbLoadRecords("db/SIOC-DMP1-MC11-motor.db","")
#dbLoadRecords("db/asynRecord.db","P=T1:M1:,R=ASYN,PORT=IP1,ADDR=0,IMAX=200,OMAX=200")
#cd ${TOP}/iocBoot/${IOC}
iocInit()

