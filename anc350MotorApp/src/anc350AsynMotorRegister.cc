#include <iocsh.h>
#include "anc350AsynMotor.h"
#include "epicsExport.h"

extern "C" {

/* int anc350AsynMotorCreate(port, address, card, Number of axes).*/
static const iocshArg anc350AsynMotorCreateArg0 = { "port",          iocshArgString};
static const iocshArg anc350AsynMotorCreateArg1 = { "address",       iocshArgInt};
static const iocshArg anc350AsynMotorCreateArg2 = { "card",          iocshArgInt};
static const iocshArg anc350AsynMotorCreateArg3 = { "Number of Axes",iocshArgInt};

static const iocshArg *const anc350AsynMotorCreateArgs[] = {
  &anc350AsynMotorCreateArg0,
  &anc350AsynMotorCreateArg1,
  &anc350AsynMotorCreateArg2,
  &anc350AsynMotorCreateArg3
};
static const iocshFuncDef anc350AsynMotorCreateDef ={"anc350AsynMotorCreate",4,anc350AsynMotorCreateArgs};

static void anc350AsynMotorCreateCallFunc(const iocshArgBuf *args)
{
  anc350AsynMotorCreate( args[0].sval, args[1].ival, args[2].ival, args[3].ival );
}


/*Register functions for IOC shell.*/
void anc350AsynMotorRegister(void)
{
  iocshRegister(&anc350AsynMotorCreateDef, anc350AsynMotorCreateCallFunc);
}
epicsExportRegistrar(anc350AsynMotorRegister);

} // extern "C"

