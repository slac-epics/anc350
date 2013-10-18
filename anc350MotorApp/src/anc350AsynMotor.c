/*
 * File:   anc350AsynMotor.c
 *
 * Date:   8th April 2009
 * Author: Alan Greer (Observatory Sciences Ltd)
 * Email:  ajg@observatorysciences.co.uk
 *
 * Description:
 *
 * This file contains asyn motor driver support for the attocube systems
 * ANC350 Piezo Motion Controller.  This device support requires the asyn
 * module to establish communications and the motor module for the interface
 * to the motor record.
 *
 * This has been built and tested against the following module versions
 * 
 * base  3.14.8
 * asyn  4.10
 * motor 6.4.2
 */
#include <stddef.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "paramLib.h"

#include "epicsFindSymbol.h"
#include "epicsTime.h"
#include "epicsThread.h"
#include "epicsEvent.h"
#include "epicsMutex.h"
#include "ellLib.h"

#include "drvSup.h"
#include "epicsExport.h"
#define DEFINE_MOTOR_PROTOTYPES 1
#include "motor_interface.h"
#include "asynDriver.h"
#include "asynOctetSyncIO.h"
#include "ucprotocol.h"
#include "anc350.h"

motorAxisDrvSET_t anc350AsynMotor =
  {
    15,
    motorAxisReport,            /**< Standard EPICS driver report function (optional) */
    motorAxisInit,              /**< Standard EPICS dirver initialisation function (optional) */
    motorAxisSetLog,            /**< Defines an external logging function (optional) */
    motorAxisOpen,              /**< Driver open function */
    motorAxisClose,             /**< Driver close function */
    motorAxisSetCallback,       /**< Provides a callback function the driver can call when the status updates */
    motorAxisSetDouble,         /**< Pointer to function to set a double value */
    motorAxisSetInteger,        /**< Pointer to function to set an integer value */
    motorAxisGetDouble,         /**< Pointer to function to get a double value */
    motorAxisGetInteger,        /**< Pointer to function to get an integer value */
    motorAxisHome,              /**< Pointer to function to execute a more to reference or home */
    motorAxisMove,              /**< Pointer to function to execute a position move */
    motorAxisVelocityMove,      /**< Pointer to function to execute a velocity mode move */
    motorAxisStop,              /**< Pointer to function to stop motion */
    motorAxisforceCallback,     /**< Pointer to function to request a poller status update */
  };

epicsExportAddress(drvet, anc350AsynMotor);

typedef struct drvAnc350 * ANC350DRV_ID;
typedef struct drvAnc350
{
    ANC350DRV_ID pNext;
    asynUser * pasynUser;
    int card;
    int nAxes;
    AXIS_HDL axis;
    epicsThreadId motorThread;
    epicsTimeStamp now;
    int movesDeferred;
    double movingPollPeriod;
    double idlePollPeriod;
    epicsEventId pollEventId;
    epicsMutexId controllerMutexId;
} drvAnc350_t;

/* Default polling periods (in milliseconds). */
static const int defaultMovingPollPeriod = 500;
static const int defaultIdlePollPeriod = 1000;

/* Message ID counter for matching replies */
static int mid = 0;
/* Mutex for protecting message ID increments */
static epicsMutexId midMutexId = NULL;
static int comms = 0;

typedef struct motorAxisHandle
{
    ANC350DRV_ID pDrv;
    int axis;
    asynUser * pasynUser;
    PARAMS params;
    motorAxisLogFunc print;
    void * logParam;
    epicsMutexId axisMutex;
    int scale;
    double previous_position;
    double previous_direction;
    double reference_position;
    int reference_search;
    double amplitude;
} motorAxis;

static ANC350DRV_ID pFirstDrv = NULL;

static int drvAnc350LogMsg( void * param, const motorAxisLogMask_t logMask, const char *pFormat, ...);
static motorAxisLogFunc drvPrint = drvAnc350LogMsg;
static motorAxisLogFunc drvPrintParam = NULL;

#define TRACE_FLOW    motorAxisTraceFlow
#define TRACE_DRIVER  motorAxisTraceIODriver
#define TRACE_ERROR   motorAxisTraceError

#define MAX(a,b) ((a)>(b)? (a): (b))
#define MIN(a,b) ((a)<(b)? (a): (b))

/*
 * Function: motorAxisReportAxis
 *
 * Parameters: pAxis   - Pointer to motor axis handle
 *             int     - Report level
 *
 * Returns: void
 * 
 * Description:
 *
 * Prints a report of the motor axis.  The amount of
 * information printed depends on the level of the report.
 */
static void motorAxisReportAxis( AXIS_HDL pAxis, int level )
{
  printf( "Found driver for drvAnc350 card %d, axis %d\n", pAxis->pDrv->card, pAxis->axis );
  if (level > 0) printf( "drvAnc350->axisMutex = %p\n", pAxis->axisMutex );

  if (level > 1)
  {
    motorParam->dump( pAxis->params );
  }
}

/*
 * Function: motorAxisReport
 *
 * Parameters: int     - Report level
 *
 * Returns: void
 * 
 * Description:
 *
 * Prints a report of each axis by looping through all axes.
 */
static void motorAxisReport( int level )
{
  ANC350DRV_ID pDrv;

  for (pDrv = pFirstDrv; pDrv != NULL; pDrv = pDrv->pNext)
  {
    int i;

    for ( i = 0; i < pDrv->nAxes; i++ )
      motorAxisReportAxis( &(pDrv->axis[i]), level );
  }
}

/*
 * Function: motorAxisInit
 *
 * Parameters: None
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Function currently does nothing.
 */
static int motorAxisInit( void )
{
  return MOTOR_AXIS_OK;
}

/*
 * Function: motorAxisSetLog
 *
 * Parameters: pAxis    - Pointer to motor axis handle
 *             logFunc  - Pointer to log function
 *             param    - Pointer to the user parameter for logging on this axis
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Defines an external logging function.
 */
static int motorAxisSetLog( AXIS_HDL pAxis, motorAxisLogFunc logFunc, void * param )
{
  if (pAxis == NULL)
  {
    if (logFunc == NULL)
    {
      drvPrint=drvAnc350LogMsg;
      drvPrintParam = NULL;
    }
    else
    {
      drvPrint=logFunc;
      drvPrintParam = param;
    }
  }
  else
  {
    if (logFunc == NULL)
    {
      pAxis->print=drvAnc350LogMsg;
      pAxis->logParam = NULL;
    }
    else
    {
      pAxis->print=logFunc;
      pAxis->logParam = param;
    }
  }
  return MOTOR_AXIS_OK;
}

/*
 * Function: motorAxisOpen
 *
 * Parameters: card     - Number representing the motor controller
 *             axis     - Axis number
 *             param    - Arbitrary parameter string
 *
 * Returns: Pointer to motor axis handle
 * 
 * Description:
 *
 * Opens the motor axis.  Creates and initialises the structure
 * used for storing axis information.  Returns a pointer to the
 * structure.
 */
static AXIS_HDL motorAxisOpen( int card, int axis, char * param )
{
  ANC350DRV_ID pDrv;
  AXIS_HDL pAxis = NULL;

  for ( pDrv=pFirstDrv; pDrv != NULL && (card != pDrv->card); pDrv = pDrv->pNext){}

  if (pDrv != NULL)
    if (axis >= 0 && axis < pDrv->nAxes) pAxis = &(pDrv->axis[axis]);

  pAxis->amplitude = 0.0;
  pAxis->reference_search = 0;
  pAxis->reference_position = 0.0;
  return pAxis;
}

/*
 * Function: motorAxisClose
 *
 * Parameters: pAxis    - Pointer to motor axis handle
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Function currently does nothing.
 */
static int motorAxisClose( AXIS_HDL pAxis )
{
  return MOTOR_AXIS_OK;
}

/*
 * Function: motorAxisGetInteger
 *
 * Parameters: pAxis    - Pointer to motor axis handle
 *             function - One of the #motorAxisParam_t values indicating which parameter to get
 *             value    - Pointer to store value
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Gets an integer parameter in the controller.
 */
static int motorAxisGetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int * value )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
  {
    return motorParam->getInteger( pAxis->params, (paramIndex) function, value );
  }
}

/*
 * Function: motorAxisGetDouble
 *
 * Parameters: pAxis    - Pointer to motor axis handle
 *             function - One of the #motorAxisParam_t values indicating which parameter to get
 *             value    - Pointer to store value
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Gets a double parameter in the controller.
 */
static int motorAxisGetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double * value )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
  {
    return motorParam->getDouble( pAxis->params, (paramIndex) function, value );
  }
}

/*
 * Function: motorAxisSetCallback
 *
 * Parameters: pAxis    - Pointer to motor axis handle
 *             callback - Pointer to a callback function taking a void parameter
 *             value    - Void pointer to parameter that should be used when calling the callback
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Set a callback function to be called when motor axis information changes.
 */
static int motorAxisSetCallback( AXIS_HDL pAxis, motorAxisCallbackFunc callback, void * param )
{
  if (pAxis == NULL) return MOTOR_AXIS_ERROR;
  else
  {
    return motorParam->setCallback( pAxis->params, callback, param );
  }
}

/*
 * Function: motorAxisAsynConnect
 *
 * Parameters: port        - String name of asyn port
 *             addr        - Address value (axis number)
 *             ppasynUser  - Additional user data
 *             inputEos    - Unused
 *             outputEos   - Unused
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Make the connection to the asyn layer using the OctetSyncIO interface.
 */
static int motorAxisAsynConnect( const char * port, int addr, asynUser ** ppasynUser, char * inputEos, char * outputEos )
{
    asynStatus status;

    status = pasynOctetSyncIO->connect( port, addr, ppasynUser, NULL);
    if (status) {
        drvPrint( drvPrintParam, TRACE_ERROR,
                  "drvPmacCreate: unable to connect to port %s\n",
                  port);
        return MOTOR_AXIS_ERROR;
    }

    return MOTOR_AXIS_OK;
}

/*
 * Function: motorAxisSet
 *
 * Parameters: pAxis     - Pointer to motor axis handle
 *             location  - Memory location to set value of
 *             value     - Value to write into memory location
 *             logGlobal - Specifies which asynUser to use
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Sends a set packet to the ANC 350 controller.  The value supplied is
 * written to the memory location specified and then an acknowledgement
 * is received.
 */
static int motorAxisSet( AXIS_HDL pAxis, int location, int value, int logGlobal )
{
  asynStatus status;
	int localMid = 1;
  unsigned int nBytesWritten = 0;
  asynUser *pasynUser = (logGlobal? pAxis->pDrv->pasynUser: pAxis->pasynUser);
  UcSetTelegram request;

	/* Lock the mid mutex and increment */
  if (epicsMutexLock(midMutexId) == epicsMutexLockOK) {
	  mid++;
  	if (mid > 10000){
  	  mid = 1;  
  	}
		localMid = mid;
  } else {
    drvPrint(drvPrintParam, TRACE_ERROR, "motorAxisSet: Failed to get midMutexId lock.\n");
  }
  epicsMutexUnlock(midMutexId);

  /* Create the command data structure */
  request.hdr.length            = sizeof( UcSetTelegram ) - sizeof( Int32 );
  request.hdr.opcode            = UC_SET;
  request.hdr.address           = 0x0000;
  request.hdr.index             = 0;
  request.hdr.correlationNumber = localMid;
  request.data[1]               = 1;


  request.hdr.address = location;
	request.data[0] = value;

  /* Set the index to the axis number */
	request.hdr.index = (pAxis->axis-1);
		
	/* Send the SET command */
	status = pasynOctetSyncIO->write(pasynUser,
                                   (char *)&request,
                                   sizeof(UcSetTelegram),
                                   0.5,
																	 &nBytesWritten);

  if (status){
    return MOTOR_AXIS_ERROR;
  }
  return MOTOR_AXIS_OK;
}

/*
 * Function: motorAxisGet
 *
 * Parameters: pAxis     - Pointer to motor axis handle
 *             location  - Memory location to set value of
 *             value     - Value to write into memory location
 *             logGlobal - Specifies which asynUser to use
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Sends a get packet to the ANC 350 controller.  The value at the memory
 * location specified is requested and the acknowledgement containing the
 * current value is received.
 */
static int motorAxisGet( AXIS_HDL pAxis, int location, int *value, int logGlobal )
{
	asynStatus status;
	int localMid = 1;
  unsigned int nBytesWritten = 0;
  unsigned int nBytesRead = 0;
  int eom = 0;
  int match = 0;
  int count = 0;
  char raw[512];
  UcGetTelegram request;
  asynUser *pasynUser = (logGlobal? pAxis->pDrv->pasynUser: pAxis->pasynUser);

  /* Create a union to map the byte response into an acknowledge structure */
  union 
  {
    char	  raw[sizeof(UcAckTelegram)];
    UcAckTelegram ack;
  } tel;

	/* Lock the mid mutex and increment */
  if (epicsMutexLock(midMutexId) == epicsMutexLockOK) {
	  mid++;
  	if (mid > 10000){
  	  mid = 1;  
  	}
		localMid = mid;
  } else {
    drvPrint(drvPrintParam, TRACE_ERROR, "motorAxisGet: Failed to get midMutexId lock.\n");
  }
  epicsMutexUnlock(midMutexId);

  /* Create the request data structure */
  request.hdr.length            = sizeof( UcGetTelegram ) - sizeof( Int32 );
  request.hdr.opcode            = UC_GET;
  request.hdr.address           = 0x0000;
  request.hdr.index             = 0;
  request.hdr.correlationNumber = localMid;

	/* The input string is the memory address to query */
	request.hdr.address = location;

	/* This corresponds to an axis number (or isn't used) */
	request.hdr.index = (pAxis->axis-1);


	/* Send the GET request */
  status = pasynOctetSyncIO->writeRead(pasynUser,
                                   (char *)&request,
                                   sizeof(UcGetTelegram),
                                   raw,
                                   28,
                                   0.1,
   																 &nBytesWritten,
                                   &nBytesRead,
                                   &eom);

  if (status==asynSuccess){
    for(count = 0; count < nBytesRead; count++){
      tel.raw[count] = raw[count];
    }
 		if (tel.ack.hdr.correlationNumber == localMid){
  		match = 1;
	  	*value = (epicsInt32)tel.ack.data[0];
	  } else {
      status = asynError;
    }
  }


  if (status!=asynSuccess){
    comms++;
    if (comms > 200){
      motorParam->setInteger( pAxis->params, motorAxisCommError, 1 );
      drvPrint( drvPrintParam, TRACE_ERROR, "anc350AsynMotorGet: Comms error.\n");
    }
    return MOTOR_AXIS_ERROR;
  } else {
    comms = 0;
  }
  motorParam->setInteger( pAxis->params, motorAxisCommError, 0 );
  return MOTOR_AXIS_OK;
}

/*
 * Function: motorAxisSetDouble
 *
 * Parameters: pAxis     - Pointer to motor axis handle
 *             function  - One of the# motorAxisParam_t values indicating which parameter to set
 *             value     - Value to be assigned to the parameter
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Sets a double parameter in the controller.
 */
static int motorAxisSetDouble( AXIS_HDL pAxis, motorAxisParam_t function, double value )
{
    int status = MOTOR_AXIS_OK;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
        {
            if (status == MOTOR_AXIS_OK )
            {
                motorParam->setDouble( pAxis->params, function, value );
                motorParam->callCallback( pAxis->params );
            }
            epicsMutexUnlock( pAxis->axisMutex );
        }
    }
  return status;
}

/*
 * Function: motorAxisSetInteger
 *
 * Parameters: pAxis     - Pointer to motor axis handle
 *             function  - One of the# motorAxisParam_t values indicating which parameter to set
 *             value     - Value to be assigned to the parameter
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Sets an integer parameter in the controller.
 */
static int motorAxisSetInteger( AXIS_HDL pAxis, motorAxisParam_t function, int value )
{
    int status = MOTOR_AXIS_OK;

    if (pAxis == NULL) return MOTOR_AXIS_ERROR;
    else
    {
        status = motorAxisSetDouble( pAxis, function, (double) value );
    }
    return status;
}

/*
 * Function: motorAxisMove
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *             position      - Position to move to in motor units
 *             relative      - If zero position is an absolute position, otherwise it is relative
 *                             to the current position
 *             min_velocity  - Minimum startup velocity in motor units/second
 *             max_velocity  - Maximum velocity during move in motor units/second
 *             acceleration  - Maximum acceleration (or decelleration) during velocity ramp in 
 *                             motor units/second squared
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * This is a normal move command.  Hump detection is turned on to stop the axis if there is a
 * problem.  The amplitude control mode is set to speed closed loop and the required frequency
 * (to achieve the required speed) is calculated and sent to the controller.  The move command
 * is then issued either as an absolute demand or as a relative demand.
 */
static int motorAxisMove( AXIS_HDL pAxis, double position, int relative, double min_velocity, double max_velocity, double acceleration )
{
	long imove;
	int cmd;
	int posdir = 0;
  int status = MOTOR_AXIS_ERROR;
  //double frequency = 1000.0;
  //double vel_amp = 0.0;

	if (pAxis != NULL){
    /* Set hump detection */
    status = motorAxisSet( pAxis, ID_ANC_STOP_EN, 1, 0 );
    /* Set amplitude ctrl to amplitude closed loop */
    status = motorAxisSet( pAxis, ID_ANC_REGSPD_SELSP, 1, 0 );
    /* Calculate the required frequency */
    //vel_amp = max_velocity / 1000.0;
    //if (vel_amp < 0.0){
    //  vel_amp = vel_amp * -1;
    //}
    //frequency = vel_amp * 100 / pAxis->amplitude;
    /* Set the frequency */
    //motorAxisSet( pAxis, ID_ANC_FAST_FREQ, ((int)frequency), 0 );

		if (relative){
			cmd = ID_ANC_RUN_RELATIVE;
		} else {
			cmd = ID_ANC_RUN_TARGET;
		}

		if (position >= 0.0){
			posdir = 1;
		} else {
			posdir = 0;
		}

		imove = (int)(position + pAxis->reference_position);

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK){
      status = motorAxisSet( pAxis, ID_ANC_TARGET, imove, 0 );
      status = motorAxisSet( pAxis, cmd, 1, 0 );
      /* Set direction indicator. */
      motorParam->setInteger(pAxis->params, motorAxisDirection, posdir);
      motorParam->setInteger( pAxis->params, motorAxisDone, 0 );
      motorParam->callCallback( pAxis->params );
      epicsMutexUnlock( pAxis->axisMutex );
    }
	  /* Signal the poller task.*/
	  epicsEventSignal(pAxis->pDrv->pollEventId);
	}
  return status;
}

/*
 * Function: motorAxisHome
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *             min_velocity  - Minimum startup velocity in motor units/second
 *             velocity      - Maximum velocity during move in motor units/second
 *             acceleration  - Maximum acceleration (or decelleration) during velocity ramp in 
 *                             motor units/second squared
 *             forwards      - If zero, initial move is in negative direction
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * This initiates a homing operation (in either direction).
 */
static int motorAxisHome( AXIS_HDL pAxis, double min_velocity, double max_velocity, double acceleration, int forwards )
{
  int posdir;
	int cmd;
  int status = MOTOR_AXIS_ERROR;
  //double frequency = 1000.0;
  //double vel_amp = 0.0;

	if (pAxis != NULL){
    /* Set hump detection */
    status = motorAxisSet( pAxis, ID_ANC_STOP_EN, 1, 0 );
    /* Set amplitude ctrl to amplitude closed loop */
    status = motorAxisSet( pAxis, ID_ANC_REGSPD_SELSP, 1, 0 );
    /* Calculate the required frequency */
    //vel_amp = max_velocity / 1000.0;
    //if (vel_amp < 0.0){
    //  vel_amp = vel_amp * -1;
    //}
    //frequency = vel_amp * 100 / pAxis->amplitude;
    /* Set the frequency */
    //motorAxisSet( pAxis, ID_ANC_FAST_FREQ, ((int)frequency), 0 );

		if (forwards > 0){
			cmd = ID_ANC_CONT_FWD;
			posdir = 1;
		} else {
			cmd = ID_ANC_CONT_BKWD;
			posdir = 0;
		}

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK){
      status = motorAxisSet( pAxis, cmd, 1, 0 );
      /* Set direction indicator. */
      motorParam->setInteger(pAxis->params, motorAxisDirection, posdir);
      motorParam->setInteger( pAxis->params, motorAxisDone, 0 );
      motorParam->callCallback( pAxis->params );
      epicsMutexUnlock( pAxis->axisMutex );
    }

    pAxis->reference_search = 1;

	  /* Signal the poller task.*/
	  epicsEventSignal(pAxis->pDrv->pollEventId);
	}
  return status;
}

/*
 * Function: motorAxisVelocityMove
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *             min_velocity  - Minimum startup velocity in motor units/second
 *             velocity      - Maximum velocity during move in motor units/second
 *             acceleration  - Maximum acceleration (or decelleration) during velocity ramp in 
 *                             motor units/second squared
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * This is a constant velocity (jog) move.  Hump detection is turned on to stop the axis if 
 * there is a problem.  The amplitude control mode is set to speed closed loop and the required
 * frequency (to achieve the required speed) is calculated and sent to the controller.  The jog
 * command is then issued.
 */
static int motorAxisVelocityMove(  AXIS_HDL pAxis, double min_velocity, double velocity, double acceleration )
{
  int posdir;
	int cmd;
  int status = MOTOR_AXIS_ERROR;
  //double frequency = 1000.0;
  //double vel_amp = 0.0;

	if (pAxis != NULL){
    /* Set hump detection */
    status = motorAxisSet( pAxis, ID_ANC_STOP_EN, 1, 0 );
    /* Set amplitude ctrl to amplitude closed loop */
    status = motorAxisSet( pAxis, ID_ANC_REGSPD_SELSP, 1, 0 );
    /* Calculate the required frequency */
    //vel_amp = velocity / 1000.0;
    //if (vel_amp < 0.0){
    //  vel_amp = vel_amp * -1;
    //}
    //frequency = vel_amp * 100 / pAxis->amplitude;
    /* Set the frequency */
    //motorAxisSet( pAxis, ID_ANC_FAST_FREQ, ((int)frequency), 0 );


		if (velocity > 0.0){
			cmd = ID_ANC_CONT_FWD;
			posdir = 1;
		} else {
			cmd = ID_ANC_CONT_BKWD;
			posdir = 0;
		}

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK){
      status = motorAxisSet( pAxis, cmd, 1, 0 );
      /* Set direction indicator. */
      motorParam->setInteger(pAxis->params, motorAxisDirection, posdir);
      motorParam->setInteger( pAxis->params, motorAxisDone, 0 );
      motorParam->callCallback( pAxis->params );
      epicsMutexUnlock( pAxis->axisMutex );
    }
	  /* Signal the poller task.*/
	  epicsEventSignal(pAxis->pDrv->pollEventId);
	}
	return status;
}

/*
 * Function: motorAxisProfileMove
 *
 * Parameters: pAxis      - Pointer to motor axis handle
 *             npoints    - Number of points in the position array
 *             positions  - Double precision array of positions
 *             times      - Double precision array of times (seconds)
 *             relative   - Integer which is non-zero for relative positions and zero for absolute
 *             trigger    - Integer indication which trigger to use to initiate motion
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Unused.
 */
static int motorAxisProfileMove( AXIS_HDL pAxis, int npoints, double positions[], double times[], int relative, int trigger )
{
  return MOTOR_AXIS_ERROR;
}

/*
 * Function: motorAxisTriggerProfile
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Unused.
 */
static int motorAxisTriggerProfile( AXIS_HDL pAxis )
{
  return MOTOR_AXIS_ERROR;
}

/*
 * Function: motorAxisStop
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *             acceleration  - Maximum acceleration (or decelleration) during velocity ramp in 
 *                             motor units/second squared
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * This aborts any current motion and brings the axis to a halt at the current position.
 * The command completes as soon as the stop is initiated
 */
static int motorAxisStop( AXIS_HDL pAxis, double acceleration )
{
  int cmd;
  int status = MOTOR_AXIS_ERROR;
  if (pAxis != NULL){
		if (pAxis->previous_direction == 1){
			cmd = ID_ANC_SGL_FWD;
		} else {
			cmd = ID_ANC_SGL_BKWD;
		}

    pAxis->reference_search = 0;

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK){
      status = motorAxisSet( pAxis, cmd, 1, 0 );
      /* Set direction indicator. */
      motorParam->setInteger( pAxis->params, motorAxisDone, 1 );
      motorParam->callCallback( pAxis->params );
      epicsMutexUnlock( pAxis->axisMutex );
    }
	  /* Signal the poller task.*/
	  epicsEventSignal(pAxis->pDrv->pollEventId);
	}
  return status;
}

/*
 * Function: motorAxisforceCallback
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * This request a poller status update.
 */
static int motorAxisforceCallback(AXIS_HDL pAxis)
{
	if (pAxis == NULL)
		return (MOTOR_AXIS_ERROR);

	pAxis->print(pAxis->logParam, TRACE_FLOW, "motorAxisforceCallback: request card %d, axis %d status update\n",
	             pAxis->pDrv->card, pAxis->axis);

	motorParam->forceCallback(pAxis->params);

	return (MOTOR_AXIS_OK);
}

/*
 * Function: drvAnc350GetGlobalStatus
 *
 * Parameters: pDrv         - Pointer to driver structure
 *             pasynUser    - Pointer to user data
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Currently only returns good status.
 */
static int drvAnc350GetGlobalStatus( ANC350DRV_ID pDrv, asynUser * pasynUser )
{
  return 0;
}

/*
 * Function: drvAnc350GetAxisStatus
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *             pasynUser     - Pointer to user data
 *             globalStatus  - Unused
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Gets the current status of the axis.  This includes
 * 1) Referenced
 * 2) Hump (limits) detected
 * 3) Current position
 * 4) Moving
 * 5) Direction
 */
static void drvAnc350GetAxisStatus( AXIS_HDL pAxis, asynUser * pasynUser, epicsUInt32 globalStatus )
{
    int done;
    double position;
		int value;
    int status;
		int referenced;
    double reference_position = 0.0;
    int direction = 0;
		int hump = 0;
		int humpstatus = 0;

    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
    {
		/*Read the axis status.*/
		status = motorAxisGet(pAxis, ID_ANC_STATUS, &value, 0);
		humpstatus = status;
		if (status == asynSuccess) {

			/* Use for in position */
			done = value&ANC_STATUS_RUNNING;
			if (done == 0){
	        	motorParam->setInteger(pAxis->params, motorAxisDone, 1 );
			} else {
	    	    motorParam->setInteger(pAxis->params, motorAxisDone, 0 );
			}

			/* Use for valid reference position */
	  		referenced = (value&ANC_STATUS_REF_VALID) >> 11;
			if (referenced == 0){
	  			motorParam->setInteger(pAxis->params, motorAxisHomed, referenced);
	        	motorParam->setInteger(pAxis->params, motorAxisHomeSignal, referenced);
			} else {
	        	if (pAxis->reference_search == 1){
					pAxis->reference_search = 0;
					status = motorAxisSet( pAxis, ID_ANC_SGL_FWD, 1, 0 );
					motorParam->setInteger( pAxis->params, motorAxisDone, 1 );
		  		  	motorParam->setInteger(pAxis->params, motorAxisHomed, referenced);
					motorParam->setInteger(pAxis->params, motorAxisHomeSignal, referenced);
		        }
			}

			/* Hump detected? */
      		hump = (value&ANC_STATUS_HUMP) >> 1;
		}

      /* Get the current amplitude */
			status = motorAxisGet( pAxis, ID_ANC_AMPL, &value, 0 );
      pAxis->amplitude = (((double)value) / 1000.0);

      /* Get the stored reference position */
			status = motorAxisGet( pAxis, ID_ANC_REFCOUNTER, &value, 0 );
      if (status == asynSuccess){
        reference_position = ((double)value);
      } else {
        reference_position = pAxis->reference_position;
      }
 			status = motorAxisGet( pAxis, ID_ANC_COUNTER, &value, 0 );
      if (status == asynSuccess){
   			position = ((double)value);
		/* Check for homed, if true then subtract the reference position */
        /*motorParam->getInteger(pAxis->params, motorAxisHomed, &referenced);
        if (referenced == 1){
          position -= reference_position;
          pAxis->reference_position = reference_position;
        }*/
		    /* Change by ELS: always subtract the reference position regardless of homed state */
		    position -= reference_position;
            pAxis->reference_position = reference_position;
            /* Check the direction using previous position */
 	  		if ((position - pAxis->previous_position) > 500.0){
 	  			direction = 1;
 	  		} else if ((position - pAxis->previous_position) < -500.0){
 	  			direction = 0;
 	  		} else {
 	  			direction = pAxis->previous_direction;
 	  		}
 	      motorParam->setInteger( pAxis->params, motorAxisDirection, direction);
 	      /*Store position to calculate direction for next poll.*/
 	      pAxis->previous_position = position;
 	      pAxis->previous_direction = direction;
 
        status = motorParam->setDouble(pAxis->params, motorAxisPosition, position);
        motorParam->setDouble(pAxis->params, motorAxisEncoderPosn, position);
      }

			/* Check for hard limit.  Only hump available so notify limit by checking direction */
			if (hump && (humpstatus == asynSuccess)){
				if (direction == 1){
					motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 1);
					motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 0);
				} else {
					motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 0);
					motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 1);
				}
			} else {
				motorParam->setInteger(pAxis->params, motorAxisHighHardLimit, 0);
				motorParam->setInteger(pAxis->params, motorAxisLowHardLimit, 0);
			}

      /*Combine several comms type errors for the motor record comm error bit.*/
      motorParam->setInteger( pAxis->params, motorAxisProblem, globalStatus );
      motorParam->callCallback( pAxis->params );

      epicsMutexUnlock( pAxis->axisMutex );
    }
}

/*
 * Function: drvAnc350GetAxisInitialStatus
 *
 * Parameters: pAxis         - Pointer to motor axis handle
 *             pasynUser     - Pointer to user data
 *
 * Returns: Void
 * 
 * Description:
 *
 * Gets the initial status of the axis.  This includes
 * 1) Referenced state
 */
static void drvAnc350GetAxisInitialStatus( AXIS_HDL pAxis, asynUser * pasynUser )
{
		int value = 0;
    int referenced = 0;

    /* Read all the status for this axis in one go */
    if (epicsMutexLock( pAxis->axisMutex ) == epicsMutexLockOK)
    {
      /*Read the axis status.*/
			motorAxisGet(pAxis, ID_ANC_STATUS, &value, 0);
      /* Use for valid reference position */
  		referenced = (value&ANC_STATUS_REF_VALID) >> 11;
	  	motorParam->setInteger(pAxis->params, motorAxisHomed, referenced);
      motorParam->setInteger(pAxis->params, motorAxisHomeSignal, referenced);

      motorParam->setDouble(  pAxis->params, motorAxisHasEncoder, 1);
      motorParam->callCallback( pAxis->params );
      epicsMutexUnlock( pAxis->axisMutex );
    }
}

/*
 * Function: drvAnc350Task
 *
 * Parameters: pDrv         - Pointer to driver structure
 *
 * Returns: Void
 * 
 * Description:
 *
 * Continuously polling task to get the status of the controller
 * and axes.
 */
static void drvAnc350Task( ANC350DRV_ID pDrv )
{
  int i = 0;
  int done = 0;
  int eventStatus = 0;
  float timeout = 0.0;
  float factor = 0.0;
  float skipglobal = 0.0;
  float skips[pDrv->nAxes];
  epicsUInt32 globalStatus = 0;

  for (i=0; i<pDrv->nAxes; i++) {
    skips[i] = 0;
  }

  while ( 1 )
  {
    /* Wait for an event, or a timeout. If we get an event, force an update.*/
    if (epicsMutexLock(pDrv->controllerMutexId) == epicsMutexLockOK) {
      timeout = pDrv->movingPollPeriod;
      /* roughly calculate how many moving polls to an idle poll */
      factor = pDrv->movingPollPeriod / pDrv->idlePollPeriod;
    }
    else {
      drvPrint(drvPrintParam, TRACE_ERROR, "drvAnc350Task: Failed to get controllerMutexId lock.\n");
    }
    epicsMutexUnlock(pDrv->controllerMutexId);
    eventStatus = epicsEventWaitWithTimeout(pDrv->pollEventId, timeout);

    /* Get global status at the slow poll rate.*/
    if (skipglobal <= 0.0) {
      globalStatus = drvAnc350GetGlobalStatus(pDrv, pDrv->pasynUser);
      skipglobal = 1.0;
    }
    skipglobal -= factor;

    /* Get axis status */
    for ( i = 0; i < pDrv->nAxes; i++ )
    {
      AXIS_HDL pAxis = &(pDrv->axis[i]);
      if (eventStatus == epicsEventWaitOK)
      {
      	/* If we got an event, then one motor is moving, so force an update for all */
      	done = 0;
      }
      else
      {
      	/* get the cached done status */
      	epicsMutexLock( pAxis->axisMutex );
	      motorParam->getInteger( pAxis->params, motorAxisDone, &done );
	      epicsMutexUnlock( pAxis->axisMutex );
      }
      if ((skips[i]<=0.0) || (done == 0))
      {
	      /* if it's time for an idle poll or the motor is moving */
	      drvAnc350GetAxisStatus( pAxis, pDrv->pasynUser, globalStatus );
	      skips[i] = 1.0;
      }
      skips[i] -= factor;
    }
  }
}

/*
 * Function: anc350AsynMotorCreate
 *
 * Parameters: port   - String name of asyn port
 *             addr   - Address value (axis number)
 *             card   - Number representing the motor controller
 *             nAxes  - Number of axes present on the controller
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Creates the driver structure.  Creates the polling thread and sets
 * it running.  Initialises all data and structures required for the
 * system.
 */
int anc350AsynMotorCreate( char *port, int addr, int card, int nAxes )
{
  int i;
  int status = MOTOR_AXIS_OK;
  ANC350DRV_ID pDrv;
  ANC350DRV_ID * ppLast = &(pFirstDrv);

	/* Create the Mutex for the MID if necessary */
	if (midMutexId == NULL){
    if ((midMutexId = epicsMutexCreate()) == NULL) {
      drvPrint( drvPrintParam, TRACE_ERROR, "anc350AsynMotorCreate: Could not create midMutexId.\n");
    }
	}

  for ( pDrv = pFirstDrv; pDrv != NULL &&  (pDrv->card != card); pDrv = pDrv->pNext ){
    ppLast = &(pDrv->pNext);
  }

  if (nAxes < 1 ) nAxes = 1;

  if ( pDrv == NULL){
    drvPrint( drvPrintParam, TRACE_FLOW,
              "Creating ANC350 motor driver on port %s, address %d: card: %d, naxes: %d\n",
              port, addr, card, nAxes );

    pDrv = (ANC350DRV_ID) calloc( 1, sizeof(drvAnc350_t) );

    if (pDrv != NULL){
      pDrv->axis = (AXIS_HDL) calloc( nAxes, sizeof( motorAxis ) );

      if (pDrv->axis != NULL ){
        pDrv->nAxes = nAxes;
        pDrv->card = card;

        /* Set default polling rates.*/
        pDrv->movingPollPeriod = (double)defaultMovingPollPeriod / 1000.0;
        pDrv->idlePollPeriod = (double)defaultIdlePollPeriod / 1000.0;
        /* Create event to signal poller task with.*/
        pDrv->pollEventId = epicsEventMustCreate(epicsEventEmpty);
        /* Create mutex ID for controller.*/
        if ((pDrv->controllerMutexId = epicsMutexCreate()) == NULL) {
          drvPrint( drvPrintParam, TRACE_ERROR, "anc350AsynMotorCreate: Could not create controllerMutexId.\n");
        }

        status = motorAxisAsynConnect( port, addr, &(pDrv->pasynUser), "\006", "\r" );

        for (i=0; i<nAxes && status == MOTOR_AXIS_OK; i++ ){
          if ((pDrv->axis[i].params = motorParam->create( 0, MOTOR_AXIS_NUM_PARAMS )) != NULL &&
              (pDrv->axis[i].axisMutex = epicsMutexCreate( )) != NULL){

            pDrv->axis[i].pDrv = pDrv;
            pDrv->axis[i].axis = i+1;
            pDrv->axis[i].logParam  = pDrv->pasynUser;
            pDrv->axis[i].pasynUser = pDrv->pasynUser;
            pDrv->axis[i].scale = 1;

            asynPrint( pDrv->pasynUser, ASYN_TRACE_FLOW, 
                       "anc350AsynMotorCreate: Created motor for card %d, signal %d OK\n",
                       card, i );
          } else {
            asynPrint( pDrv->pasynUser, ASYN_TRACE_ERROR,
                       "anc350AsynMotorCreate: unable to set create axis %d on %s: insufficient memory\n",
                       i, port );

            status = MOTOR_AXIS_ERROR;
          }
        }

        if ( status == MOTOR_AXIS_ERROR ){
          for (i=0; i<nAxes; i++ ){
            if (pDrv->axis[i].params != NULL) motorParam->destroy( pDrv->axis[i].params );
            if (pDrv->axis[i].axisMutex != NULL) epicsMutexDestroy( pDrv->axis[i].axisMutex );
          }
          free ( pDrv );
        }
      } else {
        free ( pDrv );
        status = MOTOR_AXIS_ERROR;
      }
    } else {
      drvPrint( drvPrintParam, TRACE_ERROR,
                "anc350AsynMotorCreate: unable to create driver for port %s: insufficient memory\n",
                port );

      status = MOTOR_AXIS_ERROR;
    }

    if ( status == MOTOR_AXIS_OK ) *ppLast = pDrv;
  } else {
    drvPrint( drvPrintParam, TRACE_ERROR, "anc350AsynMotorCreate: Motor for card %d already exists\n", card );
    status = MOTOR_AXIS_ERROR;
  }
  if (status == MOTOR_AXIS_OK){
    int i;

    /* Do an initial poll of all status */
	  for ( i = 0; i < pDrv->nAxes; i++ ){
      AXIS_HDL pAxis = &(pDrv->axis[i]);

      drvAnc350GetAxisInitialStatus( pAxis, pDrv->pasynUser );
      drvAnc350GetAxisStatus( pAxis, pDrv->pasynUser, 0 );
    }

    pDrv->motorThread = epicsThreadCreate( "drvAnc350Thread",
                                           epicsThreadPriorityLow,
                                           epicsThreadGetStackSize(epicsThreadStackMedium),
                                           (EPICSTHREADFUNC) drvAnc350Task, (void *) pDrv );
    if (pDrv->motorThread == NULL){
      asynPrint(pDrv->pasynUser, ASYN_TRACE_ERROR, "anc350AsynMotorCreate: Cannot start motor polling thread\n" );
      return MOTOR_AXIS_ERROR;
    }

  }
  return status;
}

/*
 * Function: drvAnc350LogMsg
 *
 * Parameters: param    - Pointer to asynUser
 *             mask     - Reason
 *             pFormat  - String to log
 *             ...      - Data values to log
 *
 * Returns: Integer status value
 * 
 * Description:
 *
 * Logs a message using the asyn logging functions.
 */
static int drvAnc350LogMsg( void * param, const motorAxisLogMask_t mask, const char *pFormat, ...)
{

    va_list	pvar;
    int		nchar=0;
    asynUser *  pasynUser =  (asynUser *) param;
    int         reason = (int) mask;

    if ( pasynUser == NULL )
    {
        va_start(pvar, pFormat);
        vprintf( pFormat, pvar );
        va_end (pvar);
    }
    else if ( pasynTrace->getTraceMask(pasynUser) & reason )
    {
        va_start(pvar, pFormat);
        nchar = pasynTrace->vprint( pasynUser, reason, pFormat, pvar );
        va_end (pvar);
    }

    return(nchar);
}

