/*
 * File:   devAnc350.c
 *
 * Date:   12th March 2009
 * Author: Alan Greer (Observatory Sciences Ltd)
 * Email:  ajg@observatorysciences.co.uk
 *
 * Description:
 *
 * This file contains the device support code for TCP/IP communications
 * with the Attocube ANC350 Piezo Motion Controller.  This device support
 * requires the asyn module to establish communications.
 */
#include <stdlib.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>

#include <alarm.h>
#include <recGbl.h>
#include <dbAccess.h>
#include <dbDefs.h>
#include <link.h>
#include <epicsPrint.h>
#include <epicsMutex.h>
#include <epicsString.h>
#include <cantProceed.h>
#include <dbCommon.h>
#include <dbScan.h>
#include <callback.h>
#include <stringinRecord.h>
#include <stringoutRecord.h>
#include <longinRecord.h>
#include <longoutRecord.h>
#include <aiRecord.h>
#include <aoRecord.h>
#include <biRecord.h>
#include <boRecord.h>
#include <mbbiRecord.h>
#include <mbboRecord.h>
#include <waveformRecord.h>
#include <menuFtype.h>
#include <recSup.h>
#include <devSup.h>

#include <epicsExport.h>
#include "asynDriver.h"
#include "asynDrvUser.h"
#include "asynOctet.h"
#include "asynEpicsUtils.h"
#include <epicsExport.h>

#include "devAnc350.h"
#include "ucprotocol.h"
#include "anc350.h"

/* General purpose function declarations */
static asynStatus writeIt(asynUser *pasynUser, const char *message, size_t nbytes);
static asynStatus readIt(asynUser *pasynUser, char *message, size_t maxBytes, size_t *nBytesRead);
static asynStatus flushIt(asynUser *pasynUser);
static void finish(dbCommon *precord);
static char *skipWhite(char *pstart, int commaOk);

/* Define the functions for initialising and callback of the longin and longout records */
static long initLiRead(longinRecord *pli);
static void callbackLiRead(asynUser *pasynUser);
static long initLoWrite(longoutRecord *plo);
static void callbackLoWrite(asynUser *pasynUser);

/* Simple static counter for message identification */
static int mid = 0;
/* Mutex for protecting message ID increments */
static epicsMutexId midMutexId = NULL;

commonDset asynLiAnc350Read        = {5, 0, 0, initLiRead,      0, processCommon};
commonDset asynLoAnc350Write       = {5, 0, 0, initLoWrite,     0, processCommon};

epicsExportAddress(dset, asynLiAnc350Read);
epicsExportAddress(dset, asynLoAnc350Write);

/*
 * Function: writeIt
 *
 * Parameters: pasynUser - Pointer to the asynUser structure
 *             message   - Pointer to the string to write
 *             nbytes    - Number of characters (1 byte each) to write
 *
 * Returns: asynStatus success value
 * 
 * Description:
 *
 * Used to write a string to the driver via asynDriver support code.
 * A status value is returned accordingly.
 */
static asynStatus writeIt(asynUser *pasynUser,const char *message,size_t nbytes)
{
  devPvt     *pdevPvt = (devPvt *)pasynUser->userPvt;
  dbCommon   *precord = pdevPvt->precord;
  asynOctet  *poctet = pdevPvt->poctet;
  void       *octetPvt = pdevPvt->interfacePvt;
  asynStatus status;
  size_t     nbytesTransfered;
  
  /* Make the call into asyn for writing the message */
  status = poctet->write(octetPvt,pasynUser,message,nbytes,&nbytesTransfered);
  if(status!=asynSuccess) {
    /* An error has occurred, use the asynDriver print method to notify */
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
	      "%s devAsynOctet: writeIt failed %s\n",
	      precord->name,pasynUser->errorMessage);
    recGblSetSevr(precord, WRITE_ALARM, INVALID_ALARM);
    return status;
  }
  if(nbytes != nbytesTransfered) {
    /* An incorrect number of bytes has been written, again raise an error */
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
	      "%s devAsynOctet: writeIt requested %d but sent %d bytes\n",
	      precord->name,nbytes,nbytesTransfered);
    recGblSetSevr(precord, WRITE_ALARM, MINOR_ALARM);
    return asynError;
  }
  asynPrintIO(pasynUser,ASYN_TRACEIO_DEVICE,message,nbytes,
	      "%s devAsynOctet: writeIt\n",precord->name);
  return status;
}

/*
 * Function: readIt
 *
 * Parameters: pasynUser - Pointer to the asynUser structure
 *             message   - Pointer to where the string will be stored
 *             maxBytes  - Maximum number of bytes the above string can hold
 *             nbytes    - Number of characters (1 byte each) that have been read
 *
 * Returns: asynStatus success value
 * 
 * Description:
 *
 * Reads a string back from the driver via asynDriver support code.
 * A status value is returned accordingly.
 */
static asynStatus readIt(asynUser *pasynUser,char *message,
        size_t maxBytes, size_t *nBytesRead)
{
  devPvt     *pdevPvt = (devPvt *)pasynUser->userPvt;
  asynOctet  *poctet = pdevPvt->poctet;
  void       *octetPvt = pdevPvt->interfacePvt;
  asynStatus status;
  int        eomReason;
  
  /* Make the call into asyn for reading the message */
  status = poctet->read(octetPvt,pasynUser,message,maxBytes,nBytesRead,&eomReason);
  return status;
}

/*
 * Function: flushIt
 *
 * Parameters: pasynUser - Pointer to the asynUser structure
 *
 * Returns: asynStatus success value
 * 
 * Description:
 *
 * Attempts to flush the connection made through the asynDriver.
 * A status value is returned accordingly.
 */
static asynStatus flushIt(asynUser *pasynUser)
{
  devPvt     *pdevPvt = (devPvt *)pasynUser->userPvt;
  asynOctet  *poctet = pdevPvt->poctet;
  void       *octetPvt = pdevPvt->interfacePvt;
  asynStatus status;

  /* Make the call into asyn for flushing the connection */
	status = poctet->flush(octetPvt, pasynUser);
	return status;
}

/*
 * Function: finish
 *
 * Parameters: pr - Pointer to a record structure
 *
 * Returns: void
 * 
 * Description:
 *
 * Completes processing of a record.
 */
static void finish(dbCommon *pr)
{
  devPvt     *pPvt = (devPvt *)pr->dpvt;
  if(pr->pact) callbackRequestProcessCallback(&pPvt->callback,pr->prio,pr);
}

/*
 * Function: initLiRead
 *
 * Parameters: pli - Pointer to a longin record structure
 *
 * Returns: 0
 * 
 * Description:
 *
 * Initialises longin record, registers the process callback function.
 * Initialises the database address and the drvUser structure.
 */
static long initLiRead(longinRecord *pli)
{
  asynStatus status;
  devPvt     *pdevPvt;
  
  status = initCommon((dbCommon *)pli,&pli->inp,callbackLiRead,asynOctetType);
  if(status!=asynSuccess) return 0;
  pdevPvt = (devPvt *)pli->dpvt;
  initDbAddr(pdevPvt);
  initDrvUser(pdevPvt);
  return 0;
}

/*
 * Function: callbackLiRead
 *
 * Parameters: pasynUser - Pointer to the asynUser structure
 *
 * Returns: void
 * 
 * Description:
 *
 * Called from the asynDriver.  Record has processed so call
 * writeIt to issue a GET command and then readIt to get the response.
 */
static void callbackLiRead(asynUser *pasynUser)
{
  devPvt         *pdevPvt = (devPvt *)pasynUser->userPvt;
  longinRecord   *pli = (longinRecord *)pdevPvt->precord;
  asynStatus     status;
  size_t         nBytesRead = 0;
  char           raw[MAX_STRING_SIZE];
  int            localMid = 1;
  int            cmd;
  int            retries = 0;
	int            count = 0;
	int            match = 0;
	int            msgretries = 0;

	/* Create the request data structure */
  UcGetTelegram request = { {
    sizeof( UcGetTelegram ) - sizeof( Int32 ),
    UC_GET,
    0x0000,
    0,
    mid
  } };

	/* Create a union to map the byte response into an acknowledge structure */
  union {
    char	raw[sizeof(UcAckTelegram)];
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
    asynPrint(pasynUser,ASYN_TRACE_ERROR, "callbackLiRead: Failed to get midMutexId lock.\n");
  }
  epicsMutexUnlock(midMutexId);
	asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"%s sending messge ID: %d\n",pli->name,localMid);

	/* Check the INP field is not empty */
  if (!pdevPvt->userParam){
    /* Invalid INP field, cannot construct the command memory location. */
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
	      "%s error, invalid inp\n",pli->name);
    recGblSetSevr(pli,READ_ALARM,INVALID_ALARM);
    finish((dbCommon *)pli);
    return;
  } else {

		/* Read the input string for the memory location */
		sscanf(pdevPvt->userParam, "%x", &cmd);
		/* The input string is the memory address to query */
		request.hdr.address = cmd;

		/* Set the index to the specified link address */
		/* This corresponds to an axis number (or isn't used) */
		request.hdr.index = pdevPvt->addr;

		/* Set the message ID to the incremented value */
		request.hdr.correlationNumber = localMid;

		/* Flush the connection to remove any stale data */
		status = flushIt(pasynUser);

		/* Send the GET request */
		status = writeIt(pasynUser,(char *)&request,sizeof( UcGetTelegram ));
  	if(status==asynSuccess){
			while(match == 0 && msgretries < 1){
	    	while(nBytesRead == 0 && retries < 1){
					/* If write was successful then read back the response. */
					/* This initial value is the size of the following message. */
					status = readIt(pasynUser,raw,sizeof(Int32),&nBytesRead);
					retries++;
  	  	}
        if(status==asynSuccess){
					/* If response was successful then read the rest of the message. */
					/* Set the type of message in the telegram struct */
					for(count = 0; count < nBytesRead; count++){
						tel.raw[count] = raw[count];
					}
					nBytesRead = 0;
					retries = 0;
          if (tel.ack.hdr.length != 24){
            tel.ack.hdr.length = 24;
          }
  	  		while(nBytesRead != tel.ack.hdr.length && retries < 3){
  				  /* Read the remaining bytes of the message */
	  			  status = readIt(pasynUser,raw,tel.ack.hdr.length,&nBytesRead);
		  		  retries++;
  				}
  	  	 	if(status==asynSuccess){
			  		/* Set the rest of the structure by overlaying the message onto it */
				  	for(count = 0; count < nBytesRead; count++){
					  	tel.raw[(count+4)] = raw[count];
					  }
  				}
				}
				/* Check the ID numbers match */
				if (tel.ack.hdr.correlationNumber == localMid && status==asynSuccess){
					match = 1;
					pli->udf = 0;
					pli->val = (epicsInt32)tel.ack.data[0];
					asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"%s raw value read: %d\n",pli->name,tel.ack.data[0]);
					asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"%s read message ID: %d\n",pli->name,tel.ack.hdr.correlationNumber);
				} else {
					msgretries++;
				}
			}					
		}
	}

  /* Finish processing the record. */
  finish((dbCommon *)pli);
}


/*
 * Function: initLoWrite
 *
 * Parameters: plo - Pointer to a longout record structure
 *
 * Returns: 0
 * 
 * Description:
 *
 * Initialises longout record, registers the process callback function.
 * Initialises the database address and the drvUser structure.
 */
static long initLoWrite(longoutRecord *plo)
{
  asynStatus status;
  devPvt     *pdevPvt;
  
  status = initCommon((dbCommon *)plo,&plo->out,callbackLoWrite,asynOctetType);
  if(status!=asynSuccess) return 0;
  pdevPvt = (devPvt *)plo->dpvt;
  initDbAddr(pdevPvt);
  initDrvUser(pdevPvt);
  return 0;
}

/*
 * Function: callbackLoWrite
 *
 * Parameters: pasynUser - Pointer to the asynUser structure
 *
 * Returns: void
 * 
 * Description:
 *
 * Called from the asynDriver.  Record has processed so call
 * writeIt to issue a command.
 */
static void callbackLoWrite(asynUser *pasynUser)
{
  devPvt         *pdevPvt = (devPvt *)pasynUser->userPvt;
  longoutRecord  *plo = (longoutRecord *)pdevPvt->precord;
  asynStatus     status;
  int            cmd = 0;
  int            localMid = 1;
  size_t         nBytesRead = 0;
  char           raw[MAX_STRING_SIZE];
  int            retries = 0;
	int            count = 0;
	int            match = 0;
	int            msgretries = 0;

	/* Create the command data structure */
  UcSetTelegram request =
	{
		{
    	sizeof( UcSetTelegram ) - sizeof( Int32 ),
    	UC_SET,
    	0x0000,
    	0,
    	mid
  	},
		{
			1
		}
	};

	/* Create a union to map the byte response into an acknowledge structure */
  union {
    char	raw[sizeof(UcAckTelegram)];
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
    asynPrint(pasynUser,ASYN_TRACE_ERROR, "callbackLiRead: Failed to get midMutexId lock.\n");
  }
  epicsMutexUnlock(midMutexId);
	asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"%s sending messge ID: %d\n",plo->name,localMid);

	/* Check the INP field is not empty */
  if (!pdevPvt->userParam){
    /* Invalid INP field, cannot construct the command memory location. */
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
	      "%s error, invalid inp\n",plo->name);
    recGblSetSevr(plo,READ_ALARM,INVALID_ALARM);
    finish((dbCommon *)plo);
    return;
  } else {
		/* Read the output string for the memory location */
		sscanf(pdevPvt->userParam, "%x", &cmd);
		/* The input string is the memory address to query */
		request.hdr.address = cmd;
		request.data[0] = (Int32)plo->val;

		/* Set the index to the specified link address */
		request.hdr.index = pdevPvt->addr;
		
		/* Set the message ID to the incremented value */
		request.hdr.correlationNumber = localMid;

		/* Flush the connection to remove any stale data */
	  status = flushIt(pasynUser);

		/* Send the SET command */
		status = writeIt(pasynUser,(char *)&request,sizeof( UcSetTelegram ));

	  if(status==asynSuccess){
			while(match == 0 && msgretries < 1){
		    while(nBytesRead == 0 && retries < 1){
					/* If write was successful then read back the response. */
					/* This first reply contains the size of the following message */
					status = readIt(pasynUser,raw,sizeof(Int32),&nBytesRead);
					retries++;
	  	  }
	  	  if(status==asynSuccess){
					/* If response was successful then read the rest of the message. */
					/* Set the type of message in the telegram struct */
					for(count = 0; count < nBytesRead; count++){
						tel.raw[count] = raw[count];
					}
					nBytesRead = 0;
					retries = 0;
          if (tel.ack.hdr.length != 24){
            tel.ack.hdr.length = 24;
          }
	  	  	while(nBytesRead != tel.ack.hdr.length && retries < 1){
						/* Read the remaining bytes of the message */
						status = readIt(pasynUser,raw,tel.ack.hdr.length,&nBytesRead);
						retries++;
					}
	  	   	if(status==asynSuccess){
						/* Set the rest of the structure by overlaying the message onto it */
						for(count = 0; count < nBytesRead; count++){
							tel.raw[(count+4)] = raw[count];
						}
					}
				}
				/* Check the ID numbers match */
				if (tel.ack.hdr.correlationNumber == localMid && status==asynSuccess){
					match = 1;
					plo->udf = 0;
					asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"%s raw value read: %d\n",plo->name,tel.ack.data[0]);
					asynPrint(pasynUser,ASYN_TRACEIO_FILTER,"%s read message ID: %d\n",plo->name,tel.ack.hdr.correlationNumber);
				} else {
					msgretries++;
				}
			}					
		}
	}

  /* Finish processing the record. */
  finish((dbCommon *)plo);
}


/*
 * Function: initCommon
 *
 * Parameters: precord       - Pointer to the record structure
 *             plink         - Pointer to the DBLINK structure
 *             callback      - Function called when record processed
 *             interfaceType - Pointer to string val of interface type
 *
 * Returns: asynStatus success value
 * 
 * Description:
 *
 * Common initialisation for all records.  Create the asynUser structure.
 * Parse the input for the address number.  Attempt to connect and find 
 * the interface.
 */
long initCommon(dbCommon *precord,
		DBLINK *plink,
		userCallback callback,
		const char *interfaceType)
{
  devPvt        *pdevPvt;
  asynStatus    status;
  asynUser      *pasynUser;
  asynInterface *pasynInterface;
  commonDset    *pdset = (commonDset *)precord->dset;

  pdevPvt = callocMustSucceed(1, sizeof(*pdevPvt), "devAsynCommonReverse");
  precord->dpvt = pdevPvt;
  pdevPvt->precord = precord;

  /* Create the asynUser */
  pasynUser = pasynManager->createAsynUser(callback, 0);
	pasynUser->timeout = 0.1;
  pasynUser->userPvt = pdevPvt;
  pdevPvt->pasynUser = pasynUser;

	/* Create the Mutex for the MID if necessary */
	if (midMutexId == NULL){
    if ((midMutexId = epicsMutexCreate()) == NULL) {
	    asynPrint(pasynUser,ASYN_TRACE_ERROR, "initCommon: Could not create midMutexId.\n");
    }
	}

  /*
   * Parse the link for the Port number.  Should be IO_INST and start
   * with @<port> S<n> <user info>.
   */
  status = parseLink(pasynUser,
		     plink,
		     &pdevPvt->portName,
		     &pdevPvt->addr,
		     &pdevPvt->userParam);

  if (status != asynSuccess){
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
	  	"%s devAsynCommonReverse error in link %s\n",
	  	precord->name,
			pasynUser->errorMessage);
    goto bad;
  }

  /* Connect to the device */
  status = pasynManager->connectDevice(pasynUser,
				       pdevPvt->portName,
				       pdevPvt->addr);

  if (status != asynSuccess){
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
			"%s devAsynCommonReverse connectDevice failed %s\n",
			precord->name,
			pasynUser->errorMessage);
		goto bad;
  }

  /* Find and set any interfaces */
  pasynInterface = pasynManager->findInterface(pasynUser, interfaceType, 1);
  if (!pasynInterface){
    asynPrint(pasynUser,ASYN_TRACE_ERROR,
			"%s devAsynCommonReverse interface %s not found\n",
			precord->name,
			interfaceType);
    goto bad;
  }

  if (strcmp(interfaceType, asynOctetType) == 0){
      pdevPvt->poctet = pasynInterface->pinterface;
  } else if (strcmp(interfaceType, asynInt32Type) == 0){
      pdevPvt->pint32 = pasynInterface->pinterface;
  } else if (strcmp(interfaceType, asynFloat64Type) == 0){
      pdevPvt->pfloat64 = pasynInterface->pinterface;
  }

  pdevPvt->interfacePvt = pasynInterface->drvPvt;

  /* Determine if device can block */
  pasynManager->canBlock(pasynUser, &pdevPvt->canBlock);
  if (pdset->get_ioint_info){
    scanIoInit(&pdevPvt->ioScanPvt);
  }

  return 0;

	bad:
  	precord->pact = 1;
  	return -1;
}

/*
 * Function: initDrvUser
 *
 * Parameters: pdevPvt - Pointer to the device structure
 *
 * Returns: void
 * 
 * Description:
 *
 * Initialises the drvUser structure.
 */
void initDrvUser(devPvt *pdevPvt)
{
  asynUser *pasynUser = pdevPvt->pasynUser;
  asynStatus status;
  asynInterface *pasynInterface;
  dbCommon *precord = pdevPvt->precord;

  pasynInterface = pasynManager->findInterface(pasynUser, asynDrvUserType, 1);
  if (pasynInterface && pdevPvt->userParam){
    asynDrvUser *pasynDrvUser;
    void *drvPvt;

    pasynDrvUser = (asynDrvUser *)pasynInterface->pinterface;
    drvPvt = pasynInterface->drvPvt;
    status = pasynDrvUser->create(drvPvt, pasynUser, pdevPvt->userParam, 0, 0);
    if (status != asynSuccess){
      asynPrint(pasynUser,ASYN_TRACE_ERROR,
				"%s devAnc350 drvUserCreate failed %s\n",
				precord->name,
				pasynUser->errorMessage);
    }
  }
}

/*
 * Function: initDrvUser
 *
 * Parameters: pdevPvt - Pointer to the device structure
 *
 * Returns: void
 * 
 * Description:
 *
 * Initialises the drvUser structure.
 */
void initDbAddr(devPvt *pdevPvt)
{
  char userParam[64];
  dbCommon *precord = pdevPvt->precord;

  strcpy(userParam, precord->name);
  strcat(userParam, ".VAL");

  if (dbNameToAddr(userParam, &pdevPvt->dbAddr)){
    asynPrint(pdevPvt->pasynUser,ASYN_TRACE_ERROR,
			"%s devAsynCommonReverse record %s not present\n",
			precord->name,
			userParam);
    precord->pact = 1;
  }
}

/*
 * Function: processCommon
 *
 * Parameters: precord - Pointer to a record structure
 *
 * Returns: long status.
 * 
 * Description:
 *
 * This function is called whenever one of the records is processed.
 * The callback request is queued so that the whole system doesn't 
 * block.
 */
long processCommon(dbCommon *precord)
{
  devPvt *pdevPvt = (devPvt *)precord->dpvt;
  asynStatus status;

  if (!pdevPvt->gotValue && precord->pact == 0){
    if (pdevPvt->canBlock) precord->pact = 1;
    /* Request the callback be put on the queue */
    status = pasynManager->queueRequest(pdevPvt->pasynUser,
					asynQueuePriorityMedium,
					0.0);
    if ((status == asynSuccess) && pdevPvt->canBlock) return 0;
    if (pdevPvt->canBlock) precord->pact = 0;
    if (status != asynSuccess){
      /* Bad status, the queueing failed, raise an error */
      asynPrint(pdevPvt->pasynUser, ASYN_TRACE_ERROR,
				"%s devAsynCommonReverse error queuing request %s\n",
				precord->name,
				pdevPvt->pasynUser->errorMessage);
      recGblSetSevr(precord, READ_ALARM, INVALID_ALARM);
    }
  }  
  if (!strcmp("ai", precord->rdes->name)){
    return 2;
  }
  return 0;
}

/*
 * Function: parseLink
 *
 * Parameters: pasynUser - pointer to the asynUser structure
 *             plink     - pointer to a DBLINK structure
 *             port      - pointer to pointer of a string
 *             addr      - unused
 *             userParam - pointer to pointer of user param string
 *
 * Returns: asynStatus
 * 
 * Description:
 *
 * Breaks down the user input string into its various components.
 * Constructs the name of the port, either PMAC_PORT_Cn or 
 * PMAC_DPRAM_Cn where n is an integer representing the pmac card no.
 * PORT is for ASCII comms and DPRAM is for DPRAM read/write etc.
 * The remainder of the string (which might contain DPRAM address or
 * ASCII command) is stored in the userParam string.
 */
asynStatus parseLink(asynUser *pasynUser,
		     DBLINK *plink,
		     char **port,
		     int *addr,
		     char **userParam)
{
  struct instio *pinstio;
  int len;
  char *p;
  char *pnext;

  assert(addr && port && userParam);
  *addr = 0;
  *port = NULL;
  *userParam = NULL;

  /* Determine type of link, we are only interested in INST_IO */
  switch (plink->type)
    {
    case INST_IO:
      pinstio = (struct instio*)&(plink->value);
			p = pinstio->string;
			pnext = p;
      /* The next part should contain the port */
      for (len = 0; *p && !isspace(*p) && (*p != ','); len++, p++){}
      if (*p == 0) goto error;
      /* Allocate enough memory to hold the full port name */
      *port = mallocMustSucceed(len+1, "devAsynCommonReverse");
      (*port)[len] = 0;
      strncpy(*port, pnext, len);

      /* Search for an S, this represents start of the address */
      pnext = strchr(p, 'S');
			pnext++;
      if (!pnext) goto error;
      p = pnext;
      /* The next part should contain the address number */
      for (len = 0; *p && !isspace(*p) && (*p != ','); len++, p++){}
      if (*p == 0) goto error;
			sscanf(pnext, "%d", addr);

      /* Rest of string can be considered user params */
      if (userParam) *userParam = 0;
      p++;
      if (*p){
				p = skipWhite(p, 0);
				if (userParam && *p){
	  			len = strlen(p);
	  			*userParam = mallocMustSucceed(len+1, "devAsynCommonPmac");
					strncpy(*userParam, p, len);
					(*userParam)[len] = 0;
				}
			}
      break;

    error:
      /* We've hit an error so print out an explanation of why */
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		    "invalid INST_IO Must be #C<port no> userParams");
      return asynError;
      break;

    default:
      /* This is no good we want an INST_IO */
      epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
		    "Link must be INST_IO");
      return asynError;
      break;
    }

  return asynSuccess;
}

/*
 * Function: skipWhite
 *
 * Parameters: pstart  - pointer to start of string we a searching through
 *             commaOk - flag to say if we count commas or not
 *
 * Returns: pointer to a character
 * 
 * Description:
 *
 * Simple helper function to skip through any whitespace and optionally
 * commas.
 */
static char *skipWhite(char *pstart, int commaOk)
{
  char *p = pstart;
  while (*p && (isspace(*p) || (commaOk && (*p==',')))) p++;
  return p;
}


