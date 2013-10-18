/******************************************************************************
 *
 *  Project:        NHands Controller Interface
 *
 *  Filename:       anc350.h
 *
 *  Purpose:        Control Protocol Constants for ANC350
 *
 *  Author:         NHands GmbH & Co KG
 */
/*****************************************************************************/
/** @file anc350.h
 *  @brief Control Protocol Constants for ANC350
 *
 *  Product specific extension of the generic ncore interface.
 *  Defines constants to be used as parameters or parameter limits
 *  for the control protocol of ucprotocol.h .
 */
/*****************************************************************************/
/* $Id: anc350.h,v 1.1 2009/04/14 09:10:01 ajg Exp $ */

#ifndef __ANC350_H
#define __ANC350_H


/** @name Maximum axis index
 *
 *  Most of the addresses are specific to an axis or trigger. This object is selected
 *  by the index of the used address.
 *  The maximum index must be maintained.
 *  
 *  @{
 */
#define ANC_MAX_AXIS         0x06    /**< Maximum index for selecting an axis   */
#define ANC_MAX_TRIGGER      0x05    /**< Maximum index for selecting a trigger */
/* @} */



/** @name Control of Events
 *
 *  The following address controls the sending of asynchronous events.
 *  To increase protocol performance some values (e.g. ID_ANC_COUNTER) 
 *  are sent as events. 
 *  After successfully connecting to the controller the events should
 *  be activated.
 *  A value of 0 deactivates the events, a value of 1 activates the events.
 *  
 *  @{
 */
#define ID_ASYNC_EN             0x0145   /**< Controls sending of events   */
/* @} */

/** @name Axis related status Information
 *
 *  The following address (ID_...)is read only, set functions will fail.
 *  It provides information about the current states and is sent
 *  periodically by the controller. Events must be activated by @ref ID_ASYNC_EN.
 *  The appropriate axis is indicated by the index.
 *  The data field of the telegram contains the states encoded as
 *  a bit field; bit masks are ANC_STATUS_...
 *  Bit 0:  Actor running
 *  Bit 1:  Hump detected
 *  Bit 8:  Sensor error
 *  Bit 10: Sensor disconnected
 *  Bit 11: Reference valid
 *  Bit 12: Sensor disable
 *  
 *  @{
 */
#define ID_ANC_STATUS           0x0404    /**< States of the appropriate axis   */
#define ANC_STATUS_RUNNING      0x0001    /**< Bitmask actor running            */
#define ANC_STATUS_HUMP         0x0002    /**< Bitmask hump detected            */
#define ANC_STATUS_SENS_ERR     0x0100    /**< Bitmask sensor error             */
#define ANC_STATUS_DISCONN      0x0400    /**< Bitmask sensor disconnected      */
#define ANC_STATUS_REF_VALID    0x0800    /**< Bitmask reference valid          */
#define ANC_STATUS_ENABLE       0x1000    /**< Bitmask sensor enabled           */
/* @} */

/** @name Temperature status information
 *
 *  The following address is read only, set functions will fail.
 *  It provides information about the current temperature status and is sent
 *  periodically by the controller. Events must be activated by @ref ID_ASYNC_EN.
 *  A value of 0 indicates an overtemtemperature situation, a value of 1
 *  indicates temperature is ok. The temparature status is provided globally and
 *  not for each axis separately. So only index 0 is allowed.
 *  
 *  @{
 */
#define ID_ANC_TEMP_STATUS   0x0560    /**< Temperature status                */
/* @} */


/** @name Position Information
 *
 *  The following address is read only, set functions will fail.
 *  It provides information about the current positions and is sent
 *  periodically by the controller. The values can be retreived with
 *  get functions.
 *  Events must be activated by @ref ID_ASYNC_EN.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_COUNTER        0x0415     /**< Position of the appropriate axis.
                                              Unit is @ref ID_ANC_UNIT.
                                              Value is scaled with factor 1000. */
#define ID_ANC_ROTCOUNT       0x0516      /**< Count of rotations for position
                                              in case of rotator as actor, event only       */
#define ID_ANC_REFCOUNTER     0x0407     /**< Reference position of the appropriate axis.
                                              Unit is @ref ID_ANC_UNIT.
                                              Value is scaled with factor 1000. */
#define ID_ANC_REFROTCOUNT    0x0517      /**< Count of rotations for reference position
                                              in case of rotator as actor, event only */
#define ID_ANC_LEFT_LIMIT     0x0441     /**< Minimum position for position limited actors.
                                              Unit is @ref ID_ANC_UNIT.
                                              Value is scaled with factor 1000. */
#define ID_ANC_RIGHT_LIMIT    0x0442     /**< Maximum position for position limited actors.
                                              Unit is @ref ID_ANC_UNIT.
                                              Value is scaled with factor 1000. */
/* @} */


/** @name Positioning
 *
 *  The following addresses are used for actor positioning.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_POS_RESET      0x044F     /**< Executes a reset of the position. */
#define ID_ANC_TARGET         0x0408     /**< Defines the target position.
                                              Unit is @ref ID_ANC_UNIT.
                                              Value is scaled with factor 1000. */
#define ID_ANC_TGTROTCNT      0x0518      /**< Defines the count of rotations for the target position */
#define ID_ANC_RUN_TARGET     0x040D     /**< Starts approach to absolute target position.
                                              Previous movement will be stopped */
#define ID_ANC_RUN_RELATIVE   0x0418     /**< Starts approach to relative target position.
                                              Previous movement will be stopped. */
#define ID_ANC_MOVE_REF       0x0444     /**< Starts approach to reference position.
                                              Previous movement will be stopped. */
/* @} */


/** @name Manual positioning
 *
 *  The following addresses are used for manual actor positioning.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_SGL_FWD        0x0410     /**< Starts a one step positioning in forward direction,
                                              Previous movement will be stopped. */
#define ID_ANC_SGL_BKWD       0x0411     /**< Starts a one step positioning in backward direction,
                                              Previous movement will be stopped. */
#define ID_ANC_CONT_FWD       0x040E     /**< Starts continously positioning in forward direction
                                              with set parameters for amplitude and speed and
                                              amplitude control, respectively.   */
#define ID_ANC_CONT_BKWD      0x040F     /**< Starts continously positioning in backward direction
                                              with set parameters for amplitude and speed and
                                              amplitude control, respectively.   */
/* @} */


/** @name Positioning parameters
 *
 *  The following addresses are used for adjusting positioning parameters.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_AMPL           0x0400     /**< Sets the amplitude in [mV] for the actor.
                                              In case of movement the amplitude set by amplitude
                                              control is determined and sent
                                              periodically by the controller.    */
#define ID_ANC_REGSPD_SETP    0x0542     /**< Represents the speed of the actor in @ref ID_ANC_UNIT / s.
                                              The value is scaled with factor 1000.
                                              This address is read only and is sent
                                              periodically by the controller.  */
#define ID_ANC_REGSPD_SETPS   0x0549     /**< Represents the step width of the actor in @ref ID_ANC_UNIT.
                                              The value is scaled with factor 1000.
                                              This address is read only and is sent
                                              periodically by the controller     */
#define ID_ANC_ACT_AMPL       0x0514     /**< Sets the DC level in [mV] of the actor. */
#define ID_ANC_FAST_FREQ      0x0401     /**< Sets the frequency in [Hz] of the excitation signal. */
#define ID_ANC_RELAIS         0x0447     /**< Switches the output relais of the amplifier. */
/* @} */


/** @name Capacity measurement
 *
 *  The following addresses are used for capacity measurement.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_CAP_START      0x051E     /**< Starts the capacity measurement    */
#define ID_ANC_CAP_VALUE      0x0569     /**< Represents the result of the capacity measurement.
                                              This address is read only and is sent by the controller
                                              on finished measurement            */
/* @} */


/** @name Reference voltage for resistive sensors
 *
 *  The following address is used for setting the reference voltage for resistive sensors.
 *  The voltage is set globally for all axes, only index 0 is valid.
 *  
 *  @{
 */
#define ID_ANC_SENSOR_VOLT    0x0526     /**< Reference voltage in [mV] */
/* @} */


/** @name Persistence control of parameters
 *
 *  The following address is used for saving parameters to controller flash or clearing the flash.
 *  Only index 0 is valid.
 *  
 *  @{
 */
#define ID_ANC_ACTORPS_SAVE   0x050C     /**< A data value of "1234" saves all set parameters to
                                              controller flash. A data value of "4321" clears all
                                              parameters from flash.             */
/* @} */


/** @name Trigger parameters
 *
 *  The following addresses are used for adjusting trigger parameters.
 *  The appropriate trigger number is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_TRG_LOW        0x0530     /**< Lower trigger threshold position.
                                              Unit is @ref ID_ANC_TRG_UNIT.
                                              Value is scaled with factor 1000.  */
#define ID_ANC_TRG_HIGH       0x0531     /**< Upper trigger threshold position.
                                              Unit is @ref ID_ANC_TRG_UNIT.
                                              Value is scaled with factor 1000.  */
#define ID_ANC_TRG_POL        0x0532     /**< Trigger polarity                   */
#define ID_ANC_TRG_AXIS       0x0533     /**< Number of assigned axis            */
#define ID_ANC_TRG_EPS        0x0534     /**< Epsilon, unit is @ref ID_ANC_TRG_UNIT.
                                              Value is scaled with factor 1000.  */
#define ID_ANC_TRG_UNIT       0x0535     /**< Unit of trigger, This address is read only and
                                              is sent by the controller in case of changing
                                              the assigned axis and its unit     */
/* @} */


/** @name Scanner and dither module specific
 *
 *  The following addresses are used for adjusting scanner
 *  and dither module specific parameters.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_BW_LIMIT       0x0568     /**< Switches the bandwith limitation    */
#define ID_ANC_DCIN_EN        0x0561     /**< Switches the DC in connector        */
#define ID_ANC_INT_EN         0x0563     /**< Switches the internal connection to the amplifier */
#define ID_ANC_ACIN_EN        0x0562     /**< Switches the AC in connector, only valid for dither axes */
/* @} */


/** @name Actor specific paramters
 *
 *  The following addresses are used for adjusting actor specific parameters.
 *  The parameters are supplied as specific *.aps files with simple syntax:
 *  paramter name value.
 *  The appropriate axis is indicated by the index.
 *  
 *  @{
 */
#define ID_ANC_DIST_SLOW      0x0554     /**< APS name 'poslooprange'.
                                              Position loop range in @ref ID_ANC_UNIT.
                                              The value is scaled with factor 1000.  */
#define ID_ANC_SPD_GAIN       0x054B     /**< APS name 'speedgain'. Gain of approach speed function in 1/s.
                                              The value is scaled with factor 1000.  */
#define ID_ANC_SPD_ENABLE     0x054C     /**< APS name 'adaptsetpctrl'. Enables approach speed function */
#define ID_ANC_LOOP_OFFS      0x054D     /**< APS name 'actoroffset'. [mV]       */
#define ID_ANC_LOOP_GAIN      0x054E     /**< APS name 'actorgain'. @ref ID_ANC_UNIT / V.
                                              The value is scaled with factor 1000000. */
#define ID_ANC_MAX_AMP        0x054F     /**< APS name 'maxampl'. Maximum amplitude in [mV] */
#define ID_ANC_SEN_DIR        0x0551     /**< APS name 'sensordir'. Sensor direction, 0: Forward; 1: Backward */
#define ID_ANC_PERIOD         0x0553     /**< APS name 'period'. Number of periods per @ref ID_ANC_UNIT
                                              for optical sensors                */
#define ID_ANC_REGSPD_AVG     0x0544     /**< APS name 'amplctrlavg'. Average factor for speed feedback */
#define ID_ANC_REGPOS_AVG     0x0545     /**< APS name 'targetctrlavg'. Average factor for position feedback */
#define ID_ANC_REGSPD_KI      0x053F     /**< APS name 'amplctrlsensitivity'. Sensitivity for speed feedback.
                                              The value is scaled with factor 1000. */
#define ID_ANC_REGPOS_KP      0x0540     /**< APS name 'targetctrlsensitivity'. Sensitivity for position feedback.
                                              The value is scaled with factor 1000. */
#define ID_ANC_SLOW_SPEED     0x053D     /**< APS name 'slowspeed'. Actor speed for target approach
                                              in @ref ID_ANC_UNIT / s. The value is scaled with factor 1000000. */
#define ID_ANC_ACTOR_DIR      0x053A     /**< APS name 'actordir'. Actor direction, 0: Forward; 1: Backward */
#define ID_ANC_SCALE_MODE     0x0539     /**< APS name 'transfertype'. Type of sensor, 0: optical; 1: resistive */
#define ID_ANC_RES_ANGLEMIN   0x0559     /**< APS name 'positionmin'. Minimum position of the sensor
                                              in @ref ID_ANC_UNIT. The value is scaled with factor 1000. */
#define ID_ANC_RES_ANGLEMAX   0x055A     /**< APS name 'positionmax'. Maximum position of the sensor
                                              in @ref ID_ANC_UNIT. The value is scaled with factor 1000. */
#define ID_ANC_SENSOR_GAIN    0x0527     /**< APS name 'transfergain'. Gain for resistive transfer function
                                              @ref ID_ANC_UNIT / V. The value is scaled with factor 1000. */
#define ID_ANC_MAX_FREQU      0x0515     /**< APS name 'maxfrequ'. Maximum frequency for actor in [Hz] */
#define ID_ANC_ACT_ROTARY     0x0452     /**< APS name 'rotary'. 0: Actor is linear; 1: Actor is rotary  */
#define ID_ANC_SGLCIRCLE      0x0519     /**< APS name 'singlecircle'. Shortest way algorithm for rotary actors  */
#define ID_ANC_STOP_EN        0x0450     /**< APS name 'humpenable'. Enables hump detection  */
#define ID_ANC_UNIT           0x041D     /**< APS name 'sensorunit'. Sets unit for the sensor. See ID_ANC_UNIT_...  */
#define ID_ANC_SEN_AVG        0x0558     /**< APS name 'sensoravg'. Sets the sensor average factor for the sensor. */
#define ID_ANC_REGSPD_SELSP   0x054A     /**< APS name 'amplctrl'. Sets the type of setpoint for the speed feedback.
                                              0: Speed; 1: Amplitude; 2: Step Width */
#define ID_ANC_DIST_STOP      0x0555     /**< APS name 'targetrange'. Positioning accuracy in @ref ID_ANC_UNIT.
                                              The value is scaled with factor 1000. */
#define ID_ANC_TARGET_TIME    0x044B     /**< APS name 'targettime'. Minimum duration in [ms] of holding target
                                              position for successful target approach. */
#define ID_ANC_REF_OFFS       0x053B     /**< APS name 'refoffset'. Reference offset in @ref ID_ANC_UNIT.
                                              The value is scaled with factor 1000. */
#define ID_ANC_SENSOR_RES     0x0567     /**< APS name 'sensorres'. Internal averaging of the sensor signal */

/* @} */


/** @name Units
 *
 *  The following constants encode units for use as data values with @ref ID_ANC_UNIT.
 *  
 *  @{
 */
#define ANC_UNIT_MM             0x00       /**< Sensor unit mm */
#define ANC_UNIT_UM             0x01       /**< Sensor unit um */
#define ANC_UNIT_NM             0x02       /**< Sensor unit nm */
#define ANC_UNIT_PM             0x03       /**< Sensor unit pm */
#define ANC_UNIT_DEG            0x14       /**< Sensor unit deg */
#define ANC_UNIT_MDEG           0x15       /**< Sensor unit mdeg */
#define ANC_UNIT_UDEG           0x16       /**< Sensor unit udeg */
/* @} */


#endif
