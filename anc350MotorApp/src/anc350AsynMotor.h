#ifndef ANC350_ASYN_MOTOR_H
#define ANC350_ASYN_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

int anc350AsynMotorCreate( char *port, int addr, int card, int nAxes );

#ifdef __cplusplus
}
#endif
#endif
