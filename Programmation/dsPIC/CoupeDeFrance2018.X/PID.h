/* 
 * File: PID.h  
 * Author: Quentin BOYER
 * Comments: check most appropriate variable type <-> speed of execution 
 *              optimization : remove prevProcessVariable
 * Revision history: 1
 */

#ifndef PID_H
#define	PID_H

#include <xc.h> // include processor files - each processor file is guarded. 
#include "constant.h"

typedef struct PID PID;
struct PID
{
    double Kp;
    double Ki;
    double Kd;
    double bias;
    double setPoint;            //consigne
    double processVariable;     //entree
    //double prevProcessVariable;
    double prevError;
    double sumI;
    double period;               //intervalle asserv
    double output;
};

void initAllPID(PID *pidSpeedLeft, PID *pidSpeedRight, PID *pidDistance, PID *pidAngle);
void initPID(PID *pid, double Kp, double Ki, double Kd,double bias, double output, double period, double processVariable, double setPoint);
void setSetPoint(PID *pid, double setPoint);
void setProcessValue(PID *pid, double processVariable);
double compute(PID *pid, double processVariable);

#endif	/* PID_H */