/* 
 * File: timer.h  
 * Author: Quentin BOYER
 * Comments: 
 * Revision history: 1
 */

#ifndef TIMER_H
#define	TIMER_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <math.h>
#include "constant.h"
#include "PID.h"
#include "GPIO.h"
#include "PWM.h"
#include "UART.h"


void initTimer();
void initTimer1();
void initTimer2();
void initTimer3();
void initTimer4();
void delay_us(unsigned int delay);
void delay_ms(unsigned int delay);

#endif	/* TIMER_H */