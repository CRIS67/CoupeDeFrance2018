/* 
 * File: interrupt.h  
 * Author: Quentin BOYER
 * Comments: check most appropriate variable type <-> speed of execution 
 *              optimization : remove prevProcessVariable
 * Revision history: 1
 */

#ifndef INTERRUPT_H
#define	INTERRUPT_H

#include <xc.h> // include processor files - each processor file is guarded.  

#define _ISR_PSV        __attribute__((__interrupt__, __auto_psv__))
//#define _ISR_PSV_FAST   __attribute__((interrupt(auto_psv, shadow)))

void initInt();


#endif	/* INTERRUPT_H */