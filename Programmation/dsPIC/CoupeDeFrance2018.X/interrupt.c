/*
 * File:   interrupt.c
 * Author: Quentin BOYER
 *
 * Created on 28 octobre 2017, 16:51
 */

#include "interrupt.h"

volatile int i = 0;

void initInt() {
    INTCON1 = 0x8000;    //Disable traps and interrupt nesting
    INTCON2 = 0;    //Disable interruptions and traps and internal interrupt occurs on positive edge
    INTCON2bits.GIE = 1;
}

/*void _ISR_PSV _INT0Interrupt(){   External interrupt
    
}*/