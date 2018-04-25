/*
 * File:   interrupt.c
 * Author: Quentin BOYER
 *
 * Created on 28 octobre 2017, 16:51
 */

#include <p33EP512GM310.h>

#include "interrupt.h"

volatile int i = 0;
volatile unsigned int saveG;
//volatile SavePort saveG;
void initInt() {
    INTCON1 = 0x8000;           //Disable traps and interrupt nesting
    INTCON2 = 0;                //Disable interruptions and traps and internal interrupt occurs on positive edge
    saveG = PORTG;
    IEC1bits.CNIE = 1;          //Enable Input Change Interrupt
    INTCON2bits.GIE = 1;        //Enable Global Interrupt
}
void _ISR_PSV _CNInterrupt(){   //Change Notification interrupt
    IFS1bits.CNIF = 0;          // Clear CN interrupt
    pinCN p = DEFAULT;
    if((saveG & (1 << 12)) != PORTGbits.RG12)
        p = RUPT_1;
    else if((saveG & (1 << 13)) != PORTGbits.RG13)
        p = RUPT_2;
    switch(p){
        case RUPT_1:
            led = 1;
            break;
        case RUPT_2:
            led = 0;
            break;
        default:
            break;
    }
    saveG = PORTG;
}
/*void _ISR_PSV _INT0Interrupt(){   External interrupt
    
}*/