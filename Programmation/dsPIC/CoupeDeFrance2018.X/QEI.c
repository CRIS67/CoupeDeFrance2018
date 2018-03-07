/*
 * File:   QEI.c
 * Author: Quentin BOYER
 *
 * Created on 26 octobre 2017, 22:03
 * 
 * Notes:
 *  TODO :  -Use digital filter
 *          -Use index reset position + configure interrupt for a "turn" counter
 *          
 */

#include "QEI.h"

void initQEI() {
    initQEIPPS();
    initQEI1();
    initQEI2();
}

void initQEIPPS(){
    
    //TRISG = 0xFFFF;
    TRISGbits.TRISG6 = 1;
    TRISGbits.TRISG7 = 1;
    TRISGbits.TRISG8 = 1;
    TRISGbits.TRISG9 = 1;
    
    //ANSELG = 0;
    ANSELGbits.ANSG6 = 0;
	ANSELGbits.ANSG7 = 0;
    ANSELGbits.ANSG8 = 0;
    ANSELGbits.ANSG9 = 0;
    
	RPINR14bits.QEA1R = 0b1110110;  //QEI1A -> RPI118 (pin10)
    RPINR14bits.QEB1R = 0b1111000;  //QEI1B -> RPI120 (pin12)
    
    RPINR16bits.QEA2R = 0b1110111;  //QEI2A -> RPI119 (pin11)
    RPINR16bits.QEB2R = 0b1111001;  //QEI2B -> RPI121 (pin14)
    //RPINR17bits.INDX2R= 0b0000000;  //QEI1Index -> 
	
	
}

void initQEI1(){
    QEI1CONbits.CCM = 0b00;     //Quadrature Encoder Interface (x4 mode) Count mode is selected
    QEI1CONbits.GATEN = 0;      //External gate signal does not affect position counter/timer operation
    QEI1CONbits.CNTPOL = 1;     //Counter direction is NEGATIVE unless modified by external up/down signal
    QEI1CONbits.INTDIV = 0b000; //1:1 prescale value
    QEI1CONbits.PIMOD = 0b000;  //Index input event does not affect position counter
    QEI1CONbits.QEIEN = 1;      //Module counters are enabled
    
    //QEI1IOC = 0;
    QEI1IOCbits.FLTREN = 1;     //Input pin digital filter is enabled
    
    QEI1STAT = 0;
    
}

void initQEI2(){
    QEI2CONbits.CCM = 0b00;     //Quadrature Encoder Interface (x4 mode) Count mode is selected
    QEI2CONbits.GATEN = 0;      //External gate signal does not affect position counter/timer operation
    QEI2CONbits.CNTPOL = 0;     //Counter direction is positive unless modified by external up/down signal
    QEI2CONbits.INTDIV = 0b000; //1:1 prescale value
    QEI2CONbits.PIMOD = 0b000;  //Index input event does not affect position counter
    QEI2CONbits.QEIEN = 1;      //Module counters are enabled
    
    //QEI1IOC = 0;
    QEI2IOCbits.FLTREN = 1;     //Input pin digital filter is enabled
    
    QEI2STAT = 0;
}