/*
 * File:   SPI.c
 * Author: Quentin BOYER
 *
 * Created on 31 octobre 2017, 21:10
 * 
 * Notes : Parfois des erreurs lors de l'envoi de plusieurs octets en meme temps depui la RPi, à vérifier en augmentant la clock
 */

#include "SPI.h"

volatile unsigned char j = 0;

void initSPI() {
    initSPI1();
}

void initSPI1(){
    IEC0bits.SPI1EIE = 0;       //Disable SPI1 Error interrupt
    IEC0bits.SPI1IE = 0;        //Disable SPI1 Transfer done interrupt
    
    SPI1STATbits.SPIEN = 0;     //Disable the module

    TRISCbits.TRISC3 = 1;           //SCK
    TRISAbits.TRISA9 = 1;           //SDI   (MOSI)
    TRISAbits.TRISA4 = 0;           //SD0   (MISO)
    TRISBbits.TRISB0 = 1;           //SS
    
    ANSELAbits.ANSA9 = 0;
    ANSELCbits.ANSC3 = 0;
    ANSELBbits.ANSB0 = 0;
    
    SPI1STATbits.SPISIDL = 0;   //Continues the module operation in Idle mode
    SPI1STATbits.SISEL = 0b011; //Interrupt when the SPIx receive buffer is full (SPIRBF bit is set)
    
    SPI1CON1bits.DISSDO = 0;    //SDOx pin is controlled by the module
    SPI1CON1bits.MODE16 = 0;    //Communication is byte-wide (8 bits)
    SPI1CON1bits.SMP = 0;       //SMP must be cleared when SPIx is used in Slave mode.
    SPI1CON1bits.CKE = 1;       //Serial output data changes on transition from active clock state to Idle clock state
    SPI1CON1bits.SSEN = 1;      //SSx pin is used for Slave mode
    SPI1CON1bits.CKP = 0;       //Idle state for clock is a low level; active state is a high level
    SPI1CON1bits.MSTEN = 0;     //Slave mode    
    
    SPI1CON2bits.FRMEN = 0;     //Framed SPIx support is disabled
    SPI1CON2bits.SPIBEN = 0;    //Enhanced Buffer is disabled (Standard mode)
    
    IEC0bits.SPI1IE = 1;        //Enable interrupt
    IFS0bits.SPI1IF = 0;        //Clear flag
    
    
    SPI1STATbits.SPIEN = 1;     //Enable the module
}

void __attribute__((__interrupt__,no_auto_psv)) _SPI1Interrupt(void)
{
    unsigned char res = SPI1BUF;
    if(res == 0xAB)
        j = 'T';
    else{
        switch(j){
            case 'T' :
                j = 'h';
                break;
            case 'h' :
                j = 'e';
                break;
            case 'e' :
                j = ' ';
                break;
            case ' ' :
                j = 'G';
                break;
            case 'G' :
                j = 'a';
                break;
            case 'a' :
                j = 'm';
                break;
            case 'm' :
                j = 'e';
                break;
        }
    }
    SPI1BUF = j;
    //i++;
    IFS0bits.SPI1IF = 0;
}
