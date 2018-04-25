/*
 * File:   GPIO.c
 * Author: Quentin BOYER
 *
 * Created on September 18, 2017, 11:23 PM
 */

#include <p33EP512GM310.h>

#include "GPIO.h"

void initGPIO() {
    
    //1 -> input / 0 -> output
    TRISA = 0xFFFF;
    TRISB = 0xFFFF;
    TRISC = 0xFFFF;
    TRISD = 0xFFFF;
    TRISE = 0xFFFF;
    TRISF = 0xFFFF;
    TRISG = 0xFFFF;

    TRISDbits.TRISD1 = 0;   //PWM_L
    TRISDbits.TRISD2 = 0;   //PWM_R
    TRISEbits.TRISE8 = 0;   //SENS_L
    TRISEbits.TRISE9 = 0;   //SENS_R    
    TRISGbits.TRISG14 = 0;  //LED rupt
    //TRISGbits.TRISG12 = 1;  //rupt_1
    //TRISGbits.TRISG13 = 1;  //rupt_2
    
    //1 -> analog / 0 -> digital
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    ANSELC = 0x0000;
    ANSELD = 0x0000;
    ANSELE = 0x0000;
    ANSELF = 0x0000;
    ANSELG = 0xFFFF;
    //1 -> open-drain
    ODCA = 0x0000;
    ODCB = 0x0000;
    ODCC = 0x0000;
    ODCD = 0x0000;
    ODCE = 0x0000;
    ODCF = 0x0000;
    ODCG = 0x0000;
    //1 -> pull-up resistor
    CNPUA = 0x0000;
    CNPUB = 0x0000;
    CNPUC = 0x0000;
    CNPUD = 0x0000;
    CNPUE = 0x0000;
    CNPUF = 0x0000;
    CNPUG = 0x0000;
    //1 -> pull-down resistor
    CNPDA = 0x0000;
    CNPDB = 0x0000;
    CNPDC = 0x0000;
    CNPDD = 0x0000;
    CNPDE = 0x0000;
    CNPDF = 0x0000;
    CNPDG = 0x0000;
    //1 -> genere an interrupt request on a Change-of-State
    CNENA = 0x0000;
    CNENB = 0x0000;
    CNENC = 0x0000;
    CNEND = 0x0000;
    CNENE = 0x0000;
    CNENF = 0x0000;
    CNENG = 0x0000;
    CNENGbits.CNIEG12 = 1;
    CNENGbits.CNIEG13 = 1;
    //1 -> High(3.3V) or 0V(open-drain) / 0 -> Low(0V) or high-impedance(open-drain)
    LATA = 0x0000;
    LATB = 0x0000;
    LATC = 0x0000;
    LATD = 0x0000;
    LATE = 0x0000;
    LATF = 0x0000;
    LATG = 0x0000;
}
