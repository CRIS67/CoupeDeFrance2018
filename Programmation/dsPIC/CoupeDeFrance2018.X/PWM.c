/*
 * File:   PWM.c
 * Author: Quentin BOYER
 *
 * Note : WARNING : PWMLOCK configuration bit must be clear (or add unlock sequences)
 * Created on 19 septembre 2017, 23:34
 */

#include "PWM.h"

void initPWM(){
    /*initPWM1();
    initPWM2();
    initPWM3();
    initPWM4();*/
    initPWM5();
    //initPWM6();
    PTCON2 = 0x0000;    //1:1 Prescaler -> Maximum PWM resolution
    PTCON = 0x8000;     //Enable PWM Module
}
void initPWM1(){        //20KHz
    PHASE1 = 7000;      //Period of PWM1H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / (20kHz * 1) = 7000 or wih Fosc = 138.24MHz -> 6912
    SPHASE1 = 7000;     //Period of PWM1L
    PDC1 = 4000;           //Duty cycle of PWM1H
    SDC1 = 4000;           //Duty cycle of PWM1L
    DTR1 = 0;           //Dead Time
    ALTDTR1 = 0;
    IOCON1 = 0xCC00;    //set PWM Mode to True Independant  (PENH = PENL = 1 -> PWM module controles PWM pin & PMOD = 11 -> true independant mode)
    PWMCON1 = 0x0200;   //Set Independent Time Bases, Edge-Aligned Mode and Independent Duty Cycles
    FCLCON1 = 0x0003;   //Fault input is disabled
    
    //TRISBbits.TRISB14 = 0;
    //TRISBbits.TRISB15 = 0;
    
    //PMD6bits.PWM1MD = 0;
    
}
void initPWM2(){
    
}
void initPWM3(){
    
}
void initPWM4(){
    
}
void initPWM5(){       //20KHz
    PHASE5 = 7000;      //Period of PWM1H     Fosc / (F_PWM*PWM_Input_Clock_Prescaler) ex : 140Mhz / (20kHz * 1) = 7000 or wih Fosc = 138.24MHz -> 6912
    SPHASE5 = 7000;     //Period of PWM1L
    PDC5 = 0;           //Duty cycle of PWM1H
    SDC5 = 0;           //Duty cycle of PWM1L
    DTR5 = 0;           //Dead Time
    ALTDTR5 = 0;
    IOCON5 = 0xCC00;    //set PWM Mode to True Independant  (PENH = PENL = 1 -> PWM module controles PWM pin & PMOD = 11 -> true independant mode)
    PWMCON5 = 0x0200;   //Set Independent Time Bases, Edge-Aligned Mode and Independent Duty Cycles
    FCLCON5 = 0x0003;   //Fault input is disabled
    
}
void initPWM6(){
    
}

void sendToMotor(double valueR, double valueL)
{
    if(valueL > 0){
        valueL += DEAD_ZONE;
    }
    else if(valueL < 0){
        valueL -= DEAD_ZONE;
    }
    if(valueR > 0){
        valueR += DEAD_ZONE;
    }
    else if(valueR < 0){
        valueR -= DEAD_ZONE;
    }
    if(valueL <= -VSAT){
        SENS_L = BACKWARD;
        PWM_L = PWM_PR_L * VSAT / VBAT;
    }
    else if(valueL >= VSAT){
        SENS_L = FORWARD;
        PWM_L = PWM_PR_L * VSAT / VBAT;
    }
    else if(valueL < -DEAD_ZONE){
        SENS_L = BACKWARD;
        PWM_L = -(int)(valueL * PWM_PR_L / VBAT);
    }
    else if(valueL > DEAD_ZONE){
        SENS_L = FORWARD;
        PWM_L = (int)(valueL * PWM_PR_L / VBAT);
    }
    else{    //DEADZONE
        PWM_L = 0;
    }
    
    if(valueR <= -VSAT){
        SENS_R = FORWARD;
        PWM_R = PWM_PR_R * VSAT / VBAT;
    }
    else if(valueR >= VSAT){
        SENS_R = BACKWARD;
        PWM_R = PWM_PR_R * VSAT / VBAT;
    }
    else if(valueR < -DEAD_ZONE){
        SENS_R = FORWARD;
        PWM_R = -(int)(valueR * PWM_PR_R / VBAT);
    }
    else if(valueR > DEAD_ZONE){
        SENS_R = BACKWARD;
        PWM_R = (int)(valueR * PWM_PR_R / VBAT);
    }
    else{    //DEADZONE
        PWM_R = 0;
    }
}