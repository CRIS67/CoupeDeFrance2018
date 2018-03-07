/*
 * File:   timer.c
 * Author: Quentin BOYER
 *
 * Created on 19 septembre 2017, 17:17
 */

#include "timer.h"

extern double x;
extern double y;
extern double theta;
extern double xc;
extern double yc;
extern double thetac;
unsigned int n,n2;

extern PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;

double prevCommandeL;
double prevCommandeR;

int tSpeedL[10];    //for sliding average
int tSpeedR[10];

extern char arrived;

void initTimer() {
    initTimer1();
    initTimer2();

    prevCommandeL = 0;
    prevCommandeR = 0;
    
    int i;
    for(i = 0; i < 10; i++){
        tSpeedL[i] = 0;
        tSpeedR[i] = 0;
    }
}

void initTimer1(){          //Timer 1
    T1CONbits.TON = 0;      //disable timer
    TMR1 = 0;               // Clear timer register
    PR1 = 1094;//10937;     //period 10 937 -> ~10ms
    T1CONbits.TSIDL = 0;    //Continues module operation in Idle mode
    T1CONbits.TCS = 0;      //internal clock
    T1CONbits.TCKPS = 0b10; //prescaler : 1:64
    T1CONbits.TGATE = 0;    //Gated time accumulation is disabled
    IFS0bits.T1IF = 0;      //Clear flag
    IEC0bits.T1IE = 1;      //Enable interrupt
    T1CONbits.TON = 1;      //enable Timer1
}

void initTimer2(){          //Timer 2
    T2CONbits.TON = 0;      //disable timer
    TMR2 = 0;               // Clear timer register
    PR2 = 0xFFFF;           //Period value
    T2CONbits.TSIDL = 0;    //Continues module operation in Idle mode
    T2CONbits.TCS = 0;      //internal clock
    T2CONbits.TCKPS = 0b10; //prescaler : 1:64
    T2CONbits.TGATE = 0;    //Gated time accumulation is disabled
    IFS0bits.T2IF = 0;      //Clear flag
    IEC0bits.T2IE = 0;      //Enable interrupt
    T2CONbits.TON = 1;      //enable Timer1
}

void delay_us(unsigned int delay){
    TMR2 = 0;
    int begin = TMR2;
    while(TMR2 - begin < delay);
}

void delay_ms(unsigned int delay){
    unsigned int usDelay = (unsigned int)(1.09 * (double)delay);    //timer2 count every 0.914 us -> *1.09 -> ~1us
    int i;
    for( i = 0; i < 1000; i++){
        delay_us(usDelay);
    }
}

void __attribute__((interrupt,no_auto_psv)) _T1Interrupt(void)
{
    IFS0bits.T1IF = 0; //Clear Timer1 interrupt flag
    
    // <editor-fold defaultstate="collapsed" desc="Calcul position">
    //save value
    unsigned int p1 = POS1CNTL; 
    unsigned int p2 = POS2CNTL;
    //reset value
    POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;
    
    double tL = (double) p1 - 0x8000;
    double tR = (double) p2 - 0x8000;

    //double speedL = tL * 100 * 2 * PI / 4096; //  /0.01 -> 10ms
    //double speedR = tR * 100 * 2 * PI / 4096;
    tSpeedL[n] = tL;
    tSpeedR[n] = tR;
    
    
    tL = tL * 2 * PI * ENCODER_WHEEL_RADIUS;
    tL = tL / 4096;

    tR = tR * 2 * PI * ENCODER_WHEEL_RADIUS;
    tR = tR / 4096;
    theta += (tR - tL) / DISTANCE_BETWEEN_ENCODER_WHEELS;

    /*if(theta >= 2*PI)
        theta -= 2*PI;
    else if(theta < 0)
        theta += 2*PI;*/
    
    double deltaD = (tL + tR) / 2;
    x += deltaD * cos(theta);
    y += deltaD * sin(theta);
    
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Asservissement">
    n++;
    if (n >= 10) {//1000
        n = 0;
        //LATGbits.LATG14 = !LATGbits.LATG14;
        
        double speedL = 0; //  /0.01 -> 10ms
        double speedR = 0;
        
        int i;
        for(i = 0; i < 10; i++){
            speedL += tSpeedL[i];
            speedR += tSpeedR[i];
        }
        
        speedL = speedL * 100 * 2 * PI / 4096; //  /0.01 -> 10ms
        speedR = speedR * 100 * 2 * PI / 4096;
        
        double thetaRobotPoint = atan2(yc-y,xc-x);
    
        double errorD = sqrt((x-xc)*(x-xc) + (y-yc)*(y-yc));

        double alpha = theta - thetaRobotPoint;
        while(alpha >= (2*PI)){
            alpha -= 2*PI;
        }
        while(alpha <= -(2*PI)){
            alpha += 2*PI;
        }
        if(alpha > PI)
            alpha -= 2*PI;
        else if(alpha < -PI)
            alpha += 2*PI;

        if((alpha) > -PI/2 && (alpha) < PI/2)
        {
            errorD = -errorD; // gestion marche arrière
        }

        if(errorD > 50 || errorD < -50)
            setSetPoint(&pidAngle,thetaRobotPoint);
        else
            setSetPoint(&pidAngle,thetac);
        //setSetPoint(&pidAngle,thetac);

        double rD = compute(&pidDistance,errorD);
        double rA = compute(&pidAngle,theta);

        setSetPoint(&pidSpeedLeft,rD - rA);
        setSetPoint(&pidSpeedRight,rD + rA);

        /*if(pidDistance.prevError < MAX_ERROR_D && pidDistance.prevError > -MAX_ERROR_D && pidAngle.prevError < MAX_ERROR_A && pidAngle.prevError > -MAX_ERROR_A && speedL < MAX_SPEED_STOP && speedL > -MAX_SPEED_STOP && speedR < MAX_SPEED_STOP && speedR > -MAX_SPEED_STOP){
            pidSpeedLeft.sumI = 0;
            pidSpeedLeft.sumI = 0;
        }*/

        double commandeL = compute(&pidSpeedLeft,speedL);
        double commandeR = compute(&pidSpeedRight,speedR);
        
        if(pidDistance.prevError < MAX_ERROR_D && pidDistance.prevError > -MAX_ERROR_D && pidAngle.prevError < MAX_ERROR_A && pidAngle.prevError > -MAX_ERROR_A && speedL < MAX_SPEED_STOP && speedL > -MAX_SPEED_STOP && speedR < MAX_SPEED_STOP && speedR > -MAX_SPEED_STOP){
            commandeL = 0;
            commandeR = 0;
            pidSpeedLeft.sumI = 0;
            pidSpeedLeft.sumI = 0;
            arrived = 1;
        }
        else
            arrived = 0;

        //print(itoa((int)(errorD)));


        if(commandeR - prevCommandeR > ACC_MAX){
            commandeR = prevCommandeR + ACC_MAX;
        }
        else if(commandeR - prevCommandeR < -ACC_MAX){
            commandeR = prevCommandeR - ACC_MAX;
        }
        if(commandeL - prevCommandeL > ACC_MAX){
            commandeL = prevCommandeL + ACC_MAX;
        }
        else if(commandeL - prevCommandeL < -ACC_MAX){
            commandeL = prevCommandeL - ACC_MAX;
        }

        prevCommandeL = commandeL;
        prevCommandeR = commandeR;

        sendToMotor(commandeR, commandeL);
        
    }// </editor-fold>
}