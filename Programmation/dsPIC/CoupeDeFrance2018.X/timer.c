/*
 * File:   timer.c
 * Author: Quentin BOYER
 *
 * Created on 19 septembre 2017, 17:17
 */

#include "timer.h"

extern volatile double x;
extern volatile double y;
extern volatile double theta;

extern double xc;
extern double yc;
extern double thetac;

extern double xf;
extern double yf;
extern double tf;

extern volatile char   US_ON[NB_US];
extern volatile char   US_R[NB_US];
extern volatile double US[NB_US];

unsigned int n,n2;

extern volatile PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;

double volatile prevCommandeL;
double volatile prevCommandeR;

double volatile speedLSum;
double volatile speedRSum;

double volatile smoothSpeedL;
double volatile smoothSpeedR;

extern volatile char arrived;

volatile double d;

unsigned char i_us,n_us;

extern volatile char sendBT;
extern volatile unsigned char debugPosRpiAsserv;

extern volatile unsigned char stop;

void initTimer() {
    initTimer1();
    initTimer2();
    initTimer3();
    initTimer4();

    prevCommandeL = 0;
    prevCommandeR = 0;
    
    speedLSum = 0;
    speedRSum = 0;
    smoothSpeedL = 0;
    smoothSpeedR = 0;
    
    d = 0;
    i_us = 0;
    n_us = 0;
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

void initTimer3(){          //Timer 3   -> 20µs delay
    T3CONbits.TON = 0;      //disable timer
    TMR3 = 0;               // Clear timer register
    PR3 = 1400;           //Period value
    T3CONbits.TSIDL = 0;    //Continues module operation in Idle mode
    T3CONbits.TCS = 0;      //internal clock
    T3CONbits.TCKPS = 0b00; //prescaler : 1:1
    T3CONbits.TGATE = 0;    //Gated time accumulation is disabled
    IFS0bits.T3IF = 0;      //Clear flag
    IEC0bits.T3IE = 1;      //Enable interrupt
    T3CONbits.TON = 0;      //disable Timer3
}

void initTimer4(){          //Timer 4   -> count US time
    T4CONbits.TON = 0;      //disable timer
    TMR4 = 0;               // Clear timer register
    PR4 = 0xFFFF;           //Period value
    T4CONbits.TSIDL = 0;    //Continues module operation in Idle mode
    T4CONbits.TCS = 0;      //internal clock
    T4CONbits.TCKPS = 0b10; //prescaler : 1:64
    T4CONbits.TGATE = 0;    //Gated time accumulation is disabled
    IFS1bits.T4IF = 0;      //Clear flag
    IEC1bits.T4IE = 0;      //Enable interrupt
    T4CONbits.TON = 1;      //enable Timer4
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
    
    speedLSum += tL;
    speedRSum += tR;
    
    tL = tL * 2 * PI * ENCODER_WHEEL_RADIUS;
    tL = tL / 4096;

    tR = tR * 2 * PI * ENCODER_WHEEL_RADIUS;
    tR = tR / 4096;
    theta += (tR - tL) / DISTANCE_BETWEEN_ENCODER_WHEELS;
    

    /*if(theta > PI)
        theta -= 2*PI;
    else if(theta < -PI)
        theta += 2*PI;*/
    
    double deltaD = (tL + tR) / 2;
    x += deltaD * cos(theta);
    y += deltaD * sin(theta);
    d += deltaD;
    
    // </editor-fold>

    // <editor-fold defaultstate="collapsed" desc="Asservissement">
    n++;
    if (n >= N_ASSERV) {//1000
        n = 0;
        //LATGbits.LATG14 = !LATGbits.LATG14;
        
        double speedL = 0; //  /0.01 -> 10ms
        double speedR = 0;
        
        /*int i;
        for(i = 0; i < 10; i++){
            speedL += tSpeedL[i];
            speedR += tSpeedR[i];
        }*/
        
        speedL = speedLSum * (1000 / N_ASSERV) * 2 * PI / 4096;
        speedR = speedRSum * (1000 / N_ASSERV) * 2 * PI / 4096;
        
        speedLSum = 0;
        speedRSum = 0;
        /*int i;
        for(i = 1; i < N_SMOOTHING; i++){
            smoothSpeedL = smoothSpeedL + SMOOTHING_FACTOR * (speedL - smoothSpeedL);
            smoothSpeedR = smoothSpeedR + SMOOTHING_FACTOR * (speedR - smoothSpeedR);
        }*/
        
        double thetaRobotPoint = atan2(yc-y,xc-x);
        double thetaRobotPointFinal = atan2(yf-y,xf-x);
        while(thetaRobotPointFinal - theta < PI)
            thetaRobotPointFinal += 2*PI;
        while(thetaRobotPointFinal - theta > PI)
            thetaRobotPointFinal -= 2*PI;
        double distFinal = sqrt((x-xf)*(x-xf) + (y-yf)*(y-yf));
        
        double errorD = sqrt((x-xc)*(x-xc) + (y-yc)*(y-yc));

        // <editor-fold defaultstate="collapsed" desc="gestion marche arrière">
        double alpha = theta - thetaRobotPoint;
        while (alpha > PI) {
            alpha -= 2 * PI;
        }
        while (alpha < -PI) {
            alpha += 2 * PI;
        }
        if (alpha > -PI / 2 && alpha < PI / 2) {
            errorD = -errorD;
        }// </editor-fold>


        if(distFinal > DIST_AIM_POINT || distFinal < -DIST_AIM_POINT)
            setSetPoint(&pidAngle,thetaRobotPointFinal);
        else
            setSetPoint(&pidAngle,thetac);
        //setSetPoint(&pidAngle,thetac);

        double rD = compute(&pidDistance,errorD);
        double rA = compute(&pidAngle,theta);
        
        if(pidDistance.prevError < MAX_ERROR_D && pidDistance.prevError > -MAX_ERROR_D){
            rD = 0;
        }
        if(pidAngle.prevError < MAX_ERROR_A && pidAngle.prevError > -MAX_ERROR_A){
            rA = 0;
        }

        setSetPoint(&pidSpeedLeft,rD - rA);
        setSetPoint(&pidSpeedRight,rD + rA);
        
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
        
        //double commandeL = rD - rA;
        //double commandeR = rD + rA;
        
        
        
        /*if(pidDistance.prevError < MAX_ERROR_D && pidDistance.prevError > -MAX_ERROR_D && pidAngle.prevError < MAX_ERROR_A && pidAngle.prevError > -MAX_ERROR_A && speedL < MAX_SPEED_STOP && speedL > -MAX_SPEED_STOP && speedR < MAX_SPEED_STOP && speedR > -MAX_SPEED_STOP){
            commandeL = 0;
            commandeR = 0;
            pidSpeedLeft.sumI = 0;
            pidSpeedLeft.sumI = 0;
            arrived = 1;
        }
        else
            arrived = 0;*/

        /*print(itoa((int)(speedL*100)));
        print(";");*/
        /*print(itoa((int)(commandeL*100)));
        print(" / ");
        print(itoa((int)(commandeR*100)));
        print("\n");*/
        
        /*print(itoa((int)((theta*360)/(2*PI))));
        print("   /   ");
        print(itoa((int)((thetac*360)/(2*PI))));
        */
        if(sendBT){
            print(itoa((int)(x*10)));
            print(",");
            print(itoa((int)(xc*10)));
            print("\n");
        }
        if(debugPosRpiAsserv){
            printRpi(itoa((int)x));
            printRpi(",");
            printRpi(itoa((int)y));
            printRpi(",");
            printRpi(itoa((int)((theta*360)/(2*PI))));
            printRpi(",");
            printRpi(itoa((int)xc));
            printRpi(",");
            printRpi(itoa((int)yc));
            printRpi(",");
            printRpi(itoa((int)((thetac*360)/(2*PI))));
            printRpi("\n");
        }
        //print("\r\n");


        // <editor-fold defaultstate="collapsed" desc="Limit acceleration">
        if (commandeR - prevCommandeR > ACC_MAX) {
            commandeR = prevCommandeR + ACC_MAX;
        } else if (commandeR - prevCommandeR < -ACC_MAX) {
            commandeR = prevCommandeR - ACC_MAX;
        }
        if (commandeL - prevCommandeL > ACC_MAX) {
            commandeL = prevCommandeL + ACC_MAX;
        } else if (commandeL - prevCommandeL < -ACC_MAX) {
            commandeL = prevCommandeL - ACC_MAX;
        }// </editor-fold>


        prevCommandeL = commandeL;
        prevCommandeR = commandeR;

        if(!stop)
            sendToMotor(commandeR, commandeL);
        
        //print("=)\n");
        
    }// </editor-fold>
    
    // <editor-fold defaultstate="collapsed" desc="US">
    n_us++;
    if (n_us >= N_US) {
        n_us = 0;
        i_us++;
        if(i_us >= NB_US)
            i_us = 0;
        switch(i_us){
            case 0:
                TRIG_US_0 = 1;
                break;
            case 1:
                TRIG_US_1 = 1;
                break;
            case 2:
                TRIG_US_2 = 1;
                break;
            case 3:
                TRIG_US_3 = 1;
                break;
            case 4:
                TRIG_US_4 = 1;
                break;
            case 5:
                TRIG_US_5 = 1;
                break;
        }
        unsigned char i = 0;
        for(i = 0; i < NB_US; i++){
            US_ON[i] = 0;
            US_R[i] = 0;
        }
        US_ON[i_us] = 1;
        T3CONbits.TON = 1;      //enable Timer3
        /*print("toggle ");
        print(itoa((int)i_us));
        print("\n");*/
    }
    // </editor-fold>

    
    
    /*print(itoa((int)TMR1));
    print("\n");*/
}

void __attribute__((interrupt,no_auto_psv)) _T3Interrupt(void){
        IFS0bits.T3IF = 0;
        T3CONbits.TON = 0;      //disable Timer3
        TMR3 = 0;               //reset Timer3
        TRIG_US_0 = 0;
        TRIG_US_1 = 0;
        TRIG_US_2 = 0;
        TRIG_US_3 = 0;
        TRIG_US_4 = 0;
        TRIG_US_5 = 0;
}