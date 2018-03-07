/*
 * File:   main.c
 * Author: Quentin BOYER
 *
 * Created on September 18, 2017, 6:59 PM
 */


#include <xc.h>
#include <math.h>
#include <p33EP512GM310.h>
#include "clock.h"
#include "GPIO.h"
#include "timer.h"
#include "PWM.h"
#include "QEI.h"
#include "PID.h"
#include "SPI.h"
#include "ADC.h"
#include "UART.h"
#include "interrupt.h"

void testAccMax();
void reglageDiametre();
void printPos();
void straightPath(double cx, double cy, double ct);

void test();
void test2();
void testPWM();
void testSPI();
void testADC();
void testInterrupt();
void testDelay();

//Global variables
char TX[TX_SIZE];
char RX[RX_SIZE];
char unsigned TX_i;
char unsigned RX_i;
double x;
double y;
double theta;
double xc;
double yc;
double thetac;
PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;
int state = 0;
int R,L;

char arrived;

int main(){
    initClock(); //Clock 140 MHz
    initGPIO();
    initTimer();
    initPWM();
    initQEI();
    IOCON2 = 0;
    initUART();
    
    x = 0;
    y = 0;
    theta = 0;
    
    xc = 0;
    yc = 0;
    thetac = 0;
    
    TX_i = 0;
    RX_i = 0;
    
    R = 0;
    L = 0;
    
    POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;
    
    led = 1;
    
    initAllPID(&pidSpeedLeft, &pidSpeedRight, &pidDistance, &pidAngle);
    state = 0;
    //testAccMax();
    //reglageDiametre();
    //printPos();
    double i;
    while(1){
        switch(state){
            case 0:
                xc = 0;
                yc = 0;
                thetac = 0;
                led = 0;
                delay_ms(1000);
                state++;
                break;
            case 1 :
                //PDC5 = R;
                //SDC5 = L;
                //delay_ms(1000);
                led = 1;
                
                /*for(i = 0; i >= -20*PI; i-= 0.01){
                    thetac = i;
                    delay_ms(5);
                }*/
                /*for(i = 0; i <= 300; i++){
                    xc = i;
                    delay_ms(1);
                }*/
                //xc = 300;
                //yc = 0;
                //thetac = 0;
                straightPath(200,200,PI/2);
                delay_ms(10000);
                printPos();
                //state++;
                break;
            case 2 :
                xc = 100;
                yc = 0;
                thetac = PI/2;
                delay_ms(10000);
                state++;
                break;
            case 3 :
                xc = 100;
                yc = 100;
                thetac = PI/2;
                delay_ms(10000);
                state++;
                break;
            case 4 :
                xc = 100;
                yc = 100;
                thetac = PI;
                delay_ms(10000);
                state++;
                break;
            case 5 :
                xc = 0;
                yc = 100;
                thetac = PI;
                delay_ms(10000);
                state++;
                break;
            case 6 :
                xc = 0;
                yc = 100;
                thetac = 3*PI/2;
                delay_ms(10000);
                state++;
                break;
            case 7 :
                xc = 0;
                yc = 0;
                thetac = 3*PI/2;
                delay_ms(10000);
                state++;
                break;
            case 8 :
                xc = 0;
                yc = 0;
                thetac = 2*PI;
                break;
        }
        
        print("x : ");
        print(itoa((int)x));
        print("      y : ");
        print(itoa((int)y));
        print("      t : ");
        print(itoa((int)((theta*360)/(2*PI))));
        print("      t*100 : ");
        print(itoa((int)((theta*36000)/(2*PI))));
       
        print("\r\n");
        //delay_ms(5000);
    }
    return 0;
}

void testAccMax(){
    double commandeL = 12;
    double prevCommandeL = 0;
    double delta = 0.1;
    double newCL;
    while(1){
        if(commandeL - prevCommandeL > delta)
            newCL = prevCommandeL + delta;
        else if(commandeL - prevCommandeL < -delta)
            newCL = prevCommandeL - delta;
        else
            newCL = commandeL;
        prevCommandeL = newCL;
        sendToMotor(0,newCL);
        delay_ms(30);
    }
}
void reglageDiametre(){
    IEC0bits.T1IE = 0;
    POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;
    double l,r,l2,r2;
    double D1,D2;
    while(1){
        l = (double)(POS1CNTL - 0x8000) * PI * 2 * ENCODER_WHEEL_RADIUS / 4096;
        r = (POS2CNTL - 0x8000) * PI * 2 * ENCODER_WHEEL_RADIUS / 4096;
        l2 = (double)(POS1CNTL - 0x8000) * PI * 2 / 4096;
        r2 = (double)(POS2CNTL - 0x8000) * PI * 2 / 4096;
        D1 = 300/l2;
        D2 = 300/r2;
        
        print("l : ");
        print(itoa((int)l));
        print("      r : ");
        print(itoa((int)r));
        print("      D1 : ");
        print(itoa((int)(D1)));
        print("      D1*100 : ");
        print(itoa((int)(D1*100)));
        print("      D2 : ");
        print(itoa((int)(D2)));
        print("      D2*100 : ");
        print(itoa((int)(D2*100)));
       
        print("\r\n");
        delay_ms(200);
    }   
}
void printPos(){
    while(1){
        print("x : ");
        print(itoa((int)x));
        print("      y : ");
        print(itoa((int)y));
        print("      t : ");
        print(itoa((int)((theta*360)/(2*PI))));
        print("      t*100 : ");
        print(itoa((int)((theta*36000)/(2*PI))));
       
        print("\r\n");
        delay_ms(200);
    }
}
void straightPath(double cx, double cy, double ct){
    double i;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    
    /*Phase 1 : rotation */
    if(thetaRobotPoint > theta){
        for(i = theta; i <= thetaRobotPoint; i+= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    else{
        for(i = theta; i >= thetaRobotPoint; i-= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    thetac = thetaRobotPoint;
    arrived = 0;
    while(!arrived);
    
    /* Phase 2 : straight line */
    double y0 = y;
    if(cx != x){    //Warning : infinite slope
        double slope = (cy-y)/(cx-x);
        if(cx > x){
            for(i = x; i <= cx; i+= LINEAR_SPEED){
                xc = i;
                yc = y0 + i*slope;
                delay_ms(DELAY_SPEED);
            }
        }
        else{
            for(i = x; i >= cx; i-= LINEAR_SPEED){
                xc = i;
                yc = y0 + i*slope;
                delay_ms(DELAY_SPEED);
            }
        }
    }
    else{
        if(cy > y){
            for(i = y; i <= cy; i+= LINEAR_SPEED){
                yc = i;
                delay_ms(DELAY_SPEED);
            }
        }
        else{
            for(i = y; i >= cy; i= LINEAR_SPEED){
                yc = i;
                delay_ms(DELAY_SPEED);
            }
        }
    }
    xc = cx;
    yc = cy;
    arrived = 0;
    while(!arrived);
    
    /*Phase 3 : rotation */
    if(ct > theta){
        for(i = theta; i <= ct; i+= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    else{
        for(i = theta; i >= ct; i-= ROTATION_SPEED){
            thetac = i;
            delay_ms(DELAY_SPEED);
        }
    }
    thetac = ct;
    arrived = 0;
    while(!arrived);
    
}