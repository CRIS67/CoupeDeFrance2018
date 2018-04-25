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
#include "US.h"

void testAccMax();
void reglageDiametre();
void printPos();
void straightPath(double cx, double cy, double ct, double speedMax, double accMax);

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
/*Current position*/
volatile double x;
volatile double y;
volatile double theta;
/*Current setpoint position*/
double xc;
double yc;
double thetac;
/*Final setpoint position*/
double xf;
double yf;
double tf;
volatile PID pidSpeedLeft, pidSpeedRight, pidDistance, pidAngle;
int state = 0;
int R,L;

volatile char arrived;

extern volatile char   US_ON[NB_US];
extern volatile char   US_R[NB_US];
extern volatile double US[NB_US];

volatile char sendBT = 0;

int main(){
    initClock(); //Clock 140 MHz
    initGPIO();
    initPWM();
    initQEI();
    IOCON2 = 0;
    initUART();
    initInt();
    initUS();
    x = 0;
    y = 0;
    theta = 0;
    
    xc = 0;
    yc = 0;
    thetac = 0;
    
    xf = 0;
    yf = 0;
    tf = 0;
    
    TX_i = 0;
    RX_i = 0;
    
    R = 0;
    L = 0;
    
    double i;
    // <editor-fold defaultstate="collapsed" desc="Start">
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0;
    for (i = 0; i < 10000; i++);
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0;
    for (i = 0; i < 10000; i++);
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0;
    for (i = 0; i < 10000; i++);
    LED_0 = 1;
    for (i = 0; i < 10000; i++);
    LED_0 = 0; 
    // </editor-fold>

    
    POS1CNTL = 0x8000;
    POS2CNTL = 0x8000;
    
    initAllPID(&pidSpeedLeft, &pidSpeedRight, &pidDistance, &pidAngle);
    initTimer();
    state = 0;
    
    while(1){
        switch(state){
            case 0:
                xc = 0;
                yc = 0;
                thetac = 0;
                delay_ms(1000);
                //state++;
                break;
            default:
                print("ERROR UNDEFINED STATE");
                printRpi("ERROR UNDEFINED STATE");
                break;       
        }
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
        print("x : ");
        print(itoa((int)x));
        print("      y : ");
        print(itoa((int)y));
        print("      t : ");
        print(itoa((int)((theta*360)/(2*PI))));
        
        print("\r\n");
}
void straightPath(double cx, double cy, double ct, double speedMax, double accMax){
    double i;
    double thetaRobotPoint = atan2(cy-y,cx-x);
    double phi = thetaRobotPoint - theta;
    while(phi < -PI)
        phi += 2*PI;
    while(phi > PI){
        phi -= 2*PI;
    }
    /*Phase 1 : rotation */
    xf = x;
    yf = y;
    tf = theta;
    double theta0 = theta;
    double AngularAcceleration = 10;
    double maxAngularVelocity = 10;
    double angularVelocity = 0;
    double prevAngularVelocity = 0;
    double angle = 0;
    
    double sign;
    if(phi > 0)
        sign = 1;
    else
        sign = -1;
    while(angularVelocity < maxAngularVelocity && angle < phi/2){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    double angle1 = angle;
    while(angle < phi - angle1){
		angle += TE * angularVelocity;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    AngularAcceleration = -AngularAcceleration;
    while(angularVelocity > 0 && angle < phi){
		angularVelocity += AngularAcceleration * TE;
		angle += TE * (prevAngularVelocity + angularVelocity) / 2;
        thetac = theta0 + angle * sign;
        prevAngularVelocity = angularVelocity;
		delay_ms(TE * 1000);
	}
    thetac = theta0 + phi*sign;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
 
    delay_ms(500);
    /* Phase 2 : straight line */
    xf = cx;
    yf = cy;
    tf = ct;
    if(speedMax < 0)
        speedMax = 0;
    else if(speedMax > SPEED_MAX)
        speedMax = SPEED_MAX;
    if(accMax < 0)
        accMax = 0;
    else if(accMax > ACCELERATION_MAX)
        accMax = ACCELERATION_MAX;
    
    double y0 = y;
	double x0 = x;
	double dx = cx - x0;
	double dy = cy - y0;
	double alpha = atan2(dy,dx);
	double totalDistance = sqrt(dx*dx+dy*dy);
    
    double acc = accMax;
	double speed = 0;
	double precSpeed = 0;
	double dist = 0;
	while(speed < speedMax && dist < totalDistance/2){
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    double dist1 = dist;
	//2
	if(speed > speedMax)
        speed = speedMax;
	while(dist < totalDistance - dist1){               //Condition
		dist += TE * speed;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
	//3
	acc = -acc;
	while(speed > 0 && dist < totalDistance){				//Condition		//v > 0			/		d < totalDistance
		speed += acc * TE;
		dist += TE * (precSpeed + speed) / 2;
        xc = x0 + dist * cos(alpha);
        yc = y0 + dist * sin(alpha);
        precSpeed = speed;
		delay_ms(TE * 1000);
	}
    xc = cx;
    yc = cy;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    delay_ms(500);
    //while(!arrived);
    
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
    //print("Phase 3 OK\n");
    thetac = ct;
    /*arrived = 0;
    while(!arrived){
        printPos();
    }*/
    
    delay_ms(500);
    //while(!arrived);
}