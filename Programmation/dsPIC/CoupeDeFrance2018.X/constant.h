#ifndef CONSTANT_H
#define	CONSTANT_H

// <editor-fold defaultstate="collapsed" desc="PID">
//PID speed left    units : rad/s -> V
#define KP_SPEED_LEFT       0.38//0.57//0.576
#define KI_SPEED_LEFT       0//0.00535//0.05
#define KD_SPEED_LEFT       0.0013375//0.005
#define BIAS_SPEED_LEFT     0
#define T_SPEED_LEFT        0.01    //s

//PID speed right   units : rad/s -> V
#define KP_SPEED_RIGHT      0.38//0.57//0.576
#define KI_SPEED_RIGHT      0//0.00535//0.05
#define KD_SPEED_RIGHT      0.0013375//0.005
#define BIAS_SPEED_RIGHT    0
#define T_SPEED_RIGHT       0.01

//PID distance      units : mm -> rad/s
#define KP_DISTANCE         0.4//0.15//0.04
#define KI_DISTANCE         0
#define KD_DISTANCE         0.001//0//0.0065
#define BIAS_DISTANCE       0
#define T_DISTANCE          0.01

//PID angle         units : rad -> rad/s
#define KP_ANGLE        60//30//60
#define KI_ANGLE        0
#define KD_ANGLE        1//0//6.5
#define BIAS_ANGLE      0
#define T_ANGLE         0.01

#define MAX_I    100

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Clock">
#define CLOCK_M     35
#define CLOCK_N1    2
#define CLOCK_N2    2
#define CLOCK_SOURCE_FREQ_HZ   16000000

//#define CLOCK_FREQ  CLOCK_SOURCE_FREQ * CLOCK_M / (CLOCK_N1 * CLOCK_N2)
#define CLOCK_FREQ_HZ   140000000
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="UART">
#define BAUDRATE 9600
#define BRGVAL 455  //((CLOCK_FREQ_HZ/2/BAUDRATE)/16) - 1   9600
#define BRGVAL2 34//6//34//37   //  115000
#define TX_SIZE 100     //size of Tx buffer
#define RX_SIZE 100     //size of Rx buffer
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Motor control">
#define FORWARD     1
#define BACKWARD    0

#define PWM_PR_L    PHASE3
#define PWM_PR_R    SPHASE3

#define VBAT        (int)(((double)ADC1BUF0*5.7*3.3)/1024)//12  //à remplacer plus tard par lecture de la tension ?

#define VSAT        12  //saturation pour brider la vitesse

#define DEAD_ZONE   1.3   //tension min qui fait tourner le moteur A MESURER
#define COEF_MOT_BO 0.65//0.428571    //  250*12/7000
//#define VMIN        1//0.3   //arreter les moteurs si la commande trop faible

#define ACC_MAX 10000

#define MAX_ERROR_D     1//1//10      //mm
#define MAX_ERROR_A     0.01//0.01rad ~= 0.57°
#define MAX_SPEED_STOP  5 / ENCODER_WHEEL_RADIUS    //rad/s -> 5mm/s

#define N_ASSERV        10  //nombre d'itérations entre 2 boucles d'asserv'
#define DIST_AIM_POINT  50  //distance au dessus de laquelle le robot vise le point final
#define SMOOTHING_FACTOR    0.4
#define N_SMOOTHING     5

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I/O">

#define NB_US       6               //number of sensors
#define N_US        20              //loop iterations

#define PWM_R       PDC3            //PWM5L pin7  RD2 PWM_ASS_1
#define PWM_L       SDC3            //PWM5H pin6  RD1 PWM_ASS_0
#define SENS_L      LATGbits.LATG1  //      pin18 RE8 SENS_ASS_0
#define SENS_R      LATGbits.LATG0  //      pin19 RE9 SENS_ASS_1

#define RUPT_ASS_0  PORTEbits.RE8
#define RUPT_ASS_1  PORTGbits.RG10
#define RUPT_ASS_2  PORTAbits.RA12
#define RUPT_ASS_3  PORTEbits.RE9

#define RUPT_ACT_0  PORTFbits.RF9
#define RUPT_ACT_1  PORTBbits.RB1
#define RUPT_ACT_2  PORTBbits.RB0
#define RUPT_ACT_3  PORTAbits.RA1
#define RUPT_ACT_4  PORTAbits.RA0
#define RUPT_ACT_5  PORTAbits.RA11

#define SIGNAL_JUMPER   PORTFbits.RF10

#define ECHO_US_0   PORTAbits.RA8
#define ECHO_US_1   PORTDbits.RD14
#define ECHO_US_2   PORTEbits.RE14
#define ECHO_US_3   PORTEbits.RE12
#define ECHO_US_4   PORTFbits.RF13
#define ECHO_US_5   PORTCbits.RC11

#define TRIG_US_0   LATBbits.LATB4
#define TRIG_US_1   LATDbits.LATD15
#define TRIG_US_2   LATEbits.LATE15
#define TRIG_US_3   LATEbits.LATE13
#define TRIG_US_4   LATFbits.LATF12
#define TRIG_US_5   LATGbits.LATG11

#define US_NUM_0    LATCbits.LATC1
#define US_NUM_1    LATCbits.LATC0


#define LED_0       LATGbits.LATG3
#define LED_1       LATFbits.LATF4
#define LED_2       LATFbits.LATF5

#define SENS_ASS_0  LATGbits.LATG0
#define SENS_ASS_1  LATGbits.LATG1
#define SENS_ACT_0  LATGbits.LATG13
#define SENS_ACT_1  LATGbits.LATG12

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Odometry">
#define PI                                  3.14159265358979323846
#define ENCODER_WHEEL_RADIUS                24.6                    //mm         
#define DISTANCE_BETWEEN_ENCODER_WHEELS     300//295.8918449447346863    //mm
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ADC">
#define AUTO_SAMPLE 1   //0-Sampling begins when SAMP bit is set / 1-Sampling begins immediately after last conversion; SAMP bit is auto-set
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Path generation">
#define ROTATION_SPEED      0.002   //rad
#define LINEAR_SPEED        0.1     //mm
#define DELAY_SPEED         1       //ms

#define ACCELERATION_MAX    1000000     //mm.s^-2
#define SPEED_MAX           1000000    //mm.s^-1
#define TE                  0.01    //s
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Servomotors">
#define SERVO0  0
#define SERVO1  1
#define SERVO2  2
#define SERVO3  3
#define SERVO4  4
#define SERVO5  5
#define SERVO6  6

#define SERVO0_UP       2600
#define SERVO0_DOWN     1800
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="AX12">
#define AX12_BAUDRATE 500000
#define AX12_BRGVAL 34  //((CLOCK_FREQ_HZ/2/BAUDRATE)/4) - 1 

#define AX12_TX_SIZE 100
#define AX12_RX_SIZE 100

#define AX12_ID_1 0x01
#define AX12_ID_3 0x03
#define AX12_ID_ALL 0xFE

#define AX12_COMMAND_CW_ANGLE_LIMIT_LOW 0x06
#define AX12_COMMAND_CW_ANGLE_LIMIT_HIGH 0x07
#define AX12_COMMAND_CCW_ANGLE_LIMIT_LOW 0x08
#define AX12_COMMAND_CCW_ANGLE_LIMIT_HIGH 0x09
#define AX12_COMMAND_STATUS_RETURN_LEVEL 0x10
#define AX12_COMMAND_LED 0x19
#define AX12_COMMAND_GOAL_POSITION_LOW 0x1E
#define AX12_COMMAND_GOAL_POSITION_HIGH 0x1F
#define AX12_COMMAND_MOVING_SPEED_LOW 0x20
#define AX12_COMMAND_MOVING_SPEED_HIGH 0x21

#define AX12_INSTRUCTION_WRITE_DATA 0x03
// </editor-fold>


#endif	/* UART_H */
