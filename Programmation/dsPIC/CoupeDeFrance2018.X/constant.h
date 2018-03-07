#ifndef CONSTANT_H
#define	CONSTANT_H

// <editor-fold defaultstate="collapsed" desc="PID">
//PID speed left    units : rad/s -> V
#define KP_SPEED_LEFT       0.576
#define KI_SPEED_LEFT       0.05
#define KD_SPEED_LEFT       0.005
#define BIAS_SPEED_LEFT     0
#define T_SPEED_LEFT        0.01    //s

//PID speed right   units : rad/s -> V
#define KP_SPEED_RIGHT      0.576
#define KI_SPEED_RIGHT      0.05
#define KD_SPEED_RIGHT      0.005
#define BIAS_SPEED_RIGHT    0
#define T_SPEED_RIGHT       0.01

//PID distance      units : mm -> rad/s
#define KP_DISTANCE         0.07
#define KI_DISTANCE         0
#define KD_DISTANCE         0
#define BIAS_DISTANCE       0
#define T_DISTANCE          0.01

//PID angle         units : rad -> rad/s
#define KP_ANGLE        10
#define KI_ANGLE        0
#define KD_ANGLE        0
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
#define BRGVAL 455//((CLOCK_FREQ_HZ/2/BAUDRATE)/16) - 1

#define TX_SIZE 100     //size of Tx buffer
#define RX_SIZE 100     //size of Rx buffer
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Motor control">
#define FORWARD     1
#define BACKWARD    0

#define PWM_PR_L    PHASE5
#define PWM_PR_R    SPHASE5

#define VBAT        12  //à remplacer plus tard par lecture de la tension ?

#define VSAT        12  //saturation pour brider la vitesse

#define DEAD_ZONE   1.3   //tension min qui fait tourner le moteur A MESURER
//#define VMIN        1//0.3   //arreter les moteurs si la commande trop faible

#define ACC_MAX 0.24

#define MAX_ERROR_D     1//10      //mm
#define MAX_ERROR_A     0.01//0.01rad ~= 0.57°
#define MAX_SPEED_STOP  5 / ENCODER_WHEEL_RADIUS    //rad/s -> 5mm/s

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="I/O">
#define led LATGbits.LATG14     //Led-Rupt
#define PWM_R   PDC5            //PWM5L pin7  RD2 PWM_ASS_1
#define PWM_L   SDC5            //PWM5H pin6  RD1 PWM_ASS_0
#define SENS_L  LATEbits.LATE8  //      pin18 RE8 SENS_ASS_0
#define SENS_R  LATEbits.LATE9  //      pin19 RE9 SENS_ASS_1
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Odometry">
#define PI                                  3.14159265358979323846
#define ENCODER_WHEEL_RADIUS                24.6                    //mm         
#define DISTANCE_BETWEEN_ENCODER_WHEELS     295.8918449447346863    //mm
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ADC">
#define AUTO_SAMPLE 0   //0-Sampling begins when SAMP bit is set / 1-Sampling begins immediately after last conversion; SAMP bit is auto-set
// </editor-fold>

#endif	/* UART_H */
