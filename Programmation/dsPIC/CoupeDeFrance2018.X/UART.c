/*
 * File:   UART.c
 * Author: Quentin BOYER
 *
 * Created on 31 octobre 2017, 21:10
 * 
 * Notes : Parfois des erreurs lors de l'envoi de plusieurs octets en meme temps depui la RPi, à vérifier en augmentant la clock
 */

#include "UART.h"
#include "PWM.h"

extern double xc;
extern int state;
extern PID pidAngle, pidDistance, pidSpeedLeft, pidSpeedRight;
extern int R,L;
extern volatile double US[NB_US];

extern volatile unsigned char stop;

char TxLoopBuffer[TX_SIZE];
char TxLoopBuffer2[TX_SIZE];
char RxBuffer[RX_SIZE];
char RxBuffer2[RX_SIZE];
unsigned char iD,iF,iD2,iF2;                //index of first data and of first free element
unsigned char TxOn,TxOn2;
unsigned char iRx,iRx2;

volatile double receivedX,receivedY,receivedTheta;
volatile unsigned char newPosReceived = 0;
volatile unsigned char newAngleReceived = 0;

volatile unsigned char debugPosRpi = 0;
volatile unsigned char debugPosRpiAsserv = 0;

void initUART(){
    initUART1();
    initUART2();
}
void initUART1() {
    IEC0bits.U1TXIE = 0;            //Disable UART1 Tx interrupt
    IEC0bits.U1RXIE = 0;            //Disable UART1 Rx interrupt
    
    TRISBbits.TRISB6 = 0;           //TX
    TRISBbits.TRISB5 = 1;           //RX
    
    RPINR18 = 0b0100101;
    RPOR2bits.RP38R = 0b000001; 
    
    U1MODEbits.UARTEN = 0;
    U1MODEbits.USIDL = 0;           // Bit13 Continue in Idle
    U1MODEbits.IREN = 0;            // Bit12 No IR translation
    U1MODEbits.RTSMD = 0;           // Bit11 Simplex Mode
    U1MODEbits.UEN = 0b00;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 0;
    U1MODEbits.PDSEL = 0b00;
    U1MODEbits.STSEL = 0;

    U1STAbits.UTXBRK = 0;           //Bit11 Disabled
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;         //Interrupt is generated when any character is transferred to the Transmit Shift Register and the transmit buffer is empty (which implies at least one location is empty in the transmit buffer)
    U1STAbits.UTXINV = 0;
    U1STAbits.ADDEN = 0;
    U1STAbits.URXISEL = 0;
    
    U1BRG = BRGVAL ;
    
    IFS0bits.U1TXIF = 0;            // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 1;            // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0;            // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1;            // Enable Recieve Interrupts
    
    U1MODEbits.UARTEN = 1;          //Enable the module
    U1STAbits.UTXEN = 1;
    
    iD = 0;
    iF = 0;
    TxOn = 0;
    iRx = 0;
}
void initUART2() {
    IEC1bits.U2TXIE = 0;            //Disable UART2 Tx interrupt
    IEC1bits.U2RXIE = 0;            //Disable UART2 Rx interrupt
    
    RPINR19 = 0b01010000;          //RPI80(pin82-RE0) tied to UART2 RX
    RPOR9bits.RP81R = 0b000011;     //RP81 (pin53-RE1) tied to UART2 TX
    
    U2MODEbits.UARTEN = 0;
    U2MODEbits.USIDL = 0;           // Bit13 Continue in Idle
    U2MODEbits.IREN = 0;            // Bit12 No IR translation
    U2MODEbits.RTSMD = 0;           // Bit11 Simplex Mode
    U2MODEbits.UEN = 0b00;
    U2MODEbits.LPBACK = 0;
    U2MODEbits.ABAUD = 0;
    U2MODEbits.URXINV = 0;
    U2MODEbits.BRGH = 1;
    U2MODEbits.PDSEL = 0b00;
    U2MODEbits.STSEL = 0;

    U2STAbits.UTXBRK = 0;           //Bit11 Disabled
    U2STAbits.UTXISEL0 = 0;
    U2STAbits.UTXISEL1 = 0;         //Interrupt is generated when any character is transferred to the Transmit Shift Register and the transmit buffer is empty (which implies at least one location is empty in the transmit buffer)
    U2STAbits.UTXINV = 0;
    U2STAbits.ADDEN = 0;
    U2STAbits.URXISEL = 0;
    
    U2BRG = BRGVAL2 ;
    
    IFS1bits.U2RXIF = 0;
    IFS1bits.U2TXIF = 0;
    IEC1bits.U2TXIE = 1;
    IEC1bits.U2RXIE = 1;
    
    U2MODEbits.UARTEN = 1;          //Enable the module
    U2STAbits.UTXEN = 1;
    
    iD2 = 0;
    iF2 = 0;
    TxOn2 = 0;
    iRx2 = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U1RXInterrupt(void)
{
    if(U1STAbits.OERR == 1)
        U1STAbits.OERR = 0;
    
    while(U1STAbits.URXDA != 0)
    {
        //U1TXREG = U1RXREG; // Loopback
        if(iRx >= RX_SIZE)
            iRx = 0;
        RxBuffer[iRx] = U1RXREG;
        iRx++;
        if(RxBuffer[iRx-1] == '\n'){
            if(RxBuffer[0] == 's' && RxBuffer[1] == 'e' && RxBuffer[2] == 't')
                LED_0 = !LED_0;
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == 'a' && RxBuffer[2] == '+'){
                pidAngle.Kp += 1;
                print("kpA : ");
                print(itoa((int)(pidAngle.Kp*1)));
                print("\n");
            }
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == 'd' && RxBuffer[2] == '+'){
                pidDistance.Kp += 0.001;
                print("kpD : ");
                print(itoa((int)(pidDistance.Kp*1000)));
                print("\n");
            }
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == 'a'  && RxBuffer[2] == '-'){
                pidAngle.Kp -= 1;
                print("kpA : ");
                print(itoa((int)(pidAngle.Kp*1)));
                print("\n");
            }
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == 'd'  && RxBuffer[2] == '-'){
                pidDistance.Kp -= 0.001;
                print("kpD : ");
                print(itoa((int)(pidDistance.Kp*1000)));
                print("\n");
            }
            else if(RxBuffer[0] == 'd' && RxBuffer[1] == 'a' && RxBuffer[2] == '+'){
                pidAngle.Kd += 0.1;
                print("kdA : ");
                print(itoa((int)(pidAngle.Kd*10)));
                print("\n");
            }
            else if(RxBuffer[0] == 'd' && RxBuffer[1] == 'd' && RxBuffer[2] == '+'){
                pidDistance.Kd += 0.0001;
                print("kdD : ");
                print(itoa((int)(pidDistance.Kd*10000)));
                print("\n");
            }
            else if(RxBuffer[0] == 'd' && RxBuffer[1] == 'a' && RxBuffer[2] == '-'){
                pidAngle.Kd -= 0.1;
                print("kdA : ");
                print(itoa((int)(pidAngle.Kd*10)));
                print("\n");
            }
            else if(RxBuffer[0] == 'd' && RxBuffer[1] == 'd' && RxBuffer[2] == '-'){
                pidDistance.Kd -= 0.0001;
                print("kdD : ");
                print(itoa((int)(pidDistance.Kd*10000)));
                print("\n");
            }
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == '+'){
                pidSpeedLeft.Kp += 0.01;
                print("P : ");
                print(itoa(pidSpeedLeft.Kp*100));
                print("\n");
            }
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == '-'){
                pidSpeedLeft.Kp -= 0.01;
                print("P : ");
                print(itoa(pidSpeedLeft.Kp*100));
                print("\n");
            }
            else if(RxBuffer[0] == 'r' && RxBuffer[1] == '+'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                PWM_R+=50;
                print("R : ");
                print(itoa(PWM_R));
                print("\n");
            }
            else if(RxBuffer[0] == 'r' && RxBuffer[1] == '-'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                PWM_R-=50;
                print("R : ");
                print(itoa(PWM_R));
                print("\n");
            }
            else if(RxBuffer[0] == 'l' && RxBuffer[1] == '+'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                PWM_L+=50;
                print("L : ");
                print(itoa(PWM_L));
                print("\n");
            }
            else if(RxBuffer[0] == 'l' && RxBuffer[1] == '-'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                PWM_L-=50;
                print("L : ");
                print(itoa(PWM_L));
                print("\n");
            }
            else if(RxBuffer[0] == 'c' && RxBuffer[1] == 's' && RxBuffer[2] == 'r'){
                SENS_R = !SENS_R;
                print("sens R : ");
                print(itoa(SENS_R));
                print("\n");
            }
            else if(RxBuffer[0] == 'c' && RxBuffer[1] == 's' && RxBuffer[2] == 'l'){
                SENS_L = !SENS_L;
                print("sens L : ");
                print(itoa(SENS_L));
                print("\n");
            }
            else if(RxBuffer[0] == 'S' && RxBuffer[1] == '0' && RxBuffer[2] == ' ' && RxBuffer[3] == 'u' && RxBuffer[4] == 'p'){
                servoUs(SERVO0,SERVO0_UP);
            }
            else if(RxBuffer[0] == 'S' && RxBuffer[1] == '0' && RxBuffer[2] == ' ' && RxBuffer[3] == 'd' && RxBuffer[4] == 'o' && RxBuffer[5] == 'w' && RxBuffer[6] == 'n'){
                servoUs(SERVO0,SERVO0_DOWN);
            }
            else if(RxBuffer[0] == 's'){
                state = 1;
            }
            else{
                print("error\n");
            }
            iRx = 0;
        }
        
            
    }
    IFS0bits.U1RXIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U2RXInterrupt(void)
{
    if(U2STAbits.OERR == 1)
        U2STAbits.OERR = 0;
    
    while(U2STAbits.URXDA != 0)
    {
        /*LED_0 = !LED_0;
        char c = U2RXREG;
        U2TXREG = c; // Loopback*/
        if(iRx2 >= RX_SIZE)
            iRx2 = 0;
        RxBuffer2[iRx2] = U2RXREG;
        if(RxBuffer2[iRx2] == '\n'){
            printRpi("\\n");
        }
        else{
             if(TxOn2 == 1)
                push2(RxBuffer2[iRx2]);
             else{
                TxOn2 = 1;
                U2TXREG = RxBuffer2[iRx2];
            }
        }
        if(RxBuffer2[iRx2] == '\n'){
            printRpi("\n");
            //set
            if(RxBuffer2[0] == 's' && RxBuffer2[1] == 't' && RxBuffer2[2] == 'a' && RxBuffer2[3] == 'r' && RxBuffer2[4] == 't' && RxBuffer2[5] == '\n' ){
                stop = 0;
            }
            else if(RxBuffer2[0] == 't' && RxBuffer2[1] == '\n'){
                LED_0 = !LED_0;
                LED_1 = !LED_1;
                LED_2 = !LED_2;
            }
            else if(RxBuffer2[0] == 's' && RxBuffer2[1] == 't' && RxBuffer2[2] == 'o' && RxBuffer2[3] == 'p'&& RxBuffer2[4] == '\n' ){
                stop = 1;
            }
            else if(RxBuffer2[0] == 's' && RxBuffer2[1] == '\n'){
                stop = 1;
            }
            else if(RxBuffer2[0] == 's' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 't' && RxBuffer2[3] == ' '){
                if(RxBuffer2[4] == 's' && RxBuffer2[5] == 't' && RxBuffer2[6] == 'a' && RxBuffer2[7] == 't' && RxBuffer2[8] == 'e' && RxBuffer2[9] == ' '){
                    if(stop){
                        state = RxBuffer2[10] - '0';
                        printRpi("state = ");
                        printRpi(itoa((int)state));
                        printRpi("\n");
                    }
                }
            }
            //get
            else if(RxBuffer2[0] == 'g' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 't' && RxBuffer2[3] == ' '){ //get US 0/1 checked
                if(RxBuffer2[4] == 'U' && RxBuffer2[5] == 'S' && RxBuffer2[6] == ' '){
                    char n = RxBuffer2[7] - '0';
                    if(n < 0 || n >= NB_US){
                        printRpi("error invalid US id\n");
                    }
                    else{
                        printRpi("US[");
                        printRpi(itoa((int)n));
                        printRpi("] = ");
                        printRpi(itoa((int)US[(unsigned char)n]));
                        printRpi("\n");
                    }
                }
                else if(RxBuffer2[4] == 's' && RxBuffer2[5] == 't' && RxBuffer2[6] == 'a' && RxBuffer2[7] == 't' && RxBuffer2[8] == 'e'){
                    printRpi("state = ");
                    printRpi(itoa((int)state));
                    printRpi("\n");
                }
                else{
                    printRpi("error invalid get argument\n");
                }
                    
            }
            //servo //CHECK
            else if(RxBuffer2[0] == 's' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 'r' && RxBuffer2[3] == 'v' && RxBuffer2[4] == 'o' && RxBuffer2[5] == ' ' && RxBuffer2[7] == ' '){
                unsigned char n = 0;
                unsigned int val = 0;
                while(RxBuffer2[8+n] != '\n' && n <= 4){
                    val = val*10 + RxBuffer2[8+n] - '0';
                    n++;
                }
                if(n == 5 || n == 0){
                    printRpi("error invalid servo value\n");
                }
                
                switch(RxBuffer2[6]){
                    case '0' :
                        servoUs(SERVO0,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 0\n");
                        break;
                    case '1' :
                        servoUs(SERVO1,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 1\n");
                        break;
                    case '2' :
                        servoUs(SERVO2,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 2\n");
                        break;
                    case '3' :
                        servoUs(SERVO3,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 3\n");
                        break;
                    case '4' :
                        servoUs(SERVO4,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 4\n");
                        break;
                    case '5' :
                        servoUs(SERVO5,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 5\n");
                        break;
                    case '6' :
                        servoUs(SERVO6,val);
                        printRpi(itoa((int)val));
                        printRpi(" sent to Servo 6\n");
                        break;
                    default :
                        printRpi("error invalid servo id\n");
                        break;
                }
            }
            //go CHECK
            else if(RxBuffer2[0] == 'g' && RxBuffer2[1] == 'o' && RxBuffer2[2] == ' '){
                unsigned char n = 0;
                char positive = 1;
                if(RxBuffer2[3] == '-'){
                    positive = 0;
                    n++;
                }
                int x = 0;
                while(RxBuffer2[3+n] != ' ' && n <= 5){
                    x = x*10 + RxBuffer2[3+n] - '0';
                    n++;
                }
                if(n + positive == 6 || n == 0){    //if x has 5 digits or 6digits - 1 ('-') = 5
                    printRpi("error invalid x value\n");
                }
                if(!positive)
                    x = -x;
                
                unsigned char m = 0;
                positive = 1;
                if(RxBuffer2[3+n+1] == '-'){
                    positive = 0;
                    m++;
                }
                int y = 0;
                while(RxBuffer2[3+n+1+m] != '\n' && m <= 5){
                    y = y*10 + RxBuffer2[3+n+1+m] - '0';
                    m++;
                }
                if(m + positive == 6 || m == 0){    //if x has 5 digits or 6digits - 1 ('-') = 5
                    printRpi("error invalid y value\n");
                }
                if(!positive)
                    y = -y;
                if(x >= 0 && x <= 3000 && y >= 0 && y <= 2000){
                    receivedX = (double)x;
                    receivedY = (double)y;
                    newPosReceived = 1;
                    printRpi("received [go] : x = ");
                    printRpi(itoa(x));
                    printRpi(" & y = ");
                    printRpi(itoa(y));
                    printRpi("\n");
                }
                else
                    printRpi("error invalid x or y value\n");
            }
            //turn CHECK
            else if(RxBuffer2[0] == 't' && RxBuffer2[1] == 'u' && RxBuffer2[2] == 'r' && RxBuffer2[3] == 'n' && RxBuffer2[4] == ' '){
                unsigned char n = 0;
                char positive = 1;
                if(RxBuffer2[5] == '-'){
                    positive = 0;
                    n++;
                }
                int t = 0;
                while(RxBuffer2[5+n] != '\n' && n <= 4){
                    t = t*10 + RxBuffer2[5+n] - '0';
                    n++;
                }
                if(n + positive == 5 || n == 0){    //if x has 4 digits or 5 digits - 1 ('-') = 4
                    printRpi("error invalid t value\n");
                }
                if(!positive)
                    t = -t;
                 
                if(t > -180 && t <= 180){
                    receivedTheta = ((double)t)*2*PI/360;
                    newAngleReceived = 1;
                    printRpi("received [turn] : ");
                    printRpi(itoa(t));
                    printRpi("\n");
                }  
                else
                    printRpi("error invalid t value\n");
            }
            /*turn without limit*/
            else if(RxBuffer2[0] == 'T' && RxBuffer2[1] == 'u' && RxBuffer2[2] == 'r' && RxBuffer2[3] == 'N' && RxBuffer2[4] == ' '){
                unsigned char n = 0;
                char positive = 1;
                if(RxBuffer2[5] == '-'){
                    positive = 0;
                    n++;
                }
                int t = 0;
                while(RxBuffer2[5+n] != '\n' && n <= 4){
                    t = t*10 + RxBuffer2[5+n] - '0';
                    n++;
                }
                if(n + positive == 5 || n == 0){    //if x has 4 digits or 5 digits - 1 ('-') = 4
                    printRpi("error invalid t value\n");
                }
                if(!positive)
                    t = -t;

                receivedTheta = ((double)t)*2*PI/360;
                newAngleReceived = 1; 
                printRpi("received [TurN] NO LIMITS ! : ");
                printRpi(itoa(t));
                printRpi("\n");
            }
            //relGo
            else if(RxBuffer2[0] == 'r' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 'l' && RxBuffer2[3] == 'G' && RxBuffer2[4] == 'o' && RxBuffer2[5] == ' '){
                
            }
            //relTurn
            else if(RxBuffer2[0] == 'r' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 'l' && RxBuffer2[3] == 'T' && RxBuffer2[4] == 'u' && RxBuffer2[5] == 'r' && RxBuffer2[6] == 'n' && RxBuffer2[7] == ' '){
                
            }
            //motor
            /*else if(RxBuffer2[0] == 'm' && RxBuffer2[1] == 'o' && RxBuffer2[2] == 't' && RxBuffer2[3] == 'o' && RxBuffer2[4] == 'r' && RxBuffer2[5] == ' '){
                unsigned char n = 0;
                int val = 0;
                char positive = 1;
                if(RxBuffer2[8] == '-'){
                    positive = 0;
                    n++;
                }
                while(RxBuffer2[8+n] != '\n' && n <= 4){
                    val = val*10 + RxBuffer2[8+n] - '0';
                    n++;
                }
                if(n == 5 || n == 0){
                    printRpi("error invalid servo value\n");
                }
                if(val >= 0 && val <= 70){//(val >-100 && val <= 100){  //70 ~= 6.5V
                    switch(RxBuffer2[6]){
                        case '0' :
                            SENS_ACT_0 = 0; //NE PAS CHANGER LE SENS SINON CA VA VOUS PETEZ AU NEZ !
                            SDC2 = val * 70;
                            if(!positive){
                                //SENS_ACT_0 = 0;   //SERIEUSEMENT JE DECONNE PAS NE DECOMMENTE PAS CES LIGNES
                                val = -val; //for display
                            }
                            else{
                                //SENS_ACT_0 = 1;   //TU JOUES AVEC LE FEU
                            }
                            printRpi(itoa((int)val));
                            printRpi(" sent to motor0\n");
                            break;
                        case '1' :
                            PDC2 = val * 70;
                            if(!positive){
                                SENS_ACT_1 = 0;
                                val = -val; //for display
                            }
                            else{
                                SENS_ACT_1 = 1;
                            }
                            printRpi(itoa((int)val));
                            printRpi(" sent to motor1\n");
                            break;
                        default :
                            printRpi("error invalid motor id\n");
                            break;
                    }
                }
                else{
                    printRpi("invalid motor value\n");
                }
            }*/
            else if(RxBuffer2[0] == 'c' && RxBuffer2[1] == 'a' && RxBuffer2[2] == 'n' && RxBuffer2[3] == 'o' && RxBuffer2[4] == 'n' && RxBuffer2[5] == ' ' && RxBuffer2[6] == 'o' && RxBuffer2[7] == 'n' && RxBuffer2[8] == '\n'){
                //SDC2 = (int)(2.8/VBAT*7000);
                //LATBbits.LATB12 = 1;
                LATBbits.LATB13 = 1;
                printRpi("received [canon on]\n");
            }
            else if(RxBuffer2[0] == 'c' && RxBuffer2[1] == 'a' && RxBuffer2[2] == 'n' && RxBuffer2[3] == 'o' && RxBuffer2[4] == 'n' && RxBuffer2[5] == ' ' && RxBuffer2[6] == 'o' && RxBuffer2[7] == 'f' && RxBuffer2[8] == 'f' && RxBuffer2[9] == '\n'){
                //SDC2 = 0;
                //LATBbits.LATB12 = 0;
                LATBbits.LATB13 = 0;
                printRpi("received [canon off]\n");
            }
            else if(RxBuffer2[0] == 'a' && RxBuffer2[1] == 'x' && RxBuffer2[2] == '1' && RxBuffer2[3] == '2' && RxBuffer2[4] == ' ' && RxBuffer2[6] == ' '){
                unsigned char n = 0;
                unsigned int val = 0;
                while(RxBuffer2[7+n] != '\n' && n <= 4){
                    val = val*10 + RxBuffer2[7+n] - '0';
                    n++;
                }
                if(n == 5 || n == 0 || val > 1023){
                    printRpi("error invalid servo value\n");
                }
                char L = (char)(val&0xff);
                char H = (char)((val>>8)&0x3);
                switch(RxBuffer2[6]){
                    case '1' :
                        rotateToAX12(AX12_ID_1,L,H);
                        break;
                    case '3' :
                        rotateToAX12(AX12_ID_3,L,H);
                        break;
                    default :
                        printRpi("error invalid AX12 id (1 or 3)\n");
                        break;
                }
            }
            else if(RxBuffer2[0] == 'd' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 'b' && RxBuffer2[3] == 'u' && RxBuffer2[4] == 'g' && RxBuffer2[5] == ' '){
                debugPosRpi = !debugPosRpi;
            }
            else if(RxBuffer2[0] == 'd' && RxBuffer2[1] == 'e' && RxBuffer2[2] == 'b' && RxBuffer2[3] == 'u' && RxBuffer2[4] == 'g' && RxBuffer2[5] == 'A' && RxBuffer2[6] == ' '){
                debugPosRpiAsserv = !debugPosRpiAsserv;
            }
            else{
                printRpi("error : invalid cmd\n");
            }
            /*switch(RxBuffer2[0]){
                case GET:
                    switch(RxBuffer2[1]){
                        case _TMR1:
                            printRpi(itoa((int)TMR1));
                            LED_1 = !LED_1;
                            break;
                        case _TMR2:
                            printRpi(itoa(TMR2));
                            LED_2 = !LED_2;
                            break;
                        default:
                            printRpi("error\n");
                            break;
                    }
                    break;
                case SET:
                    switch(RxBuffer2[1]){
                        case _TMR1:
                            break;
                        case _TMR2:
                            break;
                        default:
                            printRpi("error\n");
                            break;
                    }
                    break;
                default:
                    printRpi("error\n");
                    break;
            }*/
            iRx2 = 0;
        }
        else{
            iRx2++;
        }
            
    }
    IFS1bits.U2RXIF = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    if(iD != iF){
        U1TXREG = pop();
    }
    else
        TxOn = 0;
}

void __attribute__((interrupt,no_auto_psv)) _U2TXInterrupt(void)
{
    IFS1bits.U2TXIF = 0; // Clear TX Interrupt flag
    if(iD2 != iF2){
        U2TXREG = pop2();
    }
    else
        TxOn2 = 0;
}

void push(char c){
    TxLoopBuffer[iF] = c;
    iF++;
    if(iF == TX_SIZE)
        iF = 0;
}
void push2(char c){
    TxLoopBuffer2[iF2] = c;
    iF2++;
    if(iF2 == TX_SIZE)
        iF2 = 0;
}
char pop(){
    char r = TxLoopBuffer[iD];
    iD++;
    if(iD == TX_SIZE)
        iD = 0;
    return r;
}
char pop2(){
    char r = TxLoopBuffer2[iD2];
    iD2++;
    if(iD2 == TX_SIZE)
        iD2 = 0;
    return r;
}

void print(char *str){
    unsigned char i = 0;
    if(TxOn == 1)
        push(str[0]);
    for(i = 1; str[i] != '\0'; i++){
        push(str[i]);
    }
    if(TxOn == 0){
        TxOn = 1;
        U1TXREG = str[0];
    }
}
void printRpi(char *str){
    unsigned char i = 0;
    if(TxOn2 == 1)
        push2(str[0]);
    for(i = 1; str[i] != '\0'; i++){
        push2(str[i]);
    }
    if(TxOn2 == 0){
        TxOn2 = 1;
        U2TXREG = str[0];
    }
}
char *itoa(int value) 
 {
     static char buffer[12];        // 12 bytes is big enough for an INT32
     int original = value;        // save original value
 
     int c = sizeof(buffer)-1;
 
     buffer[c] = 0;                // write trailing null in last byte of buffer    
 
     if (value < 0)                 // if it's negative, note that and take the absolute value
         value = -value;
     
     do                             // write least significant digit of value that's left
     {
         buffer[--c] = (value % 10) + '0';    
         value /= 10;
     } while (value);
 
     if (original < 0) 
         buffer[--c] = '-';
 
     return &buffer[c];
 }