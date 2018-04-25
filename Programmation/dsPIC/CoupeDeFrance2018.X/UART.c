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

char TxLoopBuffer[TX_SIZE];
char TxLoopBuffer2[TX_SIZE];
char RxBuffer[RX_SIZE];
char RxBuffer2[RX_SIZE];
unsigned char iD,iF,iD2,iF2;                //index of first data and of first free element
unsigned char TxOn,TxOn2;
unsigned char iRx,iRx2;
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
        LED_0 = !LED_0;
        char c = U2RXREG;
        U2TXREG = c; // Loopback
        /*if(iRx2 >= RX_SIZE)
            iRx2 = 0;
        RxBuffer2[iRx2] = U2RXREG;
        iRx2++;
        if(RxBuffer2[iRx2-1] == '\n'){
            switch(RxBuffer2[0]){
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
            }
            iRx2 = 0;
        }*/
        
            
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