/*
 * File:   UART.c
 * Author: Quentin BOYER
 *
 * Created on 31 octobre 2017, 21:10
 * 
 * Notes : Parfois des erreurs lors de l'envoi de plusieurs octets en meme temps depui la RPi, à vérifier en augmentant la clock
 */

#include "UART.h"

extern double xc;
extern int state;
extern PID pidAngle;
extern int R,L;

char TxLoopBuffer[TX_SIZE];
char RxBuffer[RX_SIZE];
unsigned char iD,iF;                //index of first data and of first free element
unsigned char TxOn;
unsigned char iRx;
void initUART() {
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
                LATGbits.LATG14 = !LATGbits.LATG14;
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == '+'){
                pidAngle.Kp+=1;
            }
            else if(RxBuffer[0] == 'p' && RxBuffer[1] == '-'){
                pidAngle.Kp-=1;
            }
            else if(RxBuffer[0] == 'd' && RxBuffer[1] == '+'){
                pidAngle.Kp+=1;
            }
            else if(RxBuffer[0] == 'd' && RxBuffer[1] == '-'){
                pidAngle.Kp-=1;
            }
            else if(RxBuffer[0] == 'r' && RxBuffer[1] == '+'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                R+=50;
                print(itoa(PDC5));
                print(itoa(SDC5));
                print("\n");
            }
            else if(RxBuffer[0] == 'r' && RxBuffer[1] == '-'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                R-=50;
            }
            else if(RxBuffer[0] == 'l' && RxBuffer[1] == '+'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                L+=50;
            }
            else if(RxBuffer[0] == 'l' && RxBuffer[1] == '-'){
                LATGbits.LATG14 = !LATGbits.LATG14;
                L-=50;
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


void __attribute__((interrupt,no_auto_psv)) _U1TXInterrupt(void)
{
    IFS0bits.U1TXIF = 0; // Clear TX Interrupt flag
    if(iD != iF){
        U1TXREG = pop();
    }
    else
        TxOn = 0;
}

void push(char c){
    TxLoopBuffer[iF] = c;
    iF++;
    if(iF == TX_SIZE)
        iF = 0;
}

char pop(){
    char r = TxLoopBuffer[iD];
    iD++;
    if(iD == TX_SIZE)
        iD = 0;
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