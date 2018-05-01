#ifndef UART_H
#define	UART_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "constant.h"
#include "PID.h"
#include "AX12.h"

void initUART();
void initUART1();
void initUART2();
char pop();
char pop2();
void push(char c);
void push2(char c);
void print(char *str);
void printRpi(char *str);
char *itoa(int value);

/*typedef enum Cmd Cmd;
enum Cmd
{
    SET = 1, GET = 0
};*/
#define GET     1
#define SET     2

#define _TMR1   1
#define _TMR2   2

#endif	/* UART_H */
