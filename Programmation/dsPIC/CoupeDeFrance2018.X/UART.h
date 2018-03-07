#ifndef UART_H
#define	UART_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include "constant.h"
#include "PID.h"

void initUART();
char pop();
void push(char c);
void print(char *str);
char *itoa(int value);

#endif	/* UART_H */
