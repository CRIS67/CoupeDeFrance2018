/* 
 * File: QEI.h  
 * Author: Quentin BOYER
 * Comments: 
 * Revision history: 1
 */

#ifndef QEI_H
#define	QEI_H

#include <xc.h> // include processor files - each processor file is guarded.  

void initQEI();
void initQEIPPS();
void initQEI1();
void initQEI2();
char *itoa(int value);

#endif	/* QEI_H */