#ifndef INITS_H_
#define INITS_H_

#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/Timer.h>
#include <semaphore.h>

/* Init Functions */
extern UART_Handle uartInit();

/* Helper Functions */
extern void getChar(char *val);
extern void putString(char *val);

/* Global Variables */
extern void (*lookUpTable[26][26])(int arg1, int arg2) = {{NULL}};

#endif /* INITS_H_ */
