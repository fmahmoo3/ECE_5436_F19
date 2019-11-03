#ifndef INITSUTILSCOMMANDS_H_
#define INITSUTILSCOMMANDS_H_

#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
#include <ti/drivers/PWM.h>
#include <ti/drivers/Timer.h>
#include <semaphore.h>

/* Peripheral Init Functions */
extern void uartInit();
extern void adcInit();

/* Writing to UART Functions */
extern void getChar(char *val);
extern void putChar(char *val);
extern void putString(char *val);
extern int length(char *a);

/* Commands UART Functions */
extern void commandsInit();
extern int commandUnderstood(char a, char b);
extern void runCommand(char a, char b);
extern void toggleRed();
extern void toggleGreen();
extern void toggleBlue();
extern void rightSensorRead();
extern void frontSensorRead();


#endif /* INITSUTILSCOMMANDS_H_ */
