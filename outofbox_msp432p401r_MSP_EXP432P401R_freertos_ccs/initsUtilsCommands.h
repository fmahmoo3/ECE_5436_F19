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
extern void pwmInit();
extern void timerInit();
extern void semInit();


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
extern void forwards();
extern void backwards();
extern void stop();
extern void highSpeed();
extern void rotateRight();
extern void rotateLeft();


/* PWM Helper Functions */
extern uint32_t calculateDutyCycle(uint32_t percent);
extern void changeDutyCyclePercent(uint32_t percent, uint8_t motor);
extern void changeDutyCycle(uint32_t val, uint8_t motor);


/* PID Function */
extern void pid();


/* Ping Pong Functions */
extern void thinLineStatusChange(int val);
extern uint8_t mazeStarted();
extern void thickLineStatusChange(int val);
extern void saveToBuffer(int pid_error);
extern void printBuff();
extern sem_t semaHandlerReturn();


#endif /* INITSUTILSCOMMANDS_H_ */
