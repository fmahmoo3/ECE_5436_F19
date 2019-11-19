/* Driver Header Files */
#include <ti/devices/msp432p4xx/driverlib/gpio.h>

/* freeRtos include */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

/* Local Header Files */
#include "printThread.h"
#include "initsUtilsCommands.h"

/* Global Variables */
pthread_t printThread_handler;   // used in main_freertos.c to latch on to the printThread
sem_t sema;

void *printThread(void *arg0){
    sema = semaHandlerReturn();

    while(1){
        sem_wait(&sema);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);// turn off LED Blue
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);// turn on LED Green
        printBuff();
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);// turn off LED Green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);// turn on LED Blue
    }
}




