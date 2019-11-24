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
pthread_t printThread_handler; // used in main_freertos.c to latch on to the printThread
sem_t sema;

void *printThread(void *arg0){
    sema = semaHandlerReturn(); // Get the semaphore handle that was initialized in initsUtilsCommands.c function semaInit()

    while(1){
        sem_wait(&sema); // Thread is blocked until "sema" is posted
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2); // Turn off LED Blue
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // Turn on LED Green
        printBuff(); // print buff0 or buff1 depending on which is storing
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // Turn off LED Green
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); // Turn on LED Blue
    }
}




