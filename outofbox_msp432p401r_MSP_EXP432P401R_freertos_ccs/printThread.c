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
        toggleBlue();// Turns it off if on, which it should be
//        printBuff();
        toggleBlue();
    }
}




