/* Includes */
#include <initsUtilsCommands.h>
#include <stdlib.h>
#include <stdio.h>

/* Driver Header Files */
#include <ti/drivers/UART.h>

/* Local Header Files */
#include "uartThread.h"
#include "initsUtilsCommands.h"

/* Global Variables */
pthread_t uartThread_handler;   // used in main_freertos.c to latch on to the uartThread

void *uartThread(void *arg0) {
    commandsInit();
    putString("\n\n\rBeginning Program\r\n");

    char inChar1;
    char inChar2;

    while(1){
        putString("\nEnter your command: ");
        getChar(&inChar1);
        if(inChar1 != ' ') {
            putChar(&inChar1);

            getChar(&inChar2);
            putChar(&inChar2);

            putString("\n\r");

            if(commandUnderstood(inChar1, inChar2) == 1){
                putString("\nYour Command Was Understood: ");
                runCommand(inChar1, inChar2);
                putString("\n\n\r");
            }
            else{
                putString("\nYour Command Was NOT Understood\n\n\r");
            }
        }
    }
}




