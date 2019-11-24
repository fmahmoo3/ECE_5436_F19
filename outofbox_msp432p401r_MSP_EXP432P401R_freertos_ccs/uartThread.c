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
    commandsInit(); // Initialize the lookUpTable for commands
    putString("\n\n\rBeginning Program\r\n"); // print to UART, that board as been reset

    char inChar1; // Used to save the first character user inputs for commands
    char inChar2; // Used to save the second character user inputs for commands

    while(1){
        putString("\nEnter your command: "); // Let users know UART is ready to receive commands
        getChar(&inChar1); // Read first char user inputs

        if(inChar1 != ' ') { // If first Char is not empty, begin determining if command is valid and running command
            putChar(&inChar1); // echo value to UART so user knows what they wrote

            getChar(&inChar2); // Read second char form user input
            putChar(&inChar2); // echo value to UART so user knows what they wrote

            putString("\n\r"); // go to next line on UART console for clear formatting

            if(commandUnderstood(inChar1, inChar2) == 1){ // call to function where it can see the lookUpTable and determine it is a command
                putString("\nYour Command Was Understood: ");
                runCommand(inChar1, inChar2); // call to function where it can see the lookUpTable and run the command
                putString("\n\n\r");
            }
            else{
                putString("\nYour Command Was NOT Understood\n\n\r");
            }
        }
    }
}




