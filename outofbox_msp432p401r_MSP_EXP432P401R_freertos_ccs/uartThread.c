/* Includes */
#include <stdlib.h>
#include <stdio.h>

/* Driver Header Files */
#include <ti/drivers/UART.h>

/* Local Header Files */
#include "uartThread.h"
#include "inits.h"

/* Global Variables */
pthread_t uartThread_handler;   // used in main_freertos.c to latch on to the uartThread

void *uartThread(void *arg0) {

    putString("Beginning Program\r\n");

    while(1){
        putString("im so sad\r\n");
    }

//    char inChar1;
//    char inChar2;
//
//    while(1){
//        getChar(&inChar1);
//        putString(inChar1);
//
//        getChar(&inChar2);
//        putString(inChar2);
//
//        putChar("\n\r");
//
//        if(lookUpTable[inChar1-'a'][inChar2-'a'] != NULL){
//            putString("\nYour Command Has Been Received:");
//            lookUpTable[inChar1-'a'][inChar2-'a'](0,0);
//        }
//        else{
//            putString("\nYour Command Was not understood\n\r");
//        }
//    }
}




