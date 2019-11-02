#include <stdlib.h>
#include <stdio.h>

/*freeRtos include */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include <semaphore.h>


/* Driver Header Files */
#include <ti/devices/msp432p4xx/driverlib/gpio.h>
#include <ti/drivers/UART.h>
#include <ti/boards/MSP_EXP432P401R/Board.h>


UART_Handle uart_handle;

UART_Handle uartInit(){
    UART_init();                // Initializes TI's UART

    /* Define UART parameters*/
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

//    uart_handle = UART_open(Board_UART1, &uartParams); // UART1: RX = P3.2, TX = P3.3
    uart_handle = UART_open(Board_UART0, &uartParams); // UART0: RX = P1.2, TX = P1.3

    return uart_handle;
}

void getChar(char *val){
    UART_read(uart_handle, val, 1);
}

void putString(char *val){
    while(*val != 0){
        UART_write(uart_handle, *val++, 1);
    }


}

