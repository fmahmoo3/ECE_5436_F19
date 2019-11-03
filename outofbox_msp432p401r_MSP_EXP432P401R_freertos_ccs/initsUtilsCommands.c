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
#include "ti_drivers_config.h"

/* Local Header Files */
#include "initsUtilsCommands.h"

/* Global Variables */
UART_Handle uart_handle;
extern void (*lookUpTable[26][26])() = {{NULL}};

void uartInit(){
    UART_init();                // Initializes TI's UART


    /* Define UART parameters*/
    UART_Params uartParams;
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;

    uart_handle = UART_open(CONFIG_UART_BLUETOOTH, &uartParams); // CONFIG_UART_BLUETOOTH: RX = P3.2, TX = P3.3 CONFIG_UART_BLUETOOTH was made using the outofbox_msp432p401r.syscfg file

    if (uart_handle == NULL) {
        // UART_open() failed
        while (1);
    }
}

void getChar(char *val){
    UART_read(uart_handle, val, 1);
}

/* This was made since using putString depends on having string which has a \0 at the end. */
void putChar(char *val){
    UART_write(uart_handle, val, 1);
}

void putString(char *val){
    UART_write(uart_handle, val, length(val));
}

/* length() is a helper function for putString since UART_write() requires knowing number of chars in a string */
int length(char *a) {
    int i = 0;
    while (a[i] != '\0') {
        i++;
    }
    return i;
}

void commandsInit(){
    /* Initialize GPIO Output Pins for commands */
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // on board LED1 red LED
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); // on board LED1 green LED
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); // on board LED1 blue LED

    lookUpTable['t'-'a']['r'-'a'] = &toggleRed; //tr
    lookUpTable['t'-'a']['g'-'a'] = &toggleGreen; //tg
    lookUpTable['t'-'a']['b'-'a'] = &toggleBlue; //tb
}

int commandUnderstood(char a, char b){
    if(lookUpTable[a -'a'][b -'a'] != NULL){
        return 1;
    }
    else{
        return 0;
    }
}

void runCommand(char a, char b){
    lookUpTable[a -'a'][b -'a']();
}

void toggleRed(){
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN0);
    putString("Red LED was Toggled");

    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){
        putString(" HIGH");
    }
    else{
        putString(" LOW");
    }
}

void toggleGreen(){
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
    putString("Green LED was Toggled");

    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == GPIO_INPUT_PIN_HIGH){
        putString(" HIGH");
    }
    else{
        putString(" LOW");
    }
}

void toggleBlue(){
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
    putString("Blue LED was Toggled");

    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH){
        putString(" HIGH");
    }
    else{
        putString(" LOW");
    }
}

