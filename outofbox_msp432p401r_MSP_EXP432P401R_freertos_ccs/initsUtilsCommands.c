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
#include <ti/drivers/ADC.h>
#include "ti_drivers_config.h"

/* Local Header Files */
#include "initsUtilsCommands.h"

/* Global Variables */
UART_Handle uart_handle;
ADC_Handle adc_front_handle;
ADC_Handle adc_right_handle;
uint32_t adcFrontValueUv;
uint32_t adcRightValueUv;
void (*lookUpTable[26][26])() = {{NULL}};


/*
 *
 *
 * Peripheral Init Functions
 *
 *
 */
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

void adcInit(){
    ADC_init();

    ADC_Params   params;

    ADC_Params_init(&params);
    params.isProtected = false;

    adc_front_handle = ADC_open(CONFIG_ADC_FRONT, &params);// CONFIG_ADC_FRONT: P4.0 CONFIG_ADC_FRONT was made using the outofbox_msp432p401r.syscfg file

    if (adc_front_handle == NULL) {
        //ADC_open() failed;
        while (1);
    }

    ADC_Params_init(&params);
    params.isProtected = false;

    adc_right_handle = ADC_open(CONFIG_ADC_RIGHT, &params);// CONFIG_ADC_RIGHT: P4.1 CONFIG_ADC_Right was made using the outofbox_msp432p401r.syscfg file was made using the outofbox_msp432p401r.syscfg file

    if(adc_right_handle == NULL){
        //ADC_open() failed;
        while(1);
    }
}

/*
 *
 *
 * Writing to UART Functions
 *
 *
 */
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

/*
 *
 *
 * Commands UART Functions
 *
 *
 */
void commandsInit(){
    /* Initialize GPIO Output Pins for commands */
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0); // on board LED1 red LED
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1); // on board LED1 green LED
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); // on board LED1 blue LED

    lookUpTable['t'-'a']['r'-'a'] = &toggleRed; //tr toggle red led
    lookUpTable['t'-'a']['g'-'a'] = &toggleGreen; //tg toggle green led
    lookUpTable['t'-'a']['b'-'a'] = &toggleBlue; //tb toggle blue led
    lookUpTable['d'-'a']['f'-'a'] = &frontSensorRead; //df distance sensor front read
    lookUpTable['d'-'a']['r'-'a'] = &rightSensorRead; //dr distance sensor right read
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
    putString("Blue LED was Toggled ");

    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH){
        putString("HIGH");
    }
    else{
        putString("LOW");
    }
}

void frontSensorRead(){
    uint16_t adcValue;
    uint_fast16_t res = ADC_convert(adc_front_handle, &adcValue);

    if (res == ADC_STATUS_SUCCESS)
    {
        adcFrontValueUv = ADC_convertToMicroVolts(adc_front_handle, adcValue);
    }

    putString("Front Distance Sensor Reading in microVolts is ");
    char str[10];
    sprintf(str,"%lu",adcFrontValueUv);
    putString(&str);
}

void rightSensorRead(){
    uint16_t adcValue;
    uint_fast16_t res = ADC_convert(adc_right_handle, &adcValue);

    if (res == ADC_STATUS_SUCCESS)
    {
        adcRightValueUv = ADC_convertToMicroVolts(adc_right_handle, adcValue);
    }

    putString("Right Distance Sensor Reading in microVolts is ");
    char str[10];
    sprintf(str,"%lu",adcRightValueUv);
    putString(&str);
}

