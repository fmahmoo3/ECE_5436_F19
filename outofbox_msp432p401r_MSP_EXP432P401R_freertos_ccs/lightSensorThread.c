/* Driver Header Files */
#include <ti/devices/msp432p4xx/driverlib/gpio.h>

/* freeRtos include */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

/* Local Header Files */
#include "lightSensorThread.h"

/* Global Variables */
pthread_t lightSensorThread_handler;   // used in main_freertos.c to latch on to the lightSensorThread

void *lightSensorThread(void *arg0) {
    int threshold = 2000;
    int decay = 0;
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

    while(1) {
        decay = 0;
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);
        vTaskDelay( xDelay );
        GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3);

        while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH) {
           decay++;
        }

        if(decay >= threshold) {
           GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green LED ON
        }
        else {
           GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green LED OFF
        }
   }
}




