/* Driver Header Files */
#include <ti/devices/msp432p4xx/driverlib/gpio.h>

/* freeRtos include */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

/* Local includes */
#include "heartBeat.h"

pthread_t heartBeatThread_handler;

/*
 *  blinks a blue led on the MSP432P401R
 */
void *heartBeatThread(void *arg0){
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2); // on board blue LED

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
        vTaskDelay( xDelay );
    }
}

