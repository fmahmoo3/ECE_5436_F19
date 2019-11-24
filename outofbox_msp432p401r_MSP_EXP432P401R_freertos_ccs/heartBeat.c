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
 *  blinks a Red led on the MSP432P401R
 */
void *heartBeatThread(void *arg0){
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0); // LED 1 Red on Board

    while (1) {
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0); // If output is LOW, will set HIGH. And vice versa
        vTaskDelay( 500 / portTICK_PERIOD_MS ); // 500 [ms] Delay
    }
}

