/* Driver Header Files */
#include <ti/devices/msp432p4xx/driverlib/gpio.h>

/* freeRtos include */
#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"

/* Local Header Files */
#include "lightSensorThread.h"
#include "initsUtilsCommands.h"

/* Global Variables */
pthread_t lightSensorThread_handler;   // used in main_freertos.c to latch on to the lightSensorThread

void *lightSensorThread(void *arg0) {
    int threshold = 1500;
    int decay = 0;
//    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);

    int k;
    int countOverBlackLine = 0;

    while(1) {
        decay = 0;
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);

        for(k=0; k<1000; k++) { }

        GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3);

        while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH ) {
           decay++;
        }

        if(decay >= threshold) {
//           GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green LED ON
           countOverBlackLine++;
        }
        else {
//           GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // Green LED OFF

           if(countOverBlackLine >= 8 && mazeStarted() == 1){
               thickLineStatusChange(1);
               stop();
           }
           else if(countOverBlackLine > 1 && mazeStarted() == 1){
               thinLineStatusChange(1);
           }

           countOverBlackLine = 0;
        }

        vTaskDelay( 10 / portTICK_PERIOD_MS );
   }
}




