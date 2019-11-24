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
    int threshold = 1500; // value that tells robot that a black surface is seen
    int decay = 0; // value that will increment as reflectance sensor returns a HIGH input

    int k; // temp value for the for loop
    int countOverBlackLine = 0; // value that will tell robot how many times in a row a black surface was seen

    while(1) {
        decay = 0; // reset decay value everytime thread runs

        /* Process for reading value from reflectance sensor START */
        GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN3);
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN3);

        for(k=0; k<1000; k++) { }

        GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN3);

        while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3) == GPIO_INPUT_PIN_HIGH ) {
           decay++;
        }
        /* Process for reading value from reflectance sensor END */

        if(decay >= threshold) { // if black surface is seen, increment countOverBlackLine
           countOverBlackLine++;
        }
        else { // no longer over black surface
           if(countOverBlackLine >= 8 && mazeStarted() == 1){ // if maze is started and countOverBlackLine was for a thickLine
               thickLineStatusChange(1); // Lets the rest of the program know thickLine was crossed
               stop(); // calls function that will turn motors off, output time, and reset values for new run of maze
           }
           else if(countOverBlackLine > 1 && mazeStarted() == 1){ // if maze is started and countOverBlackLine was for a thinLine
               thinLineStatusChange(1); // Lets the rest of the program know thinLine was crossed which starts saving error values
               GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); // Turn on LED Blue
           }

           countOverBlackLine = 0; // reset count over black line value for next checks
        }

        vTaskDelay( 10 / portTICK_PERIOD_MS ); // delay of 10 [ms], decreases frequency of this thread running
   }
}




