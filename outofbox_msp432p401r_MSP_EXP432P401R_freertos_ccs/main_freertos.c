/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main_freertos.c ========
 */
/* Includes */
#include <stdlib.h>

/* POSIX Header Files */
#include <pthread.h>

/* RTOS Header Files */
#include <FreeRTOS.h>
#include <task.h>
#include "portmacro.h"

/* Driver Header Files */
#include <ti/drivers/GPIO.h>
#include <ti_drivers_config.h>


/* Local Header Files */
#include "heartBeat.h"
#include "uartThread.h"
#include "initsUtilsCommands.h"
#include "lightSensorThread.h"
#include "printThread.h"

/* Stack size in bytes */
#define THREADSTACKSIZE    2048


/*
 *  ======== main ========
 */
int main(void)
{
    pthread_attr_t      pAttrs;
    struct sched_param  priParam;
    int                 retc;
    int                 detachState;

    /* Call driver init functions */
    Board_init();
    uartInit();
    adcInit();
    pwmInit();
    timerInit();
    semInit();


    /* Set priority and stack size attributes */
    pthread_attr_init(&pAttrs);
    detachState = PTHREAD_CREATE_DETACHED;

    retc = pthread_attr_setdetachstate(&pAttrs, detachState);
    pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, THREADSTACKSIZE);

    if (retc != 0) {
        /* pthread_attr_setstacksize() failed */
        while (1);
    }
    /* End of setting priority and stack size attributes */


    /* Create heartBeat Thread with priority = 5 */
    priParam.sched_priority = 5;

    pthread_attr_setschedparam(&pAttrs, &priParam);
    retc = pthread_create(&heartBeatThread_handler, &pAttrs, heartBeatThread, NULL);

    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
    /* End of create heartBeat Thread*/


    /* Create UART Thread with priority = 1 */
    priParam.sched_priority = 1;

    pthread_attr_setschedparam(&pAttrs, &priParam);
    retc = pthread_create(&uartThread_handler, &pAttrs, uartThread, NULL);

    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
    /* End of create UART Thread*/

    /* Create Light Sensor Thread with priority = 2 */
    priParam.sched_priority = 2;

    pthread_attr_setschedparam(&pAttrs, &priParam);
    retc = pthread_create(&lightSensorThread_handler, &pAttrs, lightSensorThread, NULL);

    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
    /* End of create Light Sensor Thread*/

    /* Create print Thread with priority = 3 */
    priParam.sched_priority = 3;

    pthread_attr_setschedparam(&pAttrs, &priParam);
    retc = pthread_create(&printThread_handler, &pAttrs, printThread, NULL);

    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
    /* End of create Print Thread*/


    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    return (0);
}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    /* Handle Memory Allocation Errors */
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}

void vApplicationTickHook(void)
{
    /*
     * This function will be called by each tick interrupt if
     * configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h.  User code can be
     * added here, but the tick hook is called from an interrupt context, so
     * code must not attempt to block, and only the interrupt safe FreeRTOS API
     * functions can be used (those that end in FromISR()).
     */
}

void vPreSleepProcessing(uint32_t ulExpectedIdleTime)
{

}

//*****************************************************************************
//
//! \brief Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook(void)
{
    /* Handle Idle Hook for Profiling, Power Management etc */
}
