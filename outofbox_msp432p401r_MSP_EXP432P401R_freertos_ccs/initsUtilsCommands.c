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
#include <ti/drivers/PWM.h>
#include <ti/drivers/Timer.h>
#include "ti_drivers_config.h"

/* Local Header Files */
#include "initsUtilsCommands.h"

/* Global Variables */
char str[25]; // used to print numbers


/* Handle to write to UART */
UART_Handle uart_handle;


/* Handle to read ADC Value */
ADC_Handle adc_front_handle;
ADC_Handle adc_right_handle;


/* PWM Variables */
PWM_Handle pwm_left_handle;
PWM_Handle pwm_right_handle;
const uint32_t MOTORMAXPERIOD = 10000;
const uint8_t left = 0;
const uint8_t right = 1;
uint32_t leftMotorDutyCycle;
uint32_t rightMotorDutyCycle;


/* Timer Variables */
Timer_Handle timer_handle;
int timeThroughMaze;


/*  */
uint16_t adcFrontValue;
uint16_t adcRightValue;
void (*lookUpTable[26][26])() = {{NULL}};


/* Maze Started variables */
sem_t sema;
uint8_t thinLineStatus = 0;
uint8_t thickLineStatus = 0;
uint8_t mazeStartedStatus = 0;
uint8_t saveValue = 0;
uint8_t useBuff = 0;
int buff0[20];
uint8_t buff0Index = 0;
int buff1[20];
uint8_t buff1Index = 0;






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

    ADC_Params params;

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

void pwmInit(){
    PWM_Params pwmParams;

    //Initialize GPIO Out Pins to control direction of motors
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6); // Left Motor Control P3.6
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN6); // Right Motor Control P2.6

    // Initialize the PWM driver.
    PWM_init();

    // Initialize the PWM parameters
    PWM_Params_init(&pwmParams);
    pwmParams.idleLevel = PWM_IDLE_LOW;         // Output low when PWM is not running
    pwmParams.periodUnits = PWM_PERIOD_US;      // Period is in us
    pwmParams.periodValue = MOTORMAXPERIOD;
    pwmParams.dutyUnits = PWM_DUTY_US;    // Duty is in fractional percentage
    pwmParams.dutyValue = 0;                    // 0% initial duty cycle

    // Open the PWM left motor instance
    pwm_left_handle = PWM_open(CONFIG_PWM_LEFT_MOTOR, &pwmParams); //P3.7
    if (pwm_left_handle == NULL) {
        // PWM_open() failed
        while (1);
    }

    PWM_start(pwm_left_handle); // start PWM with 0% duty cycle


    // Open the PWM right motor instance
    pwm_right_handle = PWM_open(CONFIG_PWM_RIGHT_MOTOR, &pwmParams); //P2.4
    if (pwm_right_handle == NULL) {
        // PWM_open() failed
        while (1);
    }

    PWM_start(pwm_right_handle); // start PWM with 0% duty cycle
}

void timerInit(){
    Timer_Params params;
    Timer_init();

    // Initialize Timer parameters
    Timer_Params_init(&params);
    params.periodUnits = Timer_PERIOD_US;
    params.period = 50000; // 50000 us = 50 ms
    params.timerMode  = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = pid;

    // Open Timer instance
    timer_handle = Timer_open(CONFIG_TIMER_PID, &params);
    if (timer_handle == NULL) {
        // Timer_open() failed
        while (1);
    }
}

void semInit(){
    int retc = 0;
    retc = sem_init(&sema, 0, 0);
    if (retc == -1) {
        while (1);
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
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);


    lookUpTable['t'-'a']['r'-'a'] = &toggleRed; //tr; toggle red led
    lookUpTable['t'-'a']['g'-'a'] = &toggleGreen; //tg; toggle green led
    lookUpTable['t'-'a']['b'-'a'] = &toggleBlue; //tb; toggle blue led
    lookUpTable['d'-'a']['f'-'a'] = &frontSensorRead; //df; distance sensor front read
    lookUpTable['d'-'a']['r'-'a'] = &rightSensorRead; //dr; distance sensor right read
    lookUpTable['g'-'a']['o'-'a'] = &start; //go; runs both motors in robots forward direction
    lookUpTable['r'-'a']['e'-'a'] = &backwards; //re; runs both motors in robots backwards direction
    lookUpTable['s'-'a']['t'-'a'] = &stop; //st; turn motors off
    lookUpTable['h'-'a']['s'-'a'] = &highSpeed; //hs; increase duty cycle to 100%, no specific direction
    lookUpTable['r'-'a']['r'-'a'] = &rotateRight; //rr; rotates robot towards right, no specific speed
    lookUpTable['r'-'a']['l'-'a'] = &rotateLeft; //rl; rotates robot towards left, no specific speed
//    lookUpTable['i'-'a']['p'-'a'] = &increaseKp; //ip; increases kp by .2
//    lookUpTable['d'-'a']['p'-'a'] = &decreaseKp; //dp; decreases kp by .2
//    lookUpTable['i'-'a']['i'-'a'] = &increaseKi; //ii; increases ki by .2
//    lookUpTable['d'-'a']['i'-'a'] = &decreaseKi; //di; decreases ki by .2
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
//    putString("Red LED was Toggled");
//
//    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0) == GPIO_INPUT_PIN_HIGH){
//        putString(" HIGH");
//    }
//    else{
//        putString(" LOW");
//    }
}

void toggleGreen(){
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1);
//    putString("Green LED was Toggled");
//
//    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == GPIO_INPUT_PIN_HIGH){
//        putString(" HIGH");
//    }
//    else{
//        putString(" LOW");
//    }
}

void toggleBlue(){
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2);
//    putString("Blue LED was Toggled ");
//
//    if(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2) == GPIO_INPUT_PIN_HIGH){
//        putString("HIGH");
//    }
//    else{
//        putString("LOW");
//    }
}

void frontSensorRead(){
    uint_fast16_t res = ADC_convert(adc_front_handle, &adcFrontValue);

    if (res != ADC_STATUS_SUCCESS){
        putString("ADC Convert Failed!");
    }
    else{
        putString("Front Distance Sensor Reading is ");
        sprintf(str,"%u",adcFrontValue);
        putString(&str);
    }
}

void rightSensorRead(){
    uint_fast16_t res = ADC_convert(adc_right_handle, &adcRightValue);

    if (res != ADC_STATUS_SUCCESS){
        putString("ADC Convert Failed!");
    }
    else{
        putString("Right Distance Sensor Reading is ");
        sprintf(str,"%u",adcRightValue);
        putString(&str);
    }
}

void start(){
    //Initialize Motor so robot moves in forward direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    changeDutyCyclePercent(100, left);
    changeDutyCyclePercent(100, right);

    putString("Robot is starting maze");

    timeThroughMaze = 0;
    mazeStartedStatus = 1;
    thinLineStatus = 0;
    thickLineStatus = 0;
    saveValue = 0;
    useBuff = 0;
    buff0Index = 0;
    buff1Index = 0;

    Timer_start(timer_handle);
}

void backwards(){
    //Initialize Motor so robot moves in forward direction
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    changeDutyCyclePercent(75, left);
    changeDutyCyclePercent(75, right);

//    putString("Robot is moving backwards");
}

void stop(){
    // Since we are stopping the motors, we do not need to set the motor control directions

    leftMotorDutyCycle = 0;  // set duty cycle to 0%
    rightMotorDutyCycle = 0;  // set duty cycle to 0%

    changeDutyCyclePercent(0, left);
    changeDutyCyclePercent(0, right);

    putString("Robot has been stopped");

    mazeStartedStatus = 0;

    sprintf(str, "\n\r%lf seconds\n\r", (double) timeThroughMaze/1000);
    putString(str);
    Timer_stop(timer_handle);
}

void highSpeed(){
    // Since we are simply increase speed the motors, we do not need to set the motor control directions

    changeDutyCyclePercent(100, left);
    changeDutyCyclePercent(100, right);

//    putString("Robot is moving at High Speed");
}

void rotateRight(){
    // Since we are simply rotating, we do not need to adjust the motor speed

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

//    putString("Robot is rotating right");
}

void rotateLeft(){
    // Since we are simply rotating, we do not need to adjust the motor speed

    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

//    putString("Robot is rotating left");
}





/*
 *
 *
 * PWM Helper Functions
 *
 *
 */
uint32_t calculateDutyCycle(uint32_t percent){
    return MOTORMAXPERIOD * percent / 100;
}

void changeDutyCyclePercent(uint32_t percent, uint8_t motor){
    if(motor == right){
        rightMotorDutyCycle = calculateDutyCycle(percent);
        PWM_setDuty(pwm_right_handle, rightMotorDutyCycle);
    }
    else{
        leftMotorDutyCycle = calculateDutyCycle(percent);
        PWM_setDuty(pwm_left_handle, leftMotorDutyCycle);
    }
}

void changeDutyCycle(uint32_t val, uint8_t motor){
    if(motor == right){
        if(val<=2000)
            rightMotorDutyCycle = 2000;
        else if(val<=MOTORMAXPERIOD)
            rightMotorDutyCycle = val;
        else
            rightMotorDutyCycle = MOTORMAXPERIOD;

        PWM_setDuty(pwm_right_handle, rightMotorDutyCycle);
    }
    else{
        if(val<=2000)
            leftMotorDutyCycle = 2000;
        else if(val<=MOTORMAXPERIOD)
            leftMotorDutyCycle = val;
        else
            leftMotorDutyCycle = MOTORMAXPERIOD;

        PWM_setDuty(pwm_left_handle, leftMotorDutyCycle);
    }
}





/*
 *
 *
 * PID Functions
 *
 *
 */
int pid_error;
double kp = 2.5;
double ki = .85;
double kd = 0;

double p = 0;
double i = 0;
double d = 0;

uint32_t middle = 7600;
uint32_t thresholdFront = 7000;
uint32_t thresholdRight = 5000;
int last_error = 0;
double u;

void pid(){
    ADC_convert(adc_right_handle, &adcRightValue);
    ADC_convert(adc_front_handle, &adcFrontValue);

    if(adcFrontValue >= thresholdFront && adcRightValue >= thresholdRight){
        //Rotate Left
        GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

        changeDutyCycle(MOTORMAXPERIOD, right);
        changeDutyCycle(MOTORMAXPERIOD, left);

        while(adcFrontValue >= 5000){
            ADC_convert(adc_front_handle, &adcFrontValue);
        }
    }


    //Initialize Motor so robot moves in forward direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    ADC_convert(adc_right_handle, &adcRightValue);

    pid_error = middle-adcRightValue;

    if(pid_error<=300 && pid_error>=-300){
        pid_error = 0;
    }

    p = (kp*pid_error);
    i = ki*(pid_error+last_error);
    d = kd*(pid_error-last_error);

    u = p+i+d;

    last_error = pid_error;

    if(u<0){
        changeDutyCycle(MOTORMAXPERIOD - abs(u), left);
        changeDutyCycle(MOTORMAXPERIOD, right);

        //putString("too close \n\r");
    }
    else if(u>0){
        changeDutyCycle(MOTORMAXPERIOD, left);
        changeDutyCycle(MOTORMAXPERIOD - u, right);

        //putString("too far \n\r");
    }
    else{
        changeDutyCycle(MOTORMAXPERIOD, left);
        changeDutyCycle(MOTORMAXPERIOD, right);

        //putString("just right! \n\r");
    }

    timeThroughMaze += 50;
    if(thinLineStatus == 1){
        saveToBuffer(pid_error);
    }
}


//void increaseKp(){
//    kp += .2;
//    sprintf(str, "kp: %lf\n\r", kp);
//    putString(str);
//}
//
//void decreaseKp(){
//    kp -= .2;
//    sprintf(str, "kp: %lf\n\r", kp);
//    putString(str);
//}
//
//void increaseKi(){
//    ki += .2;
//    sprintf(str, "ki: %lf\n\r", ki);
//    putString(str);
//}
//
//void decreaseKi(){
//    ki -= .2;
//    sprintf(str, "ki: %lf\n\r", ki);
//    putString(str);
//}



/*
 *
 *
 * Ping Pong Functions
 *
 *
 */

void thinLineStatusChange(int val){
    thinLineStatus = val;
}

void thickLineStatusChange(int val){
    thickLineStatus = val;
}

uint8_t mazeStarted(){
    return mazeStartedStatus;
}

void saveToBuffer(int pid_error){
    if(saveValue == 1){
        if(useBuff == 0){
            //Use buff0
            buff0[buff0Index++] = pid_error;
        }
        else{
            //Use buff1
            buff1[buff1Index++] = pid_error;
        }

        if(useBuff == 0 && buff0Index == 20){
            useBuff = 1;
            buff0Index = 0;
            //Signal to print buff0
            sem_post(&sema);
        }

        if(useBuff == 1 && buff1Index == 20){
            useBuff = 0;
            buff1Index = 0;
            //Signal to print buff1
            sem_post(&sema);
        }


        saveValue = 0;
    }
    else{
        saveValue = 1;
    }
}

void printBuff(){
    putString("\n\n\r");
    int k;
    toggleGreen();
    if(useBuff == 1){ // saving to buff1 so print buff0
        for(k = 0; k<20; k++){
            if(k%5 == 0){
                toggleGreen();
            }

            sprintf(str,"%d\n\r",buff0[k]);
            putString(str);
        }
    }
    else{
        for(k = 0; k<20; k++){
            if(k%5 == 0){
                toggleGreen();
            }

            toggleGreen();
            sprintf(str,"%d\n\r",buff1[k]);
            putString(str);
        }
    }
}

sem_t semaHandlerReturn(){
    return sema;
}


