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
#include "ti_drivers_config.h"

/* Local Header Files */
#include "initsUtilsCommands.h"

/* Global Variables */
UART_Handle uart_handle;

ADC_Handle adc_front_handle;
ADC_Handle adc_right_handle;

PWM_Handle pwm_left_handle;
PWM_Handle pwm_right_handle;

uint32_t leftMotorDutyCycle;
uint32_t rightMotorDutyCycle;

uint16_t adcFrontValue;
uint16_t adcRightValue;
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
    pwmParams.idleLevel = PWM_IDLE_LOW;      // Output low when PWM is not running
    pwmParams.periodUnits = PWM_PERIOD_HZ;   // Period is in Hz
    pwmParams.periodValue = 1e6;             // 1MHz
    pwmParams.dutyUnits = PWM_DUTY_FRACTION; // Duty is in fractional percentage
    pwmParams.dutyValue = 0;                 // 0% initial duty cycle

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
    lookUpTable['g'-'a']['o'-'a'] = &forwards; //go; runs both motors in robots forward direction
    lookUpTable['r'-'a']['e'-'a'] = &backwards; //re; runs both motors in robots backwards direction
    lookUpTable['s'-'a']['t'-'a'] = &stop; //st; turn motors off
    lookUpTable['h'-'a']['s'-'a'] = &highSpeed; //hs; increase duty cycle to 100%, no specific direction
    lookUpTable['r'-'a']['r'-'a'] = &rotateRight; //rr; rotates robot towards right, no specific speed
    lookUpTable['r'-'a']['l'-'a'] = &rotateLeft; //rl; rotates robot towards left, no specific speed
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
    uint_fast16_t res = ADC_convert(adc_front_handle, &adcFrontValue);

    if (res != ADC_STATUS_SUCCESS){
        putString("ADC Convert Failed!");
    }
    else{
        putString("Front Distance Sensor Reading is ");
        char str[10];
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
        char str[10];
        sprintf(str,"%u",adcRightValue);
        putString(&str);
    }
}

void forwards(){
    //Initialize Motor so robot moves in forward direction
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    leftMotorDutyCycle = 37;  // set duty cycle to 37%
    rightMotorDutyCycle = 37;  // set duty cycle to 37%

    PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
    PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));

    putString("Robot is moving forwards");
    pid();
}

void backwards(){
    //Initialize Motor so robot moves in forward direction
    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    leftMotorDutyCycle = 37;  // set duty cycle to 37%
    rightMotorDutyCycle = 37;  // set duty cycle to 37%

    PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
    PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));

    putString("Robot is moving backwards");
}

void stop(){
    // Since we are stopping the motors, we do not need to set the motor control directions

    leftMotorDutyCycle = 0;  // set duty cycle to 0%
    rightMotorDutyCycle = 0;  // set duty cycle to 0%

    PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
    PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));

    putString("Robot has been stopped");
}

void highSpeed(){
    // Since we are simply increase speed the motors, we do not need to set the motor control directions

    leftMotorDutyCycle = 100;  // set duty cycle to 100%
    rightMotorDutyCycle = 100;  // set duty cycle to 100%

    PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
    PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));

    putString("Robot is moving at High Speed");
}

void rotateRight(){
    // Since we are simply rotating, we do not need to adjust the motor speed

    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    putString("Robot is rotating right");
}

void rotateLeft(){
    // Since we are simply rotating, we do not need to adjust the motor speed

    GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6); //Left Motor
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN6); //Right Motor

    putString("Robot is rotating left");
}

void pid(){
    int pid_error;
    double kp = .25;
    double ki = 0;
    double kd = 0;

    double p = 0;
    double i = 0;
    double d = 0;

    uint32_t middle = 7400; // constant value representing distance from right when in desired middle of corridor
    uint32_t rIRs;
    int last_error = 0;
    double pwm_pid;


    const TickType_t xDelay = 50 / portTICK_PERIOD_MS;

    while(1){
        vTaskDelay( xDelay );// simulating a 50ms wait time

        rightSensorRead();
        rIRs = adcRightValue;

        pid_error = middle-rIRs;

        p = (kp*pid_error);
        i = ki*(pid_error+last_error);
        d = kd*(pid_error-last_error);

        last_error = pid_error;
        pwm_pid = p+i+d;

        if(pid_error < 0){
                    // Too close to the wall, needs to move left

                    leftMotorDutyCycle -= pwm_pid;
                    rightMotorDutyCycle = 37;

                    PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
                    PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));

                    // no need to set directions because will be moving forward already :)
                    putString(" Too close to wall \n\r" );
                }
        else if(pid_error<=1500){
            //no error, keep moving forward

            leftMotorDutyCycle = 37;  // set duty cycle to 37%
            rightMotorDutyCycle = 37;  // set duty cycle to 37%

            PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
            PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));

            putString("Keep moving forward \n\r");
        }
        else{
            // Too far from the wall, needs to move right
            // steer to right: slow down right motor and set left motor to 255 adjust

            rightMotorDutyCycle -= pwm_pid;
            leftMotorDutyCycle = 37;

            PWM_setDuty(pwm_left_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * leftMotorDutyCycle) / 100));
            PWM_setDuty(pwm_right_handle, (uint32_t) (((uint64_t) PWM_DUTY_FRACTION_MAX * rightMotorDutyCycle) / 100));


            // no need to set directions because will be moving forward already :)
            putString(" Too far from wall \n\r" );
        }


//        ADCIntClear(ADC1_BASE, 2);
//        ADCProcessorTrigger(ADC1_BASE, 2);
//
//        while(!ADCIntStatus(ADC1_BASE, 2, false)){
//            //wait
//        }
//
//        ADCSequenceDataGet(ADC1_BASE, 2, ui32ADC1Value);
//
//        if(ui32ADC1Value[0] >3900){
//            stopMoving(0,0);
//            putString(" Something in front \n\r" );
//            rotateRight(0,0);
//            Task_sleep(1000);
//            stopMoving(0,0);
//            moveForward(0,0);
//        }
    }


}


