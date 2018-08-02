#include "stdint.h"
#include "stdbool.h"

#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/tm4c123gh6pm.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"

#include "sch.h"
#include "ser.h"
#include "btn.h"

//#############################################//
//                    Defines
//#############################################//

#define STEPPER_RANGE           56
#define STEPPER_OPEN            STEPPER_RANGE
#define STEPPER_CLOSED          10

//Milliseconds/step
#define STEPPER_SPEED_INIT      100
#define STEPPER_SPEED_HOME      300
#define STEPPER_SPEED_GOTO      30

#define STEPPER_ACCEL           3
#define STEPPER_ACCEL_DIST      (STEPPER_SPEED_INIT - STEPPER_SPEED_GOTO)/STEPPER_ACCEL

#define STEPPER_STATE_HOME1     0   // Find limit fast
#define STEPPER_STATE_HOME2     1   // Move away from limit
#define STEPPER_STATE_HOME3     2   // Find limit slow
#define STEPPER_STATE_IDLE      3   // Wait for command
#define STEPPER_STATE_GOTO      4   // Goto with trapezoidal profile

#define STEPPER_DIR_PLUS        true
#define STEPPER_DIR_MINUS       false

#define GPIO_ENABLE             GPIO_PIN_5
#define GPIO_DIR                GPIO_PIN_6
#define GPIO_STEP               GPIO_PIN_7

#define GPIO_LIMIT              GPIO_PIN_7

#define BUTTON_HOLD_THRESHOLD   2000

#define BUTTON_HOLD_SPEED       1
#define BUTTON_HOLD_RATE        500

//#############################################//
//                   Variables
//#############################################//

void OnButtonHold(_Bool bt1, _Bool btn2);

extern uint32_t currentTime;

uint8_t Stepper_state = STEPPER_STATE_HOME1;
int32_t Stepper_currentPosition = -1;
uint8_t Stepper_currentSpeed = STEPPER_SPEED_INIT;
int32_t Stepper_targetPosition = 0;
uint32_t Stepper_stepTimer = 0;

_Bool btnState1 = false;
_Bool btnState2 = false;
uint32_t btnTimer1 = 0;
uint32_t btnTimer2 = 0;

//#############################################//
//                 User Functions
//#############################################//

_Bool ReadLimitSwitch() {
    return (GPIOPinRead(GPIO_PORTA_BASE, GPIO_LIMIT) & GPIO_LIMIT) == 0;
}

void LimitOnFallingEdge() {
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_LIMIT);
}

void Stepper_Enable() {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_ENABLE, 0);
}

void Stepper_Disable() {
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_ENABLE, GPIO_ENABLE);
}

void Stepper_Step(_Bool dir) {

    if(dir) {
        // Step in the positive direction (STEPPER_DIR_PLUS)

        // Set DIR to LOW
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_DIR, 0);

        // Delay a short amount
        SysCtlDelay(1000);

        // Set STEP to HIGH
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_STEP, GPIO_STEP);

        // Delay a short amount
        SysCtlDelay(20000);

        // Set STEP to LOW
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_STEP, 0);
        Stepper_currentPosition++;
    } else {
        // Step in the negative direction (STEPPER_DIR_MINUS)

        // Set DIR to HIGH
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_DIR, GPIO_DIR);

        // Delay a short amount
        SysCtlDelay(1000);

        // Set STEP to HIGH
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_STEP, GPIO_STEP);

        // Delay a short amount
        SysCtlDelay(20000);

        // Set STEP to LOW
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_STEP, 0);

        Stepper_currentPosition--;
    }
}

uint8_t Task_StepperInitialize(void) {
    // Initialize A4988 GPIO pins
    //  DIR:    PC6
    //  STEP:   PC7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)) {}

    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_ENABLE | GPIO_STEP | GPIO_DIR);

    GPIOPinWrite(GPIO_PORTC_BASE,  GPIO_ENABLE | GPIO_STEP | GPIO_DIR, GPIO_ENABLE); // Set to LOW, enable is inverted

    // Initialize Limit Switch GPIO pin
    //  NO:     PA7
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {}

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_LIMIT);

    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_LIMIT, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    // Register, configure and enable the limit switch interrupt handler
    GPIOIntRegister(GPIO_PORTA_BASE, LimitOnFallingEdge);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_LIMIT, GPIO_FALLING_EDGE);
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_LIMIT);

    // Initialize the stepper in home mode
    Stepper_state = STEPPER_STATE_HOME1;
    Stepper_Enable();
    return 0;
}

_Bool Task_StepperUpdate(uint8_t taskID){
    // Stepper motor statemachine
    int32_t distance;
    switch(Stepper_state) {
    case STEPPER_STATE_HOME1:   // Find limit fast
        if(ReadLimitSwitch()) {
            Stepper_state = STEPPER_STATE_HOME2;
        } else {
            // Step negative until limit switch is read
            if(Stepper_stepTimer == 0) {
                Stepper_Step(STEPPER_DIR_MINUS);
                Stepper_stepTimer = STEPPER_SPEED_HOME;
            }
        }
        break;
    case STEPPER_STATE_HOME2:   // Move away from limit
        if(!ReadLimitSwitch()) {
            Stepper_state = STEPPER_STATE_HOME3;
        } else {
            // Step negative until limit switch is read
            if(Stepper_stepTimer == 0) {
                Stepper_Step(STEPPER_DIR_PLUS);
                Stepper_stepTimer = STEPPER_SPEED_HOME;
            }
        }
        break;
    case STEPPER_STATE_HOME3:   // Find limit slowly
        if(ReadLimitSwitch()) {
            Stepper_currentPosition = 0;
            Stepper_targetPosition = STEPPER_CLOSED;
            Stepper_state = STEPPER_STATE_IDLE;
            Stepper_Disable();
            SerialPrintln("HOME COMPLETE");
        } else {
            // Step negative until limit switch is read
            if(Stepper_stepTimer == 0) {
                Stepper_Step(STEPPER_DIR_MINUS);
                Stepper_stepTimer = STEPPER_SPEED_HOME;
            }
        }
        break;
    case STEPPER_STATE_IDLE:    // Wait for/Parse commands
        if(Stepper_currentPosition != Stepper_targetPosition) {
            Stepper_currentSpeed = STEPPER_SPEED_INIT;
            Stepper_stepTimer = 0;
            Stepper_Enable();
            Stepper_state = STEPPER_STATE_GOTO;
        }
        break;
    case STEPPER_STATE_GOTO:   // Goto (trapezoidal movement)
        if(Stepper_stepTimer == 0) {
            if(Stepper_targetPosition > Stepper_currentPosition) {
                SerialPrintlnInt(Stepper_currentSpeed);
                Stepper_Step(STEPPER_DIR_PLUS);
                Stepper_stepTimer = Stepper_currentSpeed;

                distance = Stepper_targetPosition - Stepper_currentPosition + 1;
            } else if(Stepper_targetPosition < Stepper_currentPosition) {
                SerialPrintlnInt(Stepper_currentSpeed);
                Stepper_Step(STEPPER_DIR_MINUS);
                Stepper_stepTimer = Stepper_currentSpeed;

                distance = Stepper_currentPosition - Stepper_targetPosition + 1;

            } else {
                // at targetPosition
                SerialPrintln("GOTO COMPLETE");
                Stepper_Disable();
                Stepper_state = STEPPER_STATE_IDLE;
                break;
            }
            if(distance <= (STEPPER_SPEED_INIT - Stepper_currentSpeed)/STEPPER_ACCEL + 1) {
                // Decelerate
                Stepper_currentSpeed += STEPPER_ACCEL;
            } else if (Stepper_currentSpeed > STEPPER_SPEED_GOTO) {
                // Accelerate
                Stepper_currentSpeed -= STEPPER_ACCEL;
            }
        }
        break;
    }

    if(Stepper_stepTimer > 0) {
        Stepper_stepTimer--;
    }

    _Bool btnHold1 = btnState1 && (currentTime - btnTimer1) >= BUTTON_HOLD_THRESHOLD;
    _Bool btnHold2 = btnState2 && (currentTime - btnTimer2) >= BUTTON_HOLD_THRESHOLD;

    if((btnHold1 || btnHold2) && (currentTime % BUTTON_HOLD_RATE == 0)) {
        //SerialPrintlnInt(currentTime);
        OnButtonHold(btnHold1, btnHold2);
    }

    // Returns true to set the task back to IDLE state
    return true;
}

/*
 * SerialOnCharReceived(character)
 *  Function to handle received character events over serial
 * This function is called in an interrupt, keep its exec time really short or use..
 *  other non-interrupt functions.
 */
void SerialOnCharReceived(char character) {
    //Echo the character
    //SerialWrite(character);
}

/*
 * SerialOnCharReceived(character)
 *  Function to handle received line events over serial configure the line endings..
 *   in the #define section found in ser.c.
 *  This function is called in an interrupt, keep its exec time really short or use..
 *   other non-interrupt functions.
 */
void SerialOnLineReceived(volatile char* stringStart, volatile char* stringEnd) {
    //Do nothing.. Below snippet is provided as an example
    char character = *stringStart;
    //Parse commands
    switch(character){
    case 'h':           // Home
        Stepper_state = STEPPER_STATE_HOME1;
        Stepper_Enable();
        break;
    default:
        if(character >= '0' && character <= '9') {
            //Character is a number, parse as position
            Stepper_targetPosition = ((character - '0') + 1) * (STEPPER_RANGE/10);
        }
        break;
    }
}

void OnButtonPushed(_Bool btn1, _Bool btn2) {
    if(btn1) {
        btnTimer1 = currentTime;
        btnState1 = true;
    } else {
        btnState1 = false;
        if((currentTime - btnTimer1) < BUTTON_HOLD_THRESHOLD) {
            Stepper_targetPosition = STEPPER_CLOSED;
        }
    }

    if(btn2) {
        btnTimer2 = currentTime;
        btnState2 = true;
    } else {
        if((currentTime - btnTimer2) < BUTTON_HOLD_THRESHOLD) {
            Stepper_targetPosition = STEPPER_OPEN;
        }
        btnState2 = false;
    }
}

void OnButtonHold(_Bool btn1, _Bool btn2) {
    if(btn1) {
        if((Stepper_targetPosition > STEPPER_CLOSED)) {
            Stepper_targetPosition -= BUTTON_HOLD_SPEED;
            if(Stepper_targetPosition < STEPPER_CLOSED)
                Stepper_targetPosition = STEPPER_CLOSED;
        }
    } else if(btn2) {
        if(Stepper_targetPosition < STEPPER_OPEN) {
            Stepper_targetPosition += BUTTON_HOLD_SPEED;
            if(Stepper_targetPosition > STEPPER_OPEN)
                Stepper_targetPosition = STEPPER_OPEN;
        }
    }
}

//#############################################//
//                     Setup()
//#############################################//
void Setup(void) {
    // Setting Clock to 80MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Add peripheral initializations here
    InitScheduler();
    InitSerial(&SerialOnCharReceived, &SerialOnLineReceived);
    InitButton(OnButtonPushed);


    // Add Tasks Here
    AddTaskTime(&Task_StepperInitialize, &Task_StepperUpdate, 1);
    //
}

//#############################################//
//                     Loop()
//#############################################//
void Loop(void) {
    UpdateScheduler();
}

//#############################################//
//                  Main Function
//#############################################//
/*  You shouldn't need to edit anything here as..
 *   its brought out to Setup() and Loop().
 */
int main(void) {
    Setup();
    while(1){
        Loop();
    }
}
