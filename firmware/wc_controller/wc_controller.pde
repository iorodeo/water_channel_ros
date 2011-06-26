#include <SPI.h>
#include <Streaming.h>
#include <util/atomic.h>
#include <Servo.h>
#include "wc_controller.h"
#include "io_pins.h"
#include "mcp4822.h"
#include "SystemState.h"
#include "SledMotor.h"
#include "MessageHandler.h"


SystemState sysState = SystemState();
SledMotor sledMotor = SledMotor();
MessageHandler messageHandler = MessageHandler();

void setup() {
    
    // Initialize system
    setupCommunication();
    setupTimer();
    sledMotor.initializeIO();
}

void setupCommunication() {
    // Setup serial and SPI communications
    Serial.begin(BAUDRATE);
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV8);
    SPI.begin();
}

void setupTimer() {
    // Set timer control registers and top value 
    TCCR2A = TIMER_TCCR2A;
    TCCR2B = TIMER_TCCR2B;
    OCR2A =  TIMER_TOP;
    // Enable overflow interrupt  
    TIMSK2 |= (1<<TOIE2) | (0<<OCIE2A);
    TCNT2 = 0;
}

void loop() {
    // Handle serial communications
    while (Serial.available() > 0) {
        messageHandler.process(Serial.read());
        if (messageHandler.messageReady()== true) {
            //messageHandler.printMessage();
        }
        messageHandler.switchYard(sysState);
    }

    //sendData();
}

void sendData() {
    SystemState sysStateCopy;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (sysState.sendDataFlag == true) {
            sysStateCopy = sysState;
            sysState.sendDataFlag = false;
        }
    }
    if (sysStateCopy.sendDataFlag==true) {
        Serial << sysStateCopy.operatingMode << " ";
        Serial << sysStateCopy.motorCommand << " ";
        Serial << endl;
    }
}

ISR(TIMER2_OVF_vect) { 
    long error;
    static unsigned int loopCnt = 0;
    //int motorCommand = 0;

    if (loopCnt < RT_LOOP_TOP) {
        // Only run realtime processing everytime the loop counter rolls over.
        loopCnt++;
        return;
    }
    loopCnt = 0;
    sysState.sendDataFlag = true;

    switch (sysState.operatingMode) {

        case SYS_MODE_OFF:
            sledMotor.off();
            break;

        case SYS_MODE_MOTOR_CMD: 
            sledMotor.setVelocity(sysState.motorCommand);
            break;

        //case SYS_MODE_INERTIAL:
            break;

        default:
            sledMotor.off();
            break;
    }

    // Check the watch dog counter
    sysState.watchDogCnt++;
    if (sysState.watchDogCnt >= SYS_WATCHDOG_MAX) {
        sysState.motorCommand = 0;
        sledMotor.off();
    }
}

