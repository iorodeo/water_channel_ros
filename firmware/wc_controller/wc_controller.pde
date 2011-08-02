#include <SPI.h>
#include <Streaming.h>
#include <util/atomic.h>
#include <Servo.h>
#include "wc_controller.h"
#include "io_pins.h"
#include "mcp4822.h"
#include "max1270.h"
#include "SystemState.h"
#include "SledMotor.h"
#include "MessageHandler.h"

SystemState sysState = SystemState();
SledMotor sledMotor = SledMotor();
MessageHandler messageHandler = MessageHandler();
MAX1270 analogIn = MAX1270(AIN_CS, AIN_SSTRB);

void setup() {
    
    // Initialize system
    setupCommunication();
    setupTimer();
    setupAnalogInput();
    sledMotor.initializeIO();
    sysState.initializeIO();
}

void setupAnalogInput() {
    analogIn.setBipolar();
    analogIn.setRange5V();
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

    // Send data to host computer
    sendData();
}

void sendData() {
    int ainValue[AIN_NUM];
    SystemState sysStateCopy;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (sysState.sendDataFlag == true) {
            sysStateCopy = sysState;
            sysState.sendDataFlag = false;
        }
    }
    if ((sysStateCopy.sendDataFlag==true) && (sysStateCopy.dataStreamFlag==true)) {

        // Send analog values back to controller
        Serial << "[";
        for (int i=0; i<2;i++) {
            Serial << _DEC(sysStateCopy.ainValue[i]) << ", "; 
        }
        Serial << "]" << endl;
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

    // Reset loop counter and set send data flag 
    loopCnt = 0;
    sysState.sendDataFlag = true;

    // Take actions based on operating mode
    switch (sysState.operatingMode) {

        case SYS_MODE_OFF:
            sledMotor.off();
            break;

        case SYS_MODE_MOTOR_CMD: 
            sledMotor.setVelocity(sysState.motorCommand);
            break;

        //case SYS_MODE_INERTIAL:
        //    break;

        default:
            sledMotor.off();
            break;
    }

    // Read analog inputs - how many can we read. 8 seems to be too many
    sysState.ainValue[0] = analogIn.sample(0);
    sysState.ainValue[1] = analogIn.sample(1);

    // Check the watch dog counter
    sysState.watchDogCnt++;
    if (sysState.watchDogCnt >= SYS_WATCHDOG_MAX) {
        sysState.motorCommand = 0;
        sledMotor.off();
    }
}

