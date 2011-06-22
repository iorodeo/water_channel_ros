#include <util/atomic.h>
#include "WProgram.h"
#include "wc_controller.h"
#include "SystemState.h"
#include "SledMotor.h"

extern SledMotor sledMotor;

SystemState::SystemState() {
    operatingMode = SYS_MODE_OFF;
    motorCommand = 0;
    actuatorValue = 0;
    sendDataFlag = false;
}

void SystemState::setModeOff() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        operatingMode = SYS_MODE_OFF; 
        motorCommand = 0;
        actuatorValue = 0; // Maybe not what we want
    }
}

void SystemState::setModeMotorCmd() {
    if (operatingMode != SYS_MODE_OFF) {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        controller.reset();
        operatingMode = SYS_MODE_MOTOR_CMD;
        motorCommand = 0.0;
    }
}

void SystemState::updateActuatorValue(int value) {
    actuatorValue = value;
}

void SystemState::updateMotorCmd(int value) {
    motorCommand = value;
}

