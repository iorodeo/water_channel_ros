#include <util/atomic.h>
#include "WProgram.h"
#include "SystemState.h"
#include "SledMotor.h"

extern SledMotor sledMotor;

SystemState::SystemState() {
    operatingMode = SYS_MODE_OFF;
    setPosition = 0.0;
    setVelocity = 0.0;
    position = 0.0;
    positionError = 0.0;
    motorCommand = 0.0;
    sendDataFlag = false;
    controller.setGains(
            SYS_DFLT_PGAIN_TRACKING,
            SYS_DFLT_IGAIN_TRACKING,
            SYS_DFLT_DGAIN_TRACKING,
            SYS_DFLT_FFGAIN_TRACKING
            );
}

void SystemState::setModeOff() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        operatingMode = SYS_MODE_OFF; 
        positionError = 0.0;
        motorCommand = 0.0;
    }
}

void SystemState::setModeTracking() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        updatePositionError();
        if (fabs(positionError) > SYS_POSERROR_STARTUP_LIMIT) {
            operatingMode = SYS_MODE_OFF;
            positionError = 0.0;
            motorCommand = 0.0;
        }
        else {
            controller.reset();
            operatingMode = SYS_MODE_TRACKING;
        }
    }
}

void SystemState::setModeCaptive() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        controller.reset();
        operatingMode = SYS_MODE_CAPTIVE;
    }
}

void SystemState::setModeInertial() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        controller.reset();
        operatingMode = SYS_MODE_INERTIAL;
    }
}

void SystemState::setModeMotorCmd() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        controller.reset();
        operatingMode = SYS_MODE_MOTOR_CMD;
        motorCommand = 0.0;
    }
}

void SystemState::updateSetPoint(float pos, float vel) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setPosition = pos;
        setVelocity = vel;
        updatePositionError();
    }
}

void SystemState::updatePosition(float pos) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        position = pos;
        updatePositionError();
    }
}

void SystemState::updatePositionError() {
    // Should always be called from within an atomic block
    if (operatingMode == SYS_MODE_TRACKING) {
        positionError = setPosition - position;
    }
}

void SystemState::updateActuatorValue(int value) {
    actuatorValue = value;
    
}

void SystemState::updateMotorCmd(int value) {
    motorCommand = (float) value;
    sledMotor.setVelocity(value);
}
