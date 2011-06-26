#include <util/atomic.h>
#include "WProgram.h"
#include "wc_controller.h"
#include "SystemState.h"
#include "SledMotor.h"


extern SledMotor sledMotor;

SystemState::SystemState() {
    operatingMode = SYS_MODE_OFF;
    motorCommand = 0;
    watchDogCnt = 0;
    pwm[0].attach(SYS_PWM_0_PIN, SYS_PWM_MIN_US, SYS_PWM_MAX_US);
    pwm[0].writeMicroseconds(SYS_PWM_START_US); 
    pwm[1].attach(SYS_PWM_1_PIN, SYS_PWM_MIN_US, SYS_PWM_MAX_US);
    pwm[1].writeMicroseconds(SYS_PWM_START_US); 
    sendDataFlag = false;
}

void SystemState::setModeOff() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        operatingMode = SYS_MODE_OFF; 
        motorCommand = 0;
        for (int i=0; i<SYS_NUM_PWM; i++) {
            pwm[i].writeMicroseconds(SYS_PWM_START_US);
        }
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

void SystemState::updatePWMValue(int num, int value) {
    if ((num >=0) && (num < SYS_NUM_PWM)) {
        if ((value >= SYS_PWM_MIN_US) && (value <= SYS_PWM_MAX_US)) {
            pwm[num].writeMicroseconds(value);
        }
    }
}

void SystemState::updateMotorCmd(int value) {
    motorCommand = value;
}

void SystemState::updateWatchDog() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        watchDogCnt = 0;
    }
}

