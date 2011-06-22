#include "WProgram.h"
#include "SledMotor.h"

void SledMotor::initializeIO() {
    // Create Analog output
    analogOut = MCP4822(AOUT_CS,AOUT_LDAC);

    // Configure analog outputs
    analogOut.setGain2X_AB();

    // Set SSR pins to output mode
    pinMode(SSR0,OUTPUT);
    pinMode(SSR1,OUTPUT);

    off();
}

void SledMotor::setVelocity(int value) {
    int modValue;
    modValue = deadBandComp(value);
    modValue = clamp(modValue);
    setDirection(modValue);
    setSpeed(modValue);
}

void SledMotor::setVelocity(float value) {
    float clampedValue;
    int intValue;
    clampedValue = clamp(value);
    intValue = (int) clampedValue;
    setVelocity(intValue);
}

void SledMotor::off() {
    // Set analog output value to zero and both direction lines to low.
    analogOut.setValue_A(0);
    digitalWrite(SSR0,LOW);
    digitalWrite(SSR1,LOW);
}

void SledMotor::setSpeed(int value) {
    int absValue;
    absValue = abs(value);
    analogOut.setValue_A(absValue);
}


void SledMotor::setDirection(int value) {
    if (value > 0) {
        digitalWrite(SSR1,LOW);
        digitalWrite(SSR0,HIGH);
    }
    else if (value < 0) {
        digitalWrite(SSR0,LOW);
        digitalWrite(SSR1,HIGH);
    }
    else {
        digitalWrite(SSR0,LOW);
        digitalWrite(SSR1,LOW);
    }
}

int SledMotor::clamp(int value) {
    int clampedValue;
    if (value > SM_MOTOR_CMD_CLAMP) {
        clampedValue = SM_MOTOR_CMD_CLAMP;
    }
    else if (value < -SM_MOTOR_CMD_CLAMP) {
        clampedValue = -SM_MOTOR_CMD_CLAMP;
    }
    else {
        clampedValue = value;
    }
    return clampedValue;
}

float SledMotor::clamp(float value) {
    static float sm_motor_cmd_clamp = (float) SM_MOTOR_CMD_CLAMP;
    float clampedValue;

    if (value > sm_motor_cmd_clamp) {
        clampedValue = sm_motor_cmd_clamp;
    }
    else if (value < -sm_motor_cmd_clamp) {
        clampedValue = -sm_motor_cmd_clamp;
    }
    else {
        clampedValue = value;
    }
    return clampedValue;
}

int SledMotor::deadBandComp(int value) {
    int compValue;
    compValue = value;
    if (compValue > 0) {
        compValue += SM_DRIVE_DEADBAND;
    }
    if (compValue < 0) {
        compValue -= SM_DRIVE_DEADBAND;
    }
    return compValue;
}
