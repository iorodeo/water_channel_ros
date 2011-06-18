#include <util/atomic.h>
#include "WProgram.h"
#include "wc_controller.h"
#include "SystemState.h"
#include "SledMotor.h"

extern SledMotor sledMotor;

SystemState::SystemState() {
    operatingMode = SYS_MODE_OFF;
    setPosition = 0.0;
    setVelocity = 0.0;
    position = 0.0;
    velocity = 0.0;
    positionError = 0.0;
    velocityError = 0.0;
    motorCommand = 0.0;
    sendDataFlag = false;
    force = 0.0;

    // Initialize controller and dynamics
    setGainsZero();
    dynamics.setMass(SYS_DFLT_DYNAMICS_MASS);
    dynamics.setDamping(SYS_DFLT_DYNAMICS_DAMPING);
    dynamics.setDt(1.0/((float) RT_LOOP_FREQ));
    dynamics.setVelocity(0.0);
}

void SystemState::setModeOff() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        operatingMode = SYS_MODE_OFF; 
        positionError = 0.0;
        velocityError = 0.0;
        motorCommand = 0.0;
    }
}

void SystemState::setModeTracking() {

    if (operatingMode != SYS_MODE_OFF) {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        updateError();
        if (fabs(positionError) > SYS_POS_ERROR_STARTUP_LIMIT) {
            operatingMode = SYS_MODE_OFF;
            positionError = 0.0;
            velocityError = 0.0;
            motorCommand = 0.0;
        }
        else {
            setGainsTracking();
            controller.reset();
            operatingMode = SYS_MODE_TRACKING;
        }
    }
}

void SystemState::setModeCaptive() {

    if (operatingMode != SYS_MODE_OFF) {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        force = 0.0;
        dynamics.setVelocity(0.0);
        updateError();
        if (fabs(positionError) > SYS_POS_ERROR_STARTUP_LIMIT) {
            operatingMode = SYS_MODE_OFF;
            positionError = 0.0;
            velocityError = 0.0;
            motorCommand = 0.0;
        }
        else {
            //setGainsTracking();
            setGainsVelControl();
            controller.reset();
            operatingMode = SYS_MODE_CAPTIVE;
        }
    }
}

void SystemState::setModeInertial() {

    if (operatingMode != SYS_MODE_OFF) {
        return;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        controller.reset();
        operatingMode = SYS_MODE_INERTIAL;
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

void SystemState::setModeVelControl() {
    if (operatingMode != SYS_MODE_OFF) {
        return;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        updateError();
        if (fabs(velocityError) > SYS_VEL_ERROR_STARTUP_LIMIT) {
            operatingMode = SYS_MODE_OFF;
            positionError = 0.0;
            velocityError = 0.0;
        }
        else {
            setGainsVelControl();
            controller.reset();
            operatingMode = SYS_MODE_VEL_CTL;
        }
    }
}

void SystemState::updateSetPoint(float pos, float vel) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setPosition = pos;
        setVelocity = vel;
        updateError();
    }
}

void SystemState::updatePosAndVel(float pos, float vel) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        position = pos;
        velocity = vel;
        updateError();
    }
}

void SystemState::updateError() {
    // Should always be called from within an atomic block
    positionError = setPosition - position;
    velocityError = setVelocity - velocity;
}

void SystemState::updateTestForce(float value) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        force = value;
    }
}

void SystemState::updateActuatorValue(int value) {
    actuatorValue = value;
}

void SystemState::updateMotorCmd(int value) {
    motorCommand = (float) value;
    sledMotor.setVelocity(value);
}

void SystemState::setGainsZero() {
    controller.setGains(0.0, 0.0, 0.0, 0.0);
}

void SystemState::setGainsTracking() {
    controller.setGains(
            SYS_DFLT_PGAIN_TRACKING,
            SYS_DFLT_IGAIN_TRACKING,
            SYS_DFLT_DGAIN_TRACKING,
            SYS_DFLT_FFGAIN_TRACKING
            );
}

void SystemState::setGainsVelControl() {
    controller.setGains(
            SYS_DFLT_PGAIN_VEL_CTL,
            SYS_DFLT_IGAIN_VEL_CTL,
            SYS_DFLT_DGAIN_VEL_CTL,
            SYS_DFLT_FFGAIN_VEL_CTL
            );
}

