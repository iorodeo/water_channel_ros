#include "WProgram.h"
#include "Streaming.h"
#include "MessageHandler.h"


void MessageHandler::switchYard(SystemState &sysState) {

    int cmdId;

    if (messageReady()==true) {

        if (numberOfItems() > 0) {
            // Get command identifier
            cmdId = readInt(0);
            //Serial << "cmdId: " <<_DEC(cmdId) << endl;

            // Take action based on command Id
            switch (cmdId) {

                case CMD_SET_MODE_OFF:
                    setModeOff(sysState);
                    break;

                case CMD_SET_MODE_TRACKING:
                    setModeTracking(sysState);
                    break;

                case CMD_SET_MODE_CAPTIVE:
                    setModeCaptive(sysState);
                    break;

                case CMD_SET_MODE_INERTIAL:
                    setModeInertial(sysState);
                    break;

                case CMD_SET_MODE_MOTOR_CMD:
                    setModeMotorCmd(sysState);
                    break;

                case CMD_UPDATE_SETPT:
                    updateSetPoint(sysState);
                    break;

                case CMD_UPDATE_POSITION:
                    updatePosAndVel(sysState);
                    break;

                case CMD_UPDATE_ACTUATOR_VALUE:
                    updateActuatorValue(sysState);
                    break;

                case CMD_UPDATE_MOTOR_CMD:
                    updateMotorCmd(sysState);
                    break;

                case CMD_UPDATE_TEST_FORCE:
                    updateTestForce(sysState);

                default:
                    break;

            } // switch (cmdId)
        }

        reset(); // Clear out message

    } // if ((messageReady() ...
}

void MessageHandler::setModeOff(SystemState &sysState) {
    sysState.setModeOff();
}

void MessageHandler::setModeTracking(SystemState &sysState) {
    sysState.setModeTracking();
}

void MessageHandler:: setModeCaptive(SystemState &sysState) {
    sysState.setModeCaptive();
}

void MessageHandler::setModeInertial(SystemState &sysState) {
    sysState.setModeInertial();
}

void MessageHandler::setModeMotorCmd(SystemState &sysState) {
    sysState.setModeMotorCmd();
}

void MessageHandler::updateSetPoint(SystemState &sysState) {
    float newPos;
    float newVel;
    //Serial << "updateSetPoint, numberOfItems = " << _DEC(numberOfItems()) << endl;
    if (numberOfItems() < 3) {
        return;
    }
    newPos = readFloat(1);
    newVel = readFloat(2);
    //Serial << "newPos = " << newPos << ", newVel = " << newVel << endl;
    sysState.updateSetPoint(newPos,newVel);
}

void MessageHandler::updatePosAndVel(SystemState &sysState) {
    float newPos;
    float newVel;
    //Serial << "updatePosition, numberOfItems = " << _DEC(numberOfItems()) << endl;
    if (numberOfItems() < 2) {
        return;
    }
    newPos = readFloat(1);
    newVel = readFloat(2);
    //Serial << "newPos = " << newPos << endl;
    sysState.updatePosAndVel(newPos,newVel);
}

void MessageHandler::updateActuatorValue(SystemState &sysState) {
    int newValue;
    if (numberOfItems() < 2) {
        return;
    }
    newValue = readInt(1);
    sysState.updateActuatorValue(newValue);

}

void MessageHandler::updateMotorCmd(SystemState &sysState) {
    int newValue;
    if (numberOfItems() < 2) {
        return;
    }
    newValue = readInt(1);
    sysState.updateMotorCmd(newValue);
}

void MessageHandler::updateTestForce(SystemState &sysState) {
    float newValue;
    if (numberOfItems() < 2) {
        return;
    }
    newValue = readFloat(1);
    sysState.updateTestForce(newValue);
}
