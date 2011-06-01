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

                case CMD_UPDATE_SETPT:
                    updateSetPoint(sysState);
                    break;

                case CMD_UPDATE_POSITION:
                    updatePosition(sysState);

                case CMD_UPDATE_ACTUATOR_VALUE:
                    updateActuatorValue(sysState);

                default:
                    break;

            } //switch
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

void MessageHandler::updatePosition(SystemState &sysState) {
    float newPos;
    //Serial << "updatePosition, numberOfItems = " << _DEC(numberOfItems()) << endl;
    if (numberOfItems() < 2) {
        return;
    }
    newPos = readFloat(1);
    //Serial << "newPos = " << newPos << endl;
    sysState.updatePosition(newPos);
}

void MessageHandler::updateActuatorValue(SystemState &sysState) {
    int newValue;
    if (numberOfItems() < 2) {
        return;
    }
    newValue = readInt(1);
    sysState.updateActuatorValue(newValue);

}

