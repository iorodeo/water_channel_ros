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

                case CMD_SET_MODE_MOTOR_CMD:
                    setModeMotorCmd(sysState);
                    break;

                case CMD_UPDATE_ACTUATOR_VALUE:
                    updateActuatorValue(sysState);
                    break;

                case CMD_UPDATE_MOTOR_CMD:
                    updateMotorCmd(sysState);
                    break;

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

void MessageHandler::setModeMotorCmd(SystemState &sysState) {
    sysState.setModeMotorCmd();
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
