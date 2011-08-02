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

                case CMD_SET_DATA_STREAM_ON:
                    setDataStreamOn(sysState);
                    break;

                case CMD_SET_DATA_STREAM_OFF:
                    setDataStreamOff(sysState);
                    break;

                case CMD_UPDATE_PWM_VALUE:
                    updatePWMValue(sysState);
                    break;

                case CMD_UPDATE_MOTOR_CMD:
                    updateMotorCmd(sysState);
                    break;

                case CMD_UPDATE_WATCHDOG:
                    updateWatchDog(sysState);
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

void MessageHandler::setDataStreamOn(SystemState &sysState) {
    sysState.setDataStreamOn();
}

void MessageHandler::setDataStreamOff(SystemState &sysState) {
    sysState.setDataStreamOff();
}

void MessageHandler::updatePWMValue(SystemState &sysState) {
    int pwmNum;
    int newValue;
    if (numberOfItems() < 3) {
        return;
    }
    pwmNum = readInt(1);
    newValue = readInt(2);
    sysState.updatePWMValue(pwmNum, newValue);
}

void MessageHandler::updateMotorCmd(SystemState &sysState) {
    int newValue;
    if (numberOfItems() < 2) {
        return;
    }
    newValue = readInt(1);
    sysState.updateMotorCmd(newValue);
}

void MessageHandler::updateWatchDog(SystemState &sysState) {
    sysState.updateWatchDog();
}
