#ifndef MessageHandler_h
#define MessageHandler_h
#include "SerialReceiver.h"
#include "SystemState.h"

#define CMD_SET_MODE_OFF        0
#define CMD_SET_MODE_MOTOR_CMD  4 
#define CMD_SET_DATA_STREAM_ON  5
#define CMD_SET_DATA_STREAM_OFF 6
#define CMD_GET_ANALOG_INPUT    7

#define CMD_UPDATE_PWM_VALUE    56 
#define CMD_UPDATE_MOTOR_CMD    57 
#define CMD_UPDATE_WATCHDOG     60

class MessageHandler: public SerialReceiver {
    public:
        void switchYard(SystemState &sysState);
    private:

        // Set mode commands
        void setModeOff(SystemState &sysState);
        void setModeMotorCmd(SystemState &sysState);

        // Data stream commands
        void setDataStreamOn(SystemState &sysState);
        void setDataStreamOff(SystemState &sysState);

        // Get commands
        void getAnalogInput(SystemState &sysState);

        // Update value commands
        void updatePWMValue(SystemState &sysState);
        void updateMotorCmd(SystemState &sysState);
        void updateWatchDog(SystemState &sysState);
};

#endif
