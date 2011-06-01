#ifndef MessageHandler_h
#define MessageHandler_h
#include "SerialReceiver.h"
#include "SystemState.h"


#define CMD_SET_MODE_OFF       0
#define CMD_SET_MODE_TRACKING  1
#define CMD_SET_MODE_CAPTIVE   2
#define CMD_SET_MODE_INERTIAL  3

#define CMD_UPDATE_SETPT             4
#define CMD_UPDATE_POSITION          5
#define CMD_UPDATE_ACTUATOR_VALUE    6 

class MessageHandler: public SerialReceiver {
    public:
        void switchYard(SystemState &sysState);
    private:
        void updateSetPoint(SystemState &sysState);
        void setModeOff(SystemState &sysState);
        void setModeTracking(SystemState &sysState);
        void setModeCaptive(SystemState &sysState);
        void setModeInertial(SystemState &sysState);
        void updatePosition(SystemState &sysState);
        void updateActuatorValue(SystemState &sysState);
};

#endif
