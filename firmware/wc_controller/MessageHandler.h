#ifndef MessageHandler_h
#define MessageHandler_h
#include "SerialReceiver.h"
#include "SystemState.h"


#define CMD_SET_MODE_OFF       0
#define CMD_SET_MODE_TRACKING  1
#define CMD_SET_MODE_CAPTIVE   2
#define CMD_SET_MODE_INERTIAL  3
#define CMD_SET_MODE_MOTOR_CMD 4 

#define CMD_UPDATE_SETPT             54
#define CMD_UPDATE_POSITION          55
#define CMD_UPDATE_ACTUATOR_VALUE    56 
#define CMD_UPDATE_MOTOR_CMD         57 
#define CMD_UPDATE_TEST_FORCE        58

class MessageHandler: public SerialReceiver {
    public:
        void switchYard(SystemState &sysState);
    private:
        void updateSetPoint(SystemState &sysState);
        void setModeOff(SystemState &sysState);
        void setModeTracking(SystemState &sysState);
        void setModeCaptive(SystemState &sysState);
        void setModeInertial(SystemState &sysState);
        void setModeMotorCmd(SystemState &sysState);
        void updatePosAndVel(SystemState &sysState);
        void updateActuatorValue(SystemState &sysState);
        void updateMotorCmd(SystemState &sysState);
        void updateTestForce(SystemState &sysState);
};

#endif
