#ifndef SystemState_h
#define SystemState_h
#include "PIDController.h"

#define SYS_MODE_OFF      0
#define SYS_MODE_TRACKING 1
#define SYS_MODE_CAPTIVE  2
#define SYS_MODE_INERTIAL 3
#define SYS_MODE_MOTOR_CMD 4

#define SYS_DFLT_PGAIN_TRACKING 15.0
#define SYS_DFLT_IGAIN_TRACKING 0.0
#define SYS_DFLT_DGAIN_TRACKING 0.0
#define SYS_DFLT_FFGAIN_TRACKING 0.0 

#define SYS_POSERROR_STARTUP_LIMIT 200

class SystemState {
    public:
        SystemState();
        int operatingMode;
        float setPosition;
        float setVelocity; 
        float position; 
        float positionError; 
        float motorCommand;
        PIDController controller;
        float actuatorValue;
        bool sendDataFlag; 
        void updateSetPoint(float pos, float vel);
        void updatePosition(float pos);
        void updateActuatorValue(int value);
        void updatePositionError();
        void setModeOff();
        void setModeTracking();
        void setModeCaptive();
        void setModeInertial();
        void setModeMotorCmd();
        void updateMotorCmd(int value);
};


#endif
