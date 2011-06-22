#ifndef SystemState_h
#define SystemState_h
#include "PIDController.h"
#include "Dynamics.h"

#define SYS_MODE_OFF        0
#define SYS_MODE_TRACKING   1
#define SYS_MODE_CAPTIVE    2
#define SYS_MODE_INERTIAL   3
#define SYS_MODE_MOTOR_CMD  4
#define SYS_MODE_VEL_CTL    5


class SystemState {
    public:
        SystemState();
        int operatingMode;
        int motorCommand;
        int actuatorValue;
        bool sendDataFlag; 
        PIDController controller;
        Dynamics dynamics;

        void updateActuatorValue(int value);
        void updateMotorCmd(int value);

        // Set mode methods 
        void setModeOff();
        void setModeMotorCmd();
};


#endif
