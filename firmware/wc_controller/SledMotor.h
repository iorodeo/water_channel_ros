#ifndef SledMotor_h
#define SledMotor_h

#include "io_pins.h"
#include "mcp4822.h"

#define SM_MOTOR_CMD_CLAMP 4095
#define SM_DRIVE_DEADBAND  120 

class SledMotor {
    public:
        MCP4822 analogOut;
        void initializeIO();
        void setVelocity(int vel);
        void setVelocity(float vel);
        void off();
    private:
        void setDirection(int value);
        void setSpeed(int value);
        int clamp(int value);
        float clamp(float value);
        int deadBandComp(int value);
};

#endif
