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

#define SYS_DFLT_PGAIN_TRACKING    12.0
#define SYS_DFLT_IGAIN_TRACKING    0.03
#define SYS_DFLT_DGAIN_TRACKING    0.0
#define SYS_DFLT_FFGAIN_TRACKING   0.0 

#define SYS_DFLT_PGAIN_VEL_CTL  0.5
#define SYS_DFLT_IGAIN_VEL_CTL  0.001
#define SYS_DFLT_DGAIN_VEL_CTL  0.0
#define SYS_DFLT_FFGAIN_VEL_CTL 0.0

#define SYS_DFLT_DYNAMICS_MASS    10.0
#define SYS_DFLT_DYNAMICS_DAMPING  0.0

#define SYS_POS_ERROR_STARTUP_LIMIT 200
#define SYS_VEL_ERROR_STARTUP_LIMIT 200

class SystemState {
    public:
        SystemState();
        int operatingMode;
        float setPosition;
        float setVelocity; 
        float position; 
        float velocity;
        float positionError; 
        float velocityError;
        float motorCommand;
        float force;
        PIDController controller;
        Dynamics dynamics;
        float actuatorValue;
        bool sendDataFlag; 

        // Update value methods
        void updateSetPoint(float pos, float vel);
        void updatePosAndVel(float pos, float vel);
        void updateActuatorValue(int value);
        void updateError();
        void updateTestForce(float value);

        // Set mode methods 
        void setModeOff();
        void setModeTracking();
        void setModeCaptive();
        void setModeInertial();
        void setModeMotorCmd();
        void setModeVelControl();
        void updateMotorCmd(int value);

        // Controller gain setting methods
        void setGainsZero();
        void setGainsTracking();
        void setGainsVelControl();
};


#endif
