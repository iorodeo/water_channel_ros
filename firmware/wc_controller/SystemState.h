#ifndef SystemState_h
#define SystemState_h
#include <Servo.h>
#include "PIDController.h"
#include "Dynamics.h"

// system modes
#define SYS_MODE_OFF        0
#define SYS_MODE_MOTOR_CMD  4

#define SYS_NUM_PWM 2
#define SYS_PWM_MIN_US 1000
#define SYS_PWM_MAX_US 2000
#define SYS_PWM_START_US 1500
#define SYS_PWM_0_PIN 5
#define SYS_PWM_1_PIN 6

class SystemState {
    public:
        SystemState();
        int operatingMode;
        int motorCommand;
        Servo pwm[SYS_NUM_PWM];
        int pwmValue[SYS_NUM_PWM];
        bool sendDataFlag; 
        PIDController controller;
        Dynamics dynamics;

        void updatePWMValue(int num, int value);
        void updateMotorCmd(int value);

        // Set mode methods 
        void setModeOff();
        void setModeMotorCmd();
};


#endif
