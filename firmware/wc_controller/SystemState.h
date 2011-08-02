#ifndef SystemState_h
#define SystemState_h
#include <Servo.h>
#include "io_pins.h"

// system modes
#define SYS_MODE_OFF        0
#define SYS_MODE_MOTOR_CMD  4

#define SYS_NUM_PWM 2
#define SYS_PWM_MIN_US 1000
#define SYS_PWM_MAX_US 2000
#define SYS_PWM_START_US 1500
#define SYS_PWM_0_PIN 5
#define SYS_PWM_1_PIN 6

#define SYS_WATCHDOG_MAX 15 // 0.3 sec

class SystemState {
    public:
        int operatingMode;
        int motorCommand;
        unsigned int watchDogCnt;
        bool sendDataFlag; 
        bool dataStreamFlag;
        int pwmValue[SYS_NUM_PWM];
        int ainValue[AIN_NUM];
        Servo pwm[SYS_NUM_PWM];

        SystemState();
        void initializeIO();
        void updatePWMValue(int num, int value);
        void updateMotorCmd(int value);
        void updateWatchDog();
        void setModeOff();
        void setModeMotorCmd();
        void setDataStreamOn();
        void setDataStreamOff();
};


#endif
