#ifndef PIDController_h
#define PIDController_h

#define PID_DFLT_OUTPUT_MAX  2000.0
#define PID_DFLT_OUTPUT_MIN -2000.0

class PIDController {
    public:
        PIDController();
        void setGains(float pGain, float iGain, float dGain, float ffGain);
        void setPGain(float pGain);
        void setIGain(float iGain);
        void setDGain(float dGain);
        void setFFGain(float FFGain);
        float update(float error, float feedForward);
        void setOutputLimits(float minValue, float maxValue);
        void reset();
    private:
        float pGain;   // Proportional gain
        float iGain;   // Integral gain
        float dGain;   // Derivative gain
        float ffGain;  // Feed-forward gain
        float lastError;
        float integralTerm;
        float outputMax;
        float outputMin;
        bool isFirstUpdate;
        float clamp(float value);
};


#endif
