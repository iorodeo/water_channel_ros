#include "PIDController.h"

PIDController::PIDController() {
    pGain = 0.0;
    dGain = 0.0;
    iGain = 0.0;
    ffGain = 0.0;
    lastError = 0.0;
    integralTerm = 0.0;
    isFirstUpdate = true;
    outputMax = PID_DFLT_OUTPUT_MAX;
    outputMin = PID_DFLT_OUTPUT_MIN;
}

float PIDController::update(float error, float feedForward) {
    float output;
    float proportionalTerm;
    float derivativeTerm;
    float feedForwardTerm;
    if (isFirstUpdate == true) {
        lastError = error;
        isFirstUpdate = false;
    }
    proportionalTerm = pGain*error;
    integralTerm += iGain*error;
    integralTerm = clamp(integralTerm);
    derivativeTerm = dGain*(error - lastError);
    feedForwardTerm = ffGain*feedForward;
    output = proportionalTerm + integralTerm + derivativeTerm + feedForwardTerm;
    output = clamp(output);
    lastError = error;
    return output;
}

void PIDController::reset() {
    lastError = 0.0;
    integralTerm = 0.0;
    isFirstUpdate = true;
}

void PIDController::setGains(float _pGain, float _iGain, float _dGain, float _ffGain) {
    pGain = _pGain;
    iGain = _iGain;
    dGain = _dGain;
    ffGain = _ffGain;
}

void PIDController::setPGain(float _pGain) {
    pGain = _pGain;
}

void PIDController::setIGain(float _iGain) {
    iGain = _iGain;
}

void PIDController::setDGain(float _dGain) {
    dGain = _dGain;
}

void PIDController::setFFGain(float _ffGain) {
    ffGain = _ffGain;
}

void PIDController::setOutputLimits(float minValue, float maxValue) {
    if (minValue > maxValue) return;
    outputMin = minValue;
    outputMax = maxValue;
}

float PIDController::clamp(float value) {
    float clampedValue;
    if (value > outputMax) {
        clampedValue = outputMax;
    }
    else if (value < outputMin) {
        clampedValue = outputMin;
    }
    else {
        clampedValue = value;
    }
    return clampedValue;
}

        

