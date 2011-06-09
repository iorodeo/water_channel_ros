#include "WProgram.h"
#include "SPI.h"
#include "Dynamics.h"

#define DFLT_MASS    1.0
#define DFLT_DT      1.0
#define DFLT_VEL     0.0
#define DFLT_DAMPING 0.0

Dynamics::Dynamics() {
    mass = DFLT_MASS;
    dt = DFLT_DT;
    vel = DFLT_VEL;
    damping = DFLT_DAMPING;
}

void Dynamics::update(float force) {
    vel = vel + (dt/mass)*(force - damping*vel);
}

void Dynamics::setMass(float value) {
    mass = fabs(value);
}

void Dynamics::setDt(float value) {
    dt = fabs(value);
}

void Dynamics::setVelocity(float value) {
    vel = value;
}

void Dynamics::setDamping(float value) {
    damping = value;
}

float Dynamics::getVelocity() {
    return vel;
}

float Dynamics::getMass() {
    return mass;
}

float Dynamics::getDt() {
    return dt;
}

float Dynamics::getDamping() {
    return damping;
}
