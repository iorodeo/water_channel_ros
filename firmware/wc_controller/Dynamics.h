#ifndef _DYNAMICS_H_
#define _DYNAMICS_H_

class Dynamics {
private:
    float mass;
    float dt;
    float vel;
    float damping;
public:
    Dynamics();
    void setMass(float value);
    void setDt(float value);
    void setVelocity(float value);
    void setDamping(float value);
    void update(float force);
    float getVelocity();
    float getMass();
    float getDt();
    float getDamping();
};


#endif
