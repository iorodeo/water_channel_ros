from __future__ import division

class Dynamics(object):
    """
    Simple first order dynmaics - currently eulers method
    """

    def __init__(self,mass=1.0,damping=0.0,pos_init=0.0,vel_init=0.0):
        self.mass = mass
        self.damping = damping
        self.position = pos_init 
        self.velocity = vel_init 

    def update(self,force,dt):
        self.velocity += (dt/self.mass)*(force - self.damping*self.velocity)
        self.position += dt*self.velocity

    def reset(self):
        self.position = 0.0
        self.velocity = 0.0
