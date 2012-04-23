from __future__ import division
import scipy

def get_constant(value,T,dt,t_output=False):
    """
    Returns constant trajectory with given value, duration and time
    step.
    """
    N = scipy.floor(T/dt)+1
    t = scipy.arange(0,N)*dt
    x = value*scipy.ones(t.shape)
    if not t_output:
        return x
    else:
        return t,x

def get_trapezoidal(value_0, value_1, rate, T, dt, t_output=False):
    """
    Returns a trapezoidal trajectory.  
    """
    N = scipy.floor(T/dt)+1
    t = scipy.arange(0,N)*dt
    x = scipy.zeros(t.shape)

    # Choose sign of rate based on whether or not value_0 is larger of smaller 
    # than value_1
    if value_0 <= value_1:
        rate = abs(rate)
    else:
        rate = -abs(rate)
    t_value_1 = (value_1 - value_0)/rate

    if t_value_1 > T/2:
        # Unable to go from value_0 to value_1 in time T at given rate - to the best 
        # that you can.
        ind = (t<=T/2)
        a = rate
        b = value_0
        x[ind] = a*t[ind] +  b
        value_end = a*T/2 + b

        ind = (t>T/2)
        a = 2*(value_0 -value_end)/T
        b = value_end - a*T/2
        x[ind] = a*t[ind] + b
    else:
        # We can go from value_0 to value_1 in the time T with the given rate.
        ind = (t<t_value_1)
        a = rate
        b = value_0
        x[ind] = a*t[ind] + b

        ind = scipy.logical_and(t>=t_value_1, t<(T-t_value_1))
        a = 0
        b = value_1
        x[ind] = a*t[ind] + b

        ind = (t>=(T-t_value_1))
        a = -rate
        b = value_0 - a*T 
        x[ind] = a*t[ind] + b 

    if not t_output:
        return x
    else:
        return t,x




def get_constant_velo(v,T,dt,t_output=False):
    """
    Returns constant velocity trajectory starting at 0 with the
    given duration, T,  and time step, dt.
    """
    N = scipy.floor(T/dt)+1
    t = scipy.arange(0,N)*dt
    x = v*t
    if not t_output:
        return x
    else:
        return t,x


def get_cosine(amplitude,period,cycles,dt,t_output=False):
    """
    Returns a cosine with the given amplitude, period, number of cycles,
    and time step.
    """
    T = period*cycles
    N = scipy.floor(T/dt)+1
    t = scipy.arange(0,N)*dt
    x = amplitude*scipy.cos(2*scipy.pi*t/period)
    if not t_output:
        return x
    else:
        return t, x

def get_ramp(x0,x1,vmax,a,dt, output='ramp only'):
    """
    Generates a ramp trajectory from x0 to x1 with constant acceleration, a, to
    maximum velocity v_max. 

    Note, the main purlpose of this routine is to generate a trajectory from x0
    to x1. For this reason v_max and a are adjusted slightly to work with the
    given time step.

    Arguments:
     x0 = starting position
     x1 = ending position
     vmax = maximum allowed velocity
     a = constant acceleration
     
     Keywords:
       output = 'ramp only' or 'full'
       when ramp only is selected then only the velocity ramp is returned. 
       If 'full' is selected the adjusted acceleration and maximum velocity 
       are also returned.
       
    Ouput:
      ramp = ramp trajectory form x0 to x1
    """
    # Insure we are dealing with floating point numbers
    x0, x1 = float(x0), float(x1)
    vmax, a = float(vmax), float(a)
    dt = float(dt)
    vmax, a = abs(vmax), abs(a) # Make sure that v_max and a are positive

    # Check to see if there is anything to do
    if x0==x1:
        return scipy.array([x0])

    # Get distance and sign indicating direction
    dist = abs(x1-x0)
    sign = scipy.sign(x1-x0)

    # Determine if we will reach v_max
    t2vmax = vmax/a
    t2halfdist = scipy.sqrt(0.5*dist/a)
    
    if t2vmax > t2halfdist:
        # Trajectory w/o constant velocity segment  
        T = scipy.sqrt(dist/a)
        n = int(scipy.round_((1.0/dt)*T))
         
        # Adjust accel and duration for rounding of n (discrete time steps)
        a = dist/(n*dt)**2
        T = scipy.sqrt(dist/a)
        
        # Generate trajectory
        t = scipy.linspace(0.0,2.0*T,2*n+1)
        def f1(t):
            return 0.5*sign*a*(t**2)
        def f2(t):
            s = t-T
            return f1(T)+ sign*a*T*s - 0.5*sign*a*s**2
        func_list = [f1,f2]
        cond_list = [t<=T, t>T]
        ramp = x0+scipy.piecewise(t,cond_list,func_list)
          
    else:
        # Trajectory w/ constant velocity segment
        # Compute acceleration time and adjust acceleration
        T1 = vmax/a 
        n = int(scipy.round_(T1/dt))
        a = vmax/(n*dt) # Adjusted acceleration 
        T1 = vmax/a # Adjusted acceleration time  

        # Compute and adjust constant velocity time
        T2 = dist/vmax - T1  
        m = int(scipy.round_(T2/dt))
        vmax = dist/(dt*(n+m)) # Adjusted max velocity  
        T2 = dist/vmax - T1 # Adjusted constant velocity time

        # Generate trajectory
        t = scipy.linspace(0.0,2.0*T1+T2,2*n+m+1)
        def f1(t):
            return 0.5*sign*a*(t**2)
        def f2(t):
            s = t-T1
            return f1(T1) + sign*vmax*s
        def f3(t):
            s = t-T1-T2
            return f2(T1+T2)+sign*vmax*s-0.5*sign*a*s**2 
        func_list = [f1,f2,f3]
        cond_list = [t<=T1, scipy.logical_and(t>T1,t<=T1+T2), t>T1+T2]
        ramp = x0+scipy.piecewise(t,cond_list,func_list)

    if output=='ramp only':
        return ramp
    elif output=='full':
        return ramp, vmax, a
    else:
        raise ValueError, 'unknown keyword option output=%s'%(output,)

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import pylab

    if 0:
        value = 10.0
        T = 5.0
        dt = 1/50.0
        t,x = get_constant(value,T,dt,t_output=True)
        pylab.plot(t,x)
        pylab.show()

    if 1:
        value_0 = 0.0
        value_1 = 20.0
        rate = 5.0
        T = 5.0
        dt = 1/50.0
        t,x = get_trapezoidal(value_0, value_1, rate, T, dt, t_output=True)
        pylab.plot(t,x)
        pylab.show()

    if 0:
        """
        Debugging constant velocity move.
        """
        v = 5.0
        T = 10.0
        dt = 1/50.0
        t,x = get_constant_velo(v,T,dt,t_output=True)
        pylab.plot(t,x)
        pylab.show()

    if 0:
        """
        Debugging cosine move
        """
        amplitude = 2.0
        period = 2.0
        cycles = 2
        dt = 1.0/50.0
        t,x = get_cosine(amplitude,period,cycles,dt,t_output=True)
        print dt, t[1]-t[0]
        pylab.plot(t,x,'.')
        #pylab.plot(t+cycles*period,x,'.r')
        pylab.show()

    if 0:
        """
        Debugging ramp move
        """
        x0 = 0.0
        x1 = 1000.0
        vmax = 1000.0
        a = 50.0
        dt = 1.0/50.0
        ramp = get_ramp(x0,x1,vmax,a,dt)
        pylab.plot(ramp)
        pylab.show()
