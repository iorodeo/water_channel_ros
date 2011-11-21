import scipy

def get_cosine(amplitude,period,cycles,dt,t_output=False):
    """
    Returns a cosine with the given amplitude, period, number of cycles,
    and time step.
    """
    T = period*cycles
    N = scipy.floor(T/dt)
    t = scipy.arange(0,N)*dt
    x = amplitude*scipy.cos(2*scipy.pi*t/period)
    if not t_output:
        return x
    else:
        return t, x

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import pylab
    amplitude = 2.0
    period = 2.0
    cycles = 2
    dt = 1.0/50.0
    t,x = get_cosine(amplitude,period,cycles,dt,t_output=True)
    print dt, t[1]-t[0]
    pylab.plot(t,x,'.')
    #pylab.plot(t+cycles*period,x,'.r')
    pylab.show()


