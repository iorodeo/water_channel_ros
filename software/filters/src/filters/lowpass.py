import math

class Lowpass(object):

    def __init__(self, y0=0.0,fcut=25.0):
        self.y = y0
        self.fcut = fcut

    def update(self,x,dt):
        tc = 1.0/(2.0*math.pi*self.fcut)
        alpha = dt/(tc + dt)
        self.y = alpha*x + (1.0-alpha)*self.y
        return self.y

    def getValue(self):
        return self.y


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    import scipy
    import matplotlib.pylab as pylab

    filt = Lowpass()
    f = 75.0

    t = scipy.linspace(0,2.0/f,500)
    dt = t[1] - t[0]
    x = scipy.sin(2.0*scipy.pi*f*t)

    y = []
    for xval in x:
        yval = filt.update(xval,dt)
        y.append(yval)

    y = scipy.array(y)

    pylab.plot(t,x,'b')
    pylab.plot(t,y,'r')
    pylab.show()
        











