import math

class GainScheduler(object):

    def __init__(self, min_gain, max_gain, velo_width):

        self.min_gain = min_gain
        self.max_gain = max_gain
        self.velo_width = velo_width

    def gain(self,velo):
        if abs(velo) <= self.velo_width:
            gain = self.min_gain 
            gain += abs(velo/self.velo_width)*(self.max_gain - self.min_gain)
        else:
            gain = self.max_gain
        return gain

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import matplotlib.pylab as pylab

    sch = GainScheduler(100, 1000, 0.1)
    velo = pylab.linspace(-0.2,0.2,100)
    gain = []
    for v in velo:
        gain.append(sch.gain(v))
    pylab.plot(velo,gain)
    pylab.xlabel('velocity (m/s)')
    pylab.ylabel('gain')
    pylab.grid('on')
    pylab.show()




