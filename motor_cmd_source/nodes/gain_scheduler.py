import math

class GainScheduler(object):
    """
    Two parameter gain scheduling function which applies a minimum gain
    for values near (x,y) = (0,0) and transition to a maximum gain for over 
    the transistion widths.
    """

    def __init__(self, width_x=1.0, width_y=1.0,min_gain=1.0, max_gain=2.0):
        self.width_x = width_x
        self.width_y = width_y
        self.min_gain = min_gain
        self.max_gain = max_gain

    def gain(self,x,y):
        theta = math.atan2(y,x)
        return self.x_func(x)*math.cos(theta)**2 + self.y_func(y)*math.sin(theta)**2

    def x_func(self,x):
        slope = (self.max_gain - self.min_gain)/self.width_x
        offset = self.min_gain
        value = slope*abs(x) + offset
        if value > self.max_gain:
            value = self.max_gain
        return value

    def y_func(self,y):
        slope = (self.max_gain - self.min_gain)/self.width_y
        offset = self.min_gain
        value = slope*abs(y) + offset
        if value > self.max_gain:
            value = self.max_gain
        return value


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import matplotlib.pylab as pylab

    sch = GainScheduler(width_x=1.0,width_y=2.0)

    x_vals = pylab.linspace(-5,5,1000)
    gain_vals = []

    for x in x_vals:
        gain = sch.gain(0,x)
        gain_vals.append(gain)

    pylab.plot(x_vals,gain_vals,'.')
    pylab.show()


