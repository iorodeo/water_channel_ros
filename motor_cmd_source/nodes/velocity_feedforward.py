class VelocityFeedForward(object):

    def __init__(self,coeff=0.0):
        self.coeff = coeff

    def func(self,ffValue):
        return self.coeff*ffValue
