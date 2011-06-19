class VelocityFeedForward(object):

    def __init__(self,posOffset=0.0, negOffset=0.0, posCoeff=0.0, negCoeff=0.0):
        self.posOffset = posOffset 
        self.negOffset = negOffset
        self.posCoeff = posCoeff 
        self.negCoeff = negCoeff 

    def func(self,ffValue):
        if ffValue > 0:
            rval = self.posCoeff*ffValue + self.posOffset
        elif ffValue < 0:
            rval = self.negCoeff*ffValue + self.negOffset
        else:
            rval = 0.0
        return rval
