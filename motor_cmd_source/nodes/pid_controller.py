class PIDController(object):

    def __init__(self,pgain=0.0,igain=0.0,dgain=0.0,outputMin=-4000,outputMax=4000,ffFunc=None):
        self.pgain = pgain
        self.igain = igain
        self.dgain = dgain
        self.ffFunc = ffFunc
        self.outputMax = outputMax 
        self.outputMin = outputMin 
        self.ffTerm = 0.0
        self.pTerm = 0.0
        self.iTerm = 0.0
        self.dTerm = 0.0
        self.lastError = None

    def update(self,error,ffValue):
        # Compute feedforward term
        if self.ffFunc is None:
            self.ffTerm = 0.0
        else:
            self.ffTerm = self.ffFunc(ffValue)

        # Compute proportional term
        self.pTerm = self.pgain*error

        # Compute derivative term
        # 
        # Note, really should have a dt in here to do the dTerm 
        # properly. 
        if self.lastError is None: 
            self.dTerm = 0.0
        else:
            self.dTerm = self.dgain*(error - self.lastError)
        self.lastError = error

        # Compute integral term
        self.iTerm += self.igain*error
        self.iTerm = self.clamp(self.iTerm)

        # Compute output and clamp
        output = self.ffTerm + self.pTerm + self.dTerm + self.iTerm
        output = self.clamp(output)
        return output

    def clamp(self,value): 
        if value > self.outputMax:
           output = self.outputMax
        elif value < self.outputMin:
           output = self.outputMin
        else:
           output = value
        return output

        
