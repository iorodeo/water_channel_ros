import scipy
import run_defs 

class Run_Converter(object):

    """
    Converts runs stored in hdf5 'run files' to set point and/or actuation
    arrays for outscan to the robot.
    """

    def __init__(self,mode,dt):
        """
        Initialize run loader based on operating mode.
        """
        if mode == 'position trajectory':
            self.get = self.getPositionRun
        elif mode == 'captive trajectory':
            self.get = self.getActuatorRun
        elif mode == 'inertial trajectory':
            self.get = self.getActuatorRun
        else:
            raise ValueError, 'unknown mode %s'%(mode,)
        self.dt = dt

    def getPositionRun(self,run,startPos):
        """
        Run converter for 'position trajectory' mode
        """
        runType = run.attrs['type']
        runType = runType.lower()
        runParams = run['params']

        if runType == 'constant velocity':
            T = runParams['T'][0]
            v = runParams['velocity'][0]
            setptValues = run_defs.get_constant_velo(v,T,self.dt)
            setptValues = shift2StartPos(setptValues,startPos)
        elif runType == 'ramp profile':
            dist = runParams['distance'][0]
            velo = runParams['velocity'][0]
            accel = runParams['acceleration'][0]
            setptValues = run_defs.get_ramp(0,dist,velo,accel,self.dt)
            setptValues = shift2StartPos(setptValues,startPos)
        elif runType == 'cosine':
            amplitude = runParams['amplitude'][0]
            cycles = runParams['cycles'][0]
            period = runParams['period'][0] 
            setptValues = run_defs.get_cosine(amplitude,period,cycles,self.dt)
            setptValues = shift2StartPos(setptValues,startPos)
        elif runType == 'array':
            setptValues = scipy.array(runParams)
            setptValues = shift2StartPos(setptValues,startPos)
        else:
            raise ValueError, 'uknown run type %s for mode %s'%(runType,self.mode)

        return setptValues

    def getActuatorRun(self,run,*arg):
        """
        Run converter for 'captive trajectory' mode
        """
        runType = run.attrs['type']
        runType = runType.lower()
        runParams = run['params']

        if runType == 'constant':
            value = runParams['value'][0]
            T = runParams['T'][0]
            valueArray = run_defs.get_constant(value,T,self.dt)
        elif runType == 'trapezoidal':
            T = runParams['T'][0]
            rate = runParams['rate'][0]
            value0 = runParams['value_0'][0]
            value1 = runParams['value_1'][0]
            valueArray = run_defs.get_trapezoidal(value0,value1,rate,T,self.dt)
        elif runType ==  'array':
            valueArray = scipy.array(runParams)
        else:
            raise ValueError, 'unknown run type %s for mode %s'%(runType,self.mode)
        return valueArray


def shift2StartPos(values,startPos):
    return  values - values[0] + startPos

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    import os
    import os.path
    import pylab
    from hdf5_run_reader import HDF5_Run_Reader

    if 0:
        filename = os.path.join(os.environ['HOME'], 'test_position_run_file.hdf5') 
        reader = HDF5_Run_Reader(filename) 
        run = reader.get_run(75)
        converter = Run_Converter('position trajectory',1/50.0)
        values = converter.get(run,4)
        pylab.plot(values)
        pylab.show()

    if 1:
        filename = os.path.join(os.environ['HOME'], 'test_captive_run_file.hdf5')
        reader = HDF5_Run_Reader(filename)
        run = reader.get_run(5)
        converter = Run_Converter('captive trajectory', 1/50.0)
        values = converter.get(run)
        pylab.plot(values)
        pylab.show()


