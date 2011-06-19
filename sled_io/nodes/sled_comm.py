import serial 
import time

CMD_SET_MODE_OFF = 0
CMD_SET_MODE_TRACKING = 1
CMD_SET_MODE_CAPTIVE = 2
CMD_SET_MODE_INERTIAL= 3
CMD_SET_MODE_MOTOR_CMD = 4
CMD_SET_MODE_VEL_CTL = 5

CMD_UPDATE_SETPT = 54
CMD_UPDATE_POSITION = 55
CMD_UPDATE_ACTUATOR_VALUE = 56 
CMD_UPDATE_MOTOR_CMD = 57 
CMD_UPDATE_TEST_FORCE = 58

class ControllerComm(serial.Serial):

    def __init__(self,dev='/dev/USB_Controller',baudrate=115200):
        super(ControllerComm, self).__init__(dev,baudrate)
        self.open()
        time.sleep(2.0)
        self.readInWaiting()

    def sendCmd(self,cmdStr):
        self.write(cmdStr)

    def sendSetPoint(self, pos, vel):
        cmdStr = '[%d, %f, %f]'%(CMD_UPDATE_SETPT,pos,vel)
        self.sendCmd(cmdStr)

    def sendPosAndVel(self,pos,vel):
        cmdStr = '[%d, %f, %f]'%(CMD_UPDATE_POSITION,pos,vel)
        self.sendCmd(cmdStr)

    def sendMotorCmd(self,val):
        try:
            val = int(val)
        except:
            raise ValueError,'value must be convertable to integer'

        if val > 4096 or val < -4096:
            raise ValueError, 'abs(value) must be <= 4095'
        cmdStr = '[%d, %d]'%(CMD_UPDATE_MOTOR_CMD,val)
        self.sendCmd(cmdStr)

    def sendTestForce(self,val):
        cmdStr = '[%d, %f]'%(CMD_UPDATE_TEST_FORCE,val,)
        self.sendCmd(cmdStr)

    def setModeCaptive(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_CAPTIVE,)
        self.sendCmd(cmdStr)

    def setModeInertial(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_INERTIAL,)
        self.sendCmd(cmdStr)

    def setModeTracking(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_TRACKING,)
        self.sendCmd(cmdStr)

    def setModeOff(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_OFF,)
        self.sendCmd(cmdStr)

    def setModeMotorCmd(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_MOTOR_CMD,)
        self.sendCmd(cmdStr)

    def setModeVelControl(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_VEL_CTL,)
        self.sendCmd(cmdStr)

    def readInWaiting(self,conv2float=True):
        lineList = []
        while self.inWaiting() > 0:
            line = self.readline()
            if conv2float == True:
                try:
                    line = [float(x) for x in line.split()]
                    lineList.append(line)
                except:
                    pass
            else:
                lineList.append(line)
        return lineList


# -----------------------------------------------------------------
if __name__ == "__main__":

    import atexit

    def cleanup():
        print 'cleaning up ...'
        print '  setting value to 0'
        comm.setMotorCmd(0)
        print '  setting mode off'
        comm.setModeOff()
        print '  closing'
        comm.close()

    atexit.register(cleanup)



    comm = ControllerComm()
    comm.setModeMotorCmd()
    comm.sendMotorCmd(0)
    while 1:
        val = raw_input('value = ')
        val = int(val)
        comm.sendMotorCmd(val)

    #comm.sendSetPoint(0,0)
    #comm.sendPosition(0)
    #comm.setModeTracking()

    #time.sleep(0.5)
    #lines = comm.readInWaiting()
    #print lines
    #print 

    #for i in range(0,200):
    #    comm.sendSetPoint(i,0)
    #    time.sleep(0.02)

    #time.sleep(0.5)
    #lines = comm.readInWaiting()
    #print lines
    #print

    #comm.setModeOff()
    #comm.close()



