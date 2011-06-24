import serial 
import time

CMD_SET_MODE_OFF = 0
CMD_SET_MODE_MOTOR_CMD = 4

CMD_UPDATE_ACTUATOR_VALUE = 56 
CMD_UPDATE_MOTOR_CMD = 57 

class SledIOComm(serial.Serial):

    def __init__(self,dev='/dev/USB_Controller',baudrate=115200):
        super(SledIOComm, self).__init__(dev,baudrate)
        self.open()
        time.sleep(2.0)
        self.readInWaiting()

    def sendCmd(self,cmdStr):
        self.write(cmdStr)

    def sendMotorCmd(self,val):
        val = int(val)
        cmdStr = '[%d, %d]'%(CMD_UPDATE_MOTOR_CMD,val)
        self.sendCmd(cmdStr)

    def setModeInertial(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_INERTIAL,)
        self.sendCmd(cmdStr)

    def setModeOff(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_OFF,)
        self.sendCmd(cmdStr)

    def setModeMotorCmd(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_MOTOR_CMD,)
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

    comm = SledIOComm()
    comm.setModeMotorCmd()
    comm.sendMotorCmd(0)
    while 1:
        val = raw_input('value = ')
        val = int(val)
        comm.sendMotorCmd(val)



