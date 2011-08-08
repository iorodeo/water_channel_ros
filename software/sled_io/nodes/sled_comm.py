import serial 
import time

CMD_SET_MODE_OFF = 0
CMD_SET_MODE_MOTOR_CMD = 4
CMD_SET_DATA_STREAM_ON = 5
CMD_SET_DATA_STREAM_OFF = 6

CMD_UPDATE_PWM_VALUE = 56 
CMD_UPDATE_MOTOR_CMD = 57 
CMD_UPDATE_WATCHDOG = 60

class SledIOComm(serial.Serial):

    def __init__(self,**kwargs): 
        super(SledIOComm, self).__init__(**kwargs)
        time.sleep(2.5)
        self.flushOutput()
        self.flushInput()

    def sendCmd(self,cmdStr):
        self.write(cmdStr)

    def sendMotorCmd(self,val):
        val = int(val)
        cmdStr = '[%d, %d]'%(CMD_UPDATE_MOTOR_CMD,val)
        self.sendCmd(cmdStr)

    def sendActuatorPWM(self,pwmNum,val):
        pwmNum = int(pwmNum)
        val = int(val)
        cmdStr = '[%d, %d, %d]'%(CMD_UPDATE_PWM_VALUE,pwmNum, val)
        self.sendCmd(cmdStr)

    def sendWatchDogPulse(self):
        cmdStr = '[%d]'%(CMD_UPDATE_WATCHDOG,)
        self.sendCmd(cmdStr)

    def setModeOff(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_OFF,)
        self.sendCmd(cmdStr)

    def setModeMotorCmd(self):
        cmdStr = '[%d]'%(CMD_SET_MODE_MOTOR_CMD,)
        self.sendCmd(cmdStr)

    def setDataStreamOn(self):
        cmdStr = '[%d]'%(CMD_SET_DATA_STREAM_ON,)
        self.sendCmd(cmdStr)

    def setDataStreamOff(self):
        cmdStr = '[%d]'%(CMD_SET_DATA_STREAM_OFF,)
        self.sendCmd(cmdStr)

    def flushBuffer(self):
        while self.inWaiting > 0:
            line = self.readline()

    def readInWaiting(self): 
        lineList = []
        print 'inWaiting', self.inWaiting()
        while self.inWaiting() > 0:
            try:
                line = self.readline()
                print 'line', line
            except:
                # May want to do better exception handling here
                print 'readline error'
                continue
            line = line.strip()
            try:
                line = eval(line)
                lineList.append(line)
            except:
                pass
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

    comm = SledIOComm(port='/dev/USB_Controller', baudrate=152000, timeout=0.5)
    comm.setModeMotorCmd()
    comm.sendMotorCmd(0)
    while 1:
        val = raw_input('value = ')
        val = int(val)
        comm.sendMotorCmd(val)



