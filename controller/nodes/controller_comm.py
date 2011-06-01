import serial 
import time


class ControllerComm(serial.Serial):

    def __init__(self,dev='/dev/USB_Controller',baudrate=115200):
        super(ControllerComm, self).__init__(dev,baudrate)
        self.open()
        time.sleep(2.0)
        self.readInWaiting()

    def sendCmd(self,cmdStr):
        self.write(cmdStr)

    def sendSetPoint(self, pos, vel):
        cmdStr = '[4, %f, %f]'%(pos,vel)
        self.sendCmd(cmdStr)

    def sendPosition(self,pos):
        cmdStr = '[5, %f]'%(pos,)
        self.sendCmd(cmdStr)

    def setModeTracking(self):
        cmdStr = '[1]'
        self.sendCmd(cmdStr)

    def setModeOff(self):
        cmdStr = '[0]'
        self.sendCmd(cmdStr)

    def readInWaiting(self):
        lineList = []
        while self.inWaiting() > 0:
            line = self.readline()
            try:
                line = [float(x) for x in line.split()]
                lineList.append(line)
            except:
                pass
        return lineList


# -----------------------------------------------------------------
if __name__ == "__main__":

    comm = ControllerComm()
    comm.setModeOff()
    comm.close()

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





        



