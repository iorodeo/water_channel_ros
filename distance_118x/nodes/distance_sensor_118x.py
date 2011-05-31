"""
distance_sensor_118X.py

Provides and interface to the micro-epsilon optoNCDT ILR 118X laser distance
sensors.

Usage Examples:

    sensor = DistanceSensor('/dev/ttyUSB0', baudrate=9600)
    sensor.open()

    value = sensor.getDistance() # Single shot measurement

    # Stream data from sensor
    sensor.startTracking('50hz')
    for i in range(0,num_samples):
        value = sensor.readSample(convert='float')

    sensor.close()

Author: Will Dickson, IO Rodeo Inc.
"""
import serial
import time

DFLT_SERIAL_BAUDRATE = 9600 
DFLT_SERIAL_TIMEOUT  = 2
SERIAL_SLEEP_T = 0.03
ERROR_MODE0 = 0
ERROR_MODE1 = 1 
ERROR_MODE2 = 2

TRACKING_MODE_DICT = {
        'normal'      : 'DT',
        'close_range' : 'DS',
        '10hz'        : 'DW',
        '50hz'        : 'DX',
        }
ALLOWED_BAUDRATES = (2400,4800,9600,19200,38400)

AUTOSTART_CMDS = {
        'tracking_normal'      : 'DT',
        'tracking_close_range' : 'DS',
        'tracking_10hz'        : 'DW',
        'tracking_50hz'        : 'DX',
        'distance_ext_trig'    : 'DF',
        'internal_temperature' : 'TP',
        'laser_on'             : 'LO',
        'help'                 : 'ID',
        }


class DistanceSensor(serial.Serial):

    def __init__(self, port, baudrate=DFLT_SERIAL_BAUDRATE,timeout=DFLT_SERIAL_TIMEOUT):
        serial.Serial.__init__(self,port,baudrate=baudrate,timeout=timeout)
        self.bytesize = serial.EIGHTBITS 
        self.parity = serial.PARITY_NONE
        self.stopbits = serial.STOPBITS_ONE 

    def open(self):
        """
        Open connection to sensor.
        """
        serial.Serial.open(self)
        time.sleep(SERIAL_SLEEP_T)
        while self.inWaiting() > 0:
            line = self.readline()

    def laserOn(self):
        """
        Turn on laser
        """
        self.write('LO\n')
        line = self.readline()

    def laserOff(self):
        """
        Turn off laser
        """
        self.write('LF\n')
        line = self.readline()
        
    def startDistTracking(self, mode):
        """
        Start distance tracking
        """
        mode_cmd = TRACKING_MODE_DICT[mode]
        self.write('{0}\n'.format(mode_cmd))

    def readSample(self,convert=None):
        """
        Reads distance sample from sensor - for use in distance tracking mode.

        Returns None is laser failed to get reading.
        """
        value = self.__readAndConvert(convert)
        return value

    def getDistance(self,convert=None):
        """
        Performs a one shot distance measurement.
        """
        self.write('DM\n')
        value = self.__readAndConvert(convert)
        return value

    def getInternalTemp(self):
        """
        Gets the internal temperature of the laser distance sensor
        """
        self.write('TP\n')
        value = self.__readFloat()
        return value

    def getAveraging(self):
        """
        Get distance sensor averaging - running average of 1 to 20 values.
        """
        self.write('SA\n')
        value = self.__readValue()
        value = int(value[2:])
        return value

    def setAveraging(self,val):
        """
        Set distance sensor averaging - a running average of 1 to 20 values.
        """
        assert (val >=1) and (val <=20), 'averaging value must be bewteen 1 and 20'
        cmd = 'SA{0}\n'.format(int(val))
        self.write(cmd)
        line = self.readline()

    def getDisplayFormat(self):
        """
        Returns the current display format for the sensor - decimal or hexidecimal
        """
        self.write('SD\n')
        value = self.__readValue()
        value = value[2:]
        if value == 'd':
            return 'decimal'
        else:
            return 'hexidecimal'

    def setDisplayFormat(self,val):
        """
        Sets the current display format for the sensor - allowed values decimal or 
        hexidecimal.
        """
        _val = val.lower()
        if _val == 'decimal':
            cmd = 'SDd\n'
        elif _val == 'hexidecimal':
            cmd = 'SDh\n'
        else:
            raise ValueError, 'display format must be decimal or hexidecimal'
        self.write(cmd)
        line = self.readline()

    def getMeasurementTime(self):
        """
        Returns the current measurement time setting  - value from 0 to 25. Effects
        the DT, DS, DF and DM modes. 
        """
        self.write('ST\n')
        value = self.__readValue()
        value = int(value[2:])
        return value

    def setMeasurementTime(self,val):
        """
        Set the current measurement time - a value from 0 to 25. Effects the DT, DS, DF, 
        and DM modes. 

        For DT measurement mode actual measurement time = 240 x measurement time setting (ms)
        For DS measurement mode actual measurement time = 150 x measurement time setting (ms)
        """
        assert (val >= 0) and (val <= 25), 'measurement time settin must be between 0 and 25'
        cmd = 'ST{0}\n'.format(val)
        self.write(cmd)
        value = self.readline()

    def getScaleFactor(self):
        """
        Get the current scale factor setting.
        """
        self.write('SF\n')
        value = self.__readValue()
        value = value[2:]
        value = float(value)
        return value

    def setScaleFactor(self,val):
        """
        Set the current scale factor
        """
        _val = float(val)
        cmd = 'SF{0}\n'.format(val)
        self.write(cmd)
        line = self.readline()
        
    def getErrorMode(self):
        """
        Get the error mode
        """
        self.write('SE\n')
        value = self.__readValue() 
        value = int(value[2:])
        return value

    def setErrorMode(self,val):
        """
        Set the Error mode
        """
        assert val in (ERROR_MODE0,ERROR_MODE1,ERROR_MODE2), 'unknown error mode setting'
        cmd = 'SE{0}\n'.format(val)
        self.write(cmd)
        line = self.readline()

    def getAlarmCenter(self):
        """
        Get the alarm center  - the beginng of the distance range for which the switching will
        be turned active.
        """
        self.write('AC\n')
        value = self.__readValue()
        value = int(value[2:])
        return value

    def setAlarmCenter(self,val):
        """
        Set the alarm center - the beginng of the distance range for which the switching will
        be turned active.
        """
        _val = float(val)
        cmd = 'AC{0}\n'.format(_val)
        self.write(cmd)
        line = self.readline()

    def getAlarmHysteresis(self):
        """
        Get the alarm hysteresis for beginning and end of  switchng range 
        """
        self.write('AH\n')
        value = self.__readValue()
        value = float(value[2:])
        return value

    def setAlarmHysteresis(self,val):
        """
        Set the alarm hysteresis for beginning and end of switchng range 
        """
        _val = float(val)
        cmd = 'AH{0}\n'.format(_val)
        self.write(cmd)
        value = self.__readValue()
        value = float(value[2:])
        return value

    def getAlarmWidth(self):
        """
        Get the alarm width - the length of the active range beginning at the alarm center
        """
        self.write('AW\n')
        value = self.__readValue()
        value = float(value[2:])
        return value

    def setAlarmWidth(self,val):
        """
        Set the alarm width - the length of the active range beginning at the alarm center
        """
        _val = float(val)
        cmd = 'AW{0}\n'.format(_val)
        self.write(cmd)
        line = self.readline()

    def getIoutBeginDist(self):
        """
        Get the distance where Iout is equal to 4mA
        """
        self.write('RB\n')
        value = self.__readValue()
        value = float(value[2:])
        return value

    def setIoutBeginDist(self,val):
        """
        Set the distance where Iout is equal to 4mA
        """
        _val = float(val)
        cmd = 'RB{0}\n'.format(_val)
        self.write(cmd)
        line = self.readline()

    def getIoutEndDist(self):
        """
        Get the distance where Iout is equal to 20mA.
        """
        self.write('RE\n')
        value = self.__readValue()
        value = float(value[2:])
        return value

    def setIoutEndDist(self,val):
        """
        Set the distance where Iout is equal to 20mA.
        """
        _val = float(val)
        cmd = 'RE{0}\n'.format(_val)
        self.write(cmd)
        line = self.readline()

    def getRemoveMeasurement(self):
        """
        Get the remove measurement settings which set expected range values
        """
        self.write('RM\n')
        line = self.readline()
        line = line.split()
        valx = float(line[0][2:])
        valy = float(line[1])
        valz = float(line[2])
        return valx, valy, valz 

    def setRemoveMeasurement(self,valx,valy,valz):
        """
        Set the remove measurement settings which set expected range values
        """
        _valx = float(valx)
        _valy = float(valy)
        _valz = float(valz)
        cmd = 'RM{0} {1} {2}\n'.format(_valx, _valy, _valz)
        self.write(cmd)
        line = self.readline()

    def getRemoteTrig(self):
        """
        Get the setting for the remote trigger input
        Returns delay (ms) and level (0, 1)
        """
        self.write('TD\n')
        line = self.readline()
        line = line.split()
        delay = int(line[0][2:])
        level = int(line[1])
        return delay, level

    def setRemoteTrig(self,delay,flange):
        """
        Set the delay and level settings for the remote trigger input.
        """
        assert (delay >= 0) and (delay <= 10000), 'delay must be between 0 and 10000'
        assert flange in (0,1), 'flange must be either 0 or 1'
        cmd = 'TD{0} {1}\n'.format(delay,flange)
        self.write(cmd)
        line  = self.readline()

    def getAutoStartTrig(self):
        """
        Get the auto trigger settings
        """
        self.write('TM\n')
        line = self.readline()
        line = line.split()
        state = int(line[0][2:])
        level = int(line[1])
        return state, level

    def setAutoStartTrig(self,state, level):
        """
        Set the auto trigger state and level
        """
        assert state in (0,1), 'state must be either 0 or 1'
        assert level in (0,1), 'level must be either 0 or 1'
        cmd = 'TM{0} {1}\n'.format(state,level)
        self.write(cmd)
        line = self.readline()

    def getBaudrate(self):
        """
        Get the baudrate setting
        """
        self.write('BR\n')
        value = self.__readValue()
        value = int(value[2:])
        return value
        
    def setBaudrate(self,val):
        """
        Set the baudrate
        """
        assert val in ALLOWED_BAUDRATES, 'baudrate not allowed'
        cmd = 'BR{0}\n'.format(val)
        self.write(cmd)
        time.sleep(SERIAL_SLEEP_T)
        self.close()
        self.baudrate = val
        self.open()
        line = self.readline()
        
    def getAllowedBaudrates(self):
        """
        Returns the allowed baudrate settings
        """
        return ALLOWED_BAUDRATES

    def getAutoStartCmd(self):
        """
        Get the autostart command which defines which function is available when power
        is applied to the sensor.
        """
        self.write('AS\n')
        value = self.__readValue()
        value = value[2:]
        return value

    def setAutoStartCmd(self,val):
        """
        Set the auto start command. Possible values:
          'tracking_normal'      
          'tracking_close_range' 
          'tracking_10hz'        
          'tracking_50hz'        
          'distance_ext_trig'    
          'internal_temperature' 
          'laser_on'             
          'help'                 
        """
        assert val in AUTOSTART_CMDS.keys(), 'invalid autostart command'
        cmd = 'AS{0}\n'.format(AUTOSTART_CMDS[val])
        self.write(cmd)
        line = self.readline()

    def getDistOffset(self):
        """
        Get the distance offset
        """
        self.write('OF\n')
        value = self.__readValue()
        value = float(value[2:])
        return value

    def setDistOffset(self,val=None):
        """
        Set the distance offset. 

        Note, If val==None the distance offset is set to the current position.
        """
        if val is None:
            self.write('SO\n')
        else:
            _val = float(val)
            cmd = 'OF{0}\n'.format(val)
            self.write(cmd)
        line = self.readline()

    def printHelp(self):
        """
        Prints help for the control commands.
        """
        self.write('ID\n')
        time.sleep(SERIAL_SLEEP_T)
        while self.inWaiting() > 0:
            line = self.readline()
            try:
                print line[:-1]
            except:
                pass
            time.sleep(SERIAL_SLEEP_T)

    def printSettings(self):
        """
        Prints current settings
        """
        self.write('PA\n')
        time.sleep(SERIAL_SLEEP_T)
        while self.inWaiting() > 0:
            line = self.readline()
            try:
                print line[:-1]
            except:
                pass
            time.sleep(SERIAL_SLEEP_T)

    def resetAllSettings(self):
        """
        Resets all parameters to thier standard values
        """
        self.write('PR\n')
        value = self.readline()

    def __readValue(self):
        line = self.readline()
        line = line.split()
        try:
            value = line[0]
        except:
            value = None
        return value

    def __readAndConvert(self,convert):
        value = self.__readValue()
        if convert is None:
            return value

        _convert = convert.lower()
        if _convert == 'float':
            try:
                value = float(value)
            except:
                value = None
        elif _convert == 'int':
            try:
                value = int(value)
            except:
                value = None
        else:
            raise ValueError, 'unknown conversion {0}'.format(convert)
        return value
            

    def __readFloat(self):
        value = self.__readValue()
        try:
            value = float(value)
        except:
            value = None
        return value
        


# -----------------------------------------------------------------------------
if __name__ == '__main__':

    # Some simple testing - for development

    sensor = DistanceSensor('/dev/ttyUSB0')
    sensor.open()
    if 0:
        print
        print 

        value = sensor.getDistance(convert='float')
        print 'Distance:', value

        value = sensor.getInternalTemp()
        print 'Internal Temp:', value

        value = sensor.getAveraging()
        print 'Averaging:', value
        sensor.setAveraging(1)
        value = sensor.getAveraging()
        print 'Averaging:', value

        value = sensor.getDisplayFormat()
        print 'Display format:', value
        sensor.setDisplayFormat('decimal')
        value = sensor.getDisplayFormat()
        print 'Display format:', value

        value = sensor.getMeasurementTime()
        print 'Measurement time:', value
        sensor.setMeasurementTime(0)
        value = sensor.getMeasurementTime()
        print 'Measurement time:', value

        value = sensor.getScaleFactor()
        print 'Scale factor:', value
        sensor.setScaleFactor(1000.0)
        value = sensor.getScaleFactor()
        print 'Scale factor:', value

        value = sensor.getErrorMode()
        print 'Error mode:', value
        sensor.setErrorMode(1)
        value = sensor.getErrorMode()
        print 'Error mode:', value

        value = sensor.getAlarmCenter()
        print 'Alarm center:', value
        sensor.setAlarmCenter(1.0)
        value = sensor.getAlarmCenter()
        print 'Alarm center:', value

        value = sensor.getAlarmHysteresis()
        print 'Alarm hysteresis:', value
        sensor.setAlarmHysteresis(0.1)
        value = sensor.getAlarmHysteresis()
        print 'Alarm hysteresis:', value

        value = sensor.getAlarmWidth()
        print 'Alarm width:', value
        sensor.setAlarmWidth(100000)
        value = sensor.getAlarmWidth()
        print 'Alarm width:', value

        value = sensor.getIoutBeginDist()
        print 'Iout begin dist:', value
        sensor.setIoutBeginDist(0.1)
        value = sensor.getIoutBeginDist()
        print 'Iout begin dist:', value

        value = sensor.getIoutEndDist()
        print 'Iout end dist:', value
        sensor.setIoutEndDist(25.0)
        value = sensor.getIoutEndDist()
        print 'Iout end dist:', value

        value = sensor.getRemoveMeasurement()
        print 'Remove measurement:', value
        sensor.setRemoveMeasurement(0,0,0)
        value = sensor.getRemoveMeasurement()
        print 'Remove measurement:', value

        value = sensor.getRemoteTrig()
        print 'Remote Trig:', value
        sensor.setRemoteTrig(0,0)
        value = sensor.getRemoteTrig()
        print 'Remote Trig:', value

        value = sensor.getAutoStartTrig()
        print 'AutoStartTrig:', value
        sensor.setAutoStartTrig(0,1)
        value = sensor.getAutoStartTrig()
        print 'AutoStartTrig:', value

        #value = sensor.getBaudrate()
        #print 'Baudrate:', value
        #value = sensor.getAllowedBaudrates()
        #print 'Allowed baudrates:', value
        #sensor.setBaudrate(2400)
        #value = sensor.getBaudrate()
        #print 'Baudrate:', value
        #sensor.setBaudrate(9600)
        #value = sensor.getBaudrate()
        #print 'Baudrate:', value

        value = sensor.getAutoStartCmd()
        print 'AutoStartCmd:', value
        sensor.setAutoStartCmd('help')
        value = sensor.getAutoStartCmd()
        print 'AutoStartCmd:', value

        value = sensor.getDistOffset()
        print 'Dist offset:', value
        sensor.setDistOffset(0.0)
        value = sensor.getDistOffset()
        print 'Dist offset:', value
        sensor.setDistOffset()
        value = sensor.getDistOffset()
        print 'Dist offset:', value
        sensor.setDistOffset(0.0)
        value = sensor.getDistOffset()
        print 'Dist offset:', value

        print 
        print
        sensor.printSettings()

    if 0:
        # Stream value from the sensor and plot them

        import pylab
        sensor.startDistTracking('50hz')
        N = 500
        data = pylab.zeros((N,)) 
        for i in range(0,N):
            value = sensor.readSample(convert='float')
            if not value is None:
                data[i] = value
            else:
                data[i] = 0.0

            print i, value 

        sensor.laserOff()

        pylab.figure(1)
        pylab.plot(data)
        pylab.figure(2)
        pylab.hist(data-data.mean())
        pylab.show()

    sensor.close()



