#!/usr/bin/env python
"""
sled_control.py

Main GUI for Caltech water channel sled control.

"""
import roslib 
roslib.load_manifest('gui')
import rospy
import os
import os.path
import threading
import time
import sys
import h5py
import Queue
import numpy
from PyQt4 import QtCore
from PyQt4 import QtGui
from sled_control_ui import Ui_SledControl_MainWindow
from utilities import HDF5_Run_Reader
from utilities import RobotControl 
from gui_constants import *


class SledControl_MainWindow(QtGui.QMainWindow,Ui_SledControl_MainWindow):
    """
    Main control GUI for the water channel sled control software.
    """

    def __init__(self,startupMode,parent=None):
        super(SledControl_MainWindow,self).__init__(parent)
        self.setupUi(self)
        self.initialize(startupMode)
        self.connectActions()
        self.setupTimers()

    def resizeEvent(self,event):
        """
        Window resize event. 
        """
        super(SledControl_MainWindow,self).resizeEvent(event)

    def closeEvent(self,event):
        """
        Close event handler. Put robot into know state - e.g. disable sled io, 
        current node, etc. prior to shutdown of GUI.
        """
        self.disableRobotControlMode()
        self.ioModeCheckTimer.stop()
        rospy.signal_shutdown('gui closed')
        event.accept()

    def setupTimers(self):
        """
        Setup timers used by the GUI
        """
        self.setupDistVeloMsgTimer()
        self.setupProgressBarTimer()
        self.setupIOModeCheckTimer()
        self.setupStatusMessageTimer()
        self.setupOutscanMonitorTimer()

    def setupDistVeloMsgTimer(self):
        """
        Setup timer for updating distance and velocity messages.
        """
        self.distVeloMsgTimer = QtCore.QTimer(self)
        self.distVeloMsgTimer.setInterval(DIST_VELO_MSG_TIMER_DT)
        self.distVeloMsgTimer.timeout.connect(self.distVeloMsgTimer_Callback)
        self.distVeloMsgTimer.start()

    def distVeloMsgTimer_Callback(self):
        """
        Updates distance and velocity labels using data from the roboControl object.
        """
        position = self.robotControl.position
        velocity = self.robotControl.velocity
        if position is not None:
            positionText = QtCore.QString('Position:  {0:1.3f} (m)'.format(position))
            velocityText = QtCore.QString('Velocity: {0:+1.3f} (m/s)'.format(velocity))
        else:
            positionText = QtCore.QString('Position: None')
            velocityText = QtCore.QString('Velocity: None')
        self.positionLabel.setText(positionText)
        self.velocityLabel.setText(velocityText)

    def setupProgressBarTimer(self):
        """
        Setup timer for updating the progress bar for outscan progress
        """
        self.progressBarTimer = QtCore.QTimer(self)
        self.progressBarTimer.setInterval(PROGRESS_BAR_TIMER_DT)
        self.progressBarTimer.timeout.connect(self.progressBarTimer_Callback)
        self.progressBarTimer.start()

    def progressBarTimer_Callback(self):
        """
        Updates the progress bar based on the outscanPercentComplete value.
        """
        self.progressBar.setValue(self.outscanPercentComplete)

    def setupIOModeCheckTimer(self):
        """
        Setup timer which is used to check the whether or not the sled io mode has changed
        - for example when the sled has gone out of bounds etc. This timer is started when
        the sled io is enabled and stoped when the sled io is stopped.
        """
        self.ioModeCheckTimer = QtCore.QTimer(self)
        self.ioModeCheckTimer.setInterval(IO_MODE_CHECK_TIMER_DT)
        self.ioModeCheckTimer.timeout.connect(self.ioModeCheckTimer_Callback)

    def ioModeCheckTimer_Callback(self):
        """
        When this timer is running sled io should always be enabled. If it is not something
        has happed - like going past the current bounds settings. In which case we need to 
        disable the gui etc. 
        """
        sled_io_enabled = self.robotControl.isSledIOEnabled()
        if not sled_io_enabled:
            self.ioModeCheckTimer.stop()
            self.writeStatusMessage('external disable event')
            self.disableRobotControlMode()
            self.enabled = False
            self.controlMode = None
            self.controlGroupBoxSetEnabled(False)       
            self.updateUIEnabledDisabled()

    def setupStatusMessageTimer(self):
        """
        Setup status message timer which is used to  any messages in the statusMessageQueue.
        The status message queue is used for writing messages to the GUI from other threads.
        I wasn't sure how safe it was to access the Qt stuff from outside the main GUI thread 
        so I decided this queueing approach instead. 
        """
        self.statusMessageTimer = QtCore.QTimer(self)
        self.statusMessageTimer.setInterval(STATUS_MESSAGE_TIMER_DT)
        self.statusMessageTimer.timeout.connect(self.statusMessageTimer_Callback)
        self.statusMessageTimer.start()

    def statusMessageTimer_Callback(self):
        """
        Writes any messages in the statusMessageQueue to the status message window.
        """
        try:
            message = self.statusMessageQueue.get(False)
            self.writeStatusMessage(message)
        except Queue.Empty:
            pass

    def setupOutscanMonitorTimer(self):
        """
        Setup outscan monitor timer. This timer is used to monitor the outscanInProgress
        variable to determine when an outscan has completed.
        """
        self.outscanMonitorTimer = QtCore.QTimer(self)
        self.outscanMonitorTimer.setInterval(OUTSCAN_MONITOR_TIMER_DT)
        self.outscanMonitorTimer.timeout.connect(self.outscanMonitorTimer_Callback)
        self.outscanMonitorTimer.start()

    def outscanMonitorTimer_Callback(self):
        """
        Updates the GUI state based on the value of the outscanStopSignal. 
        """
        if self.enabled:
            with self.lock:
                if self.outscanStopSignal:
                    stop = True
                else:
                    stop = False
            if stop:
                self.outscanStopActions()

    def initialize(self,startupMode):
        """
        Initialize the state of the GUI and system.
        """
        
        self.lock = threading.Lock()
        self.enabled = False
        self.statusMessageCnt = 0
        self.statusMessageQueue = Queue.Queue()
        self.startupMode = startupMode 
        self.controlMode = None
        self.runFileReader = None 
        self.outscanPercentComplete = 0
        self.outscanInProgress = False
        self.outscanStopSignal = False
        self.autorunDelay = int(DEFAULT_AUTORUN_DELAY)
        self.startPosition = DEFAULT_START_POSITION
        self.runNumber = None
        self.updateAutorun(DEFAULT_AUTORUN_CHECK)

        # Start ros node and initialize robot control
        rospy.init_node('gui')
        self.robotControl = RobotControl()

        self.writeStatusMessage('initializing')
        self.checkStartupMode()
        self.writeStatusMessage('mode = %s'%(self.startupMode,))

        # Set text for mode group box - this will change based on startup mode 
        self.modeGroupBox.setTitle(self.startupMode.title())

        # Set current tab to control tab - can this be done by name
        self.mainTabWidget.setCurrentWidget(self.controlTab)
        self.modeGroupBox.setChecked(False)
        self.joystickGroupBox.setChecked(False)
        self.feedbackGroupBox.setChecked(False)

        # Set background and text colors of status window
        bgColor = QtGui.QColor(*STATUS_WINDOW_BACKGROUND_COLOR)
        textColor = QtGui.QColor(*STATUS_WINDOW_TEXT_COLOR)
        palette = self.statusWindowTextEdit.palette()
        palette.setColor(QtGui.QPalette.Active, QtGui.QPalette.Base, bgColor)
        palette.setColor(QtGui.QPalette.Inactive, QtGui.QPalette.Base, bgColor)
        palette.setColor(QtGui.QPalette.Active, QtGui.QPalette.Text, textColor)
        self.statusWindowTextEdit.setPalette(palette)

        # Get current upper and lower bound settings
        bounds = self.robotControl.getBounds()
        self.lowerBound, self.upperBound, self.lowerBoundMin, self.upperBoundMax = bounds
        
        # Create bound validators and set text values
        self.setBoundValidators()
        self.lowerBoundLineEdit.setText('%1.2f'%(self.lowerBound,))
        self.upperBoundLineEdit.setText('%1.2f'%(self.upperBound,))
        self.lowerBoundMinLabel.setText('Lower Min: %s'%(self.lowerBoundMinStr,))
        self.upperBoundMaxLabel.setText('Upper Max: %s'%(self.upperBoundMaxStr,))

        # Get current joystick max velocity setting, create validator and set line edit
        self.joystickMaxVelo = int(self.robotControl.getJoystickMaxVelo())
        maxVelValidator = QtGui.QIntValidator(1,100,self.joystickMaxVeloLineEdit)
        self.joystickMaxVeloLineEdit.setValidator(maxVelValidator)
        self.joystickMaxVeloLineEdit.setText('%d'%(self.joystickMaxVelo,))

        # Set  autorun delay validator and line edit text 
        autorunDelayValidator = QtGui.QIntValidator(
                MIN_AUTORUN_DELAY,
                MAX_AUTORUN_DELAY,
                self.autorunDelayLineEdit,
                )
        self.autorunDelayLineEdit.setValidator(autorunDelayValidator)
        self.autorunDelayLineEdit.setText('%d'%(self.autorunDelay,))

        # Set start position validator and line edit text
        self.setStartPositionValidator()
        #self.startPositionLineEdit.setText('%1.3f'%(DEFAULT_START_POSITION,))
        self.startPositionLineEdit.setText('%s'%(self.startPositionStr,))

        # Enable/Disable appropriate widgets based on enabled state
        self.updateUIEnabledDisabled()

        # Setup default directories
        self.defaultRunFileDir = os.getenv('HOME')
        self.lastRunFileDir = self.defaultRunFileDir

        # Enable load and clear run file pushbutton enable values
        self.loadRunFilePushButton.setEnabled(True)
        self.clearRunFilePushButton.setEnabled(False)

        self.runTreeWidget.setColumnCount(2) 
        self.runTreeWidget.setHeaderLabels(['name', 'type/value'])
        self.statusbar.showMessage('')
        
        self.progressBar.setValue(0)
        self.progressBar.setVisible(False)


    def connectActions(self):
        """
        Connects events to the appropriate callback functions.
        """
        # Main window actions
        self.enableDisablePushButton.clicked.connect(self.enableDisable_Callback)

        # Actions for Controls tab
        self.modeGroupBox.clicked.connect(self.modeCheck_Callback)
        self.autorunCheckBox.stateChanged.connect(self.autorunCheckBox_Callback)
        self.runNumberLineEdit.editingFinished.connect(self.runNumberChanged_Callback)
        self.startPositionLineEdit.editingFinished.connect(self.startPositionChanged_Callback)
        self.autorunDelayLineEdit.editingFinished.connect(self.autorunDelayChanged_Callback)
        
        self.joystickGroupBox.clicked.connect(self.joystickCheck_Callback)
        self.feedbackGroupBox.clicked.connect(self.feedbackCheck_Callback)
        self.startPushButton.clicked.connect(self.start_Callback)
        self.stopPushButton.clicked.connect(self.stop_Callback)
        self.lowerBoundLineEdit.editingFinished.connect(self.lowerBoundChanged_Callback)
        self.upperBoundLineEdit.editingFinished.connect(self.upperBoundChanged_Callback)
        self.joystickMaxVeloLineEdit.editingFinished.connect(self.joystickMaxVeloChanged_Callback)

        # Actions for runs tab
        self.loadRunFilePushButton.clicked.connect(self.loadRunFile_Callback)
        self.clearRunFilePushButton.clicked.connect(self.clearRunFile_Callback)

        # Actions for log tab
        self.setLogFilePushButton.clicked.connect(self.setLogFile_Callback)
        self.deleteLogItemPushButton.clicked.connect(self.deleteLogItem_Callback)

        # Run tree actions
        self.runTreeWidget.itemClicked.connect(self.runTreeItemClicked_Callback)


    def modeCheck_Callback(self,checkValue):
        """
        Callback for checkbox of modeGroupBox - this is called the the user
        selects the current control mode (which depends on the startup
        conditions) e.g. captive trajectory, interial run, positon control.
        """
        if checkValue:
            self.joystickGroupBox.setEnabled(False)
            self.feedbackGroupBox.setEnabled(False)
            msg = '%s enabled'%(self.startupMode,)
            self.writeStatusMessage(msg)
            self.startPushButton.setEnabled(True)
            self.controlMode = 'startupMode'
        else:
            self.controlGroupBoxSetEnabled(True)
            self.startPushButton.setEnabled(False)
            msg = '%s disabled'%(self.startupMode,)
            self.writeStatusMessage(msg)
            self.controlMode = None

    def autorunCheckBox_Callback(self,checkValue):
        """
        Callback for he autorun check box. Turns on/off the autorun feature.  
        """
        if checkValue == QtCore.Qt.Checked:
            self.autorun = True
            self.writeStatusMessage('autorun enabled')
        else:
            self.autorun = False
            self.writeStatusMessage('autorun disabled')

    def updateAutorun(self,value):
        """
        Updates autorun value and checkbox.
        """
        self.autorun = value
        self.autorunCheckBox.setChecked(value)

    def runNumberChanged_Callback(self):
        """
        Callback for updating changes to the run number
        """
        runNumberNew = self.runNumberLineEdit.text()
        runNumberNew = int(runNumberNew)
        if runNumberNew != self.runNumber:
            self.runNumber = runNumberNew
            self.writeStatusMessage('run number set to %d'%(self.runNumber,))

    @property
    def startPositionStr(self):
        """
        Provides a string representation for the start position
        """
        return '%1.3f'%(self.startPosition,)

    def startPositionChanged_Callback(self):
        """
        Callback for handling changes to the start position.
        """
        startPositionNew = self.startPositionLineEdit.text()
        startPositionNew = float(startPositionNew)
        if startPositionNew != self.startPosition:
            startPositionNew = startPositionNew
            self.startPosition = startPositionNew
            self.writeStatusMessage('start position set to %s m'%(self.startPositionStr,))
        self.startPositionLineEdit.setText('%s'%(self.startPositionStr,))

    def setStartPositionValidator(self):
        """
        Sets start position validator based on the current upper and lower
        bounds settings.
        """
        startPositionValidator = QtGui.QDoubleValidator(self.startPositionLineEdit)
        startPositionValidator.setRange(self.lowerBound, self.upperBound, 3)
        startPositionValidator.fixup = self.startPositionFixup
        self.startPositionLineEdit.setValidator(startPositionValidator)

    def startPositionFixup(self,value):
        """
        Fixup funtion for start position value which are out of bounds.  Just puts 
        the start position back to the previous value if it is set out of bounds.
        """
        self.startPositionLineEdit.setText('%s'%(self.startPositionStr,))

    def autorunDelayChanged_Callback(self):
        """
        Callback for handling updates tothe autorun delay value
        """
        autorunDelayNew = self.autorunDelayLineEdit.text()
        autorunDelayNew = int(autorunDelayNew)
        if autorunDelayNew != self.autorunDelay:
            self.autorunDelay = autorunDelayNew
            self.writeStatusMessage('autorun delay set to %d s'%(self.autorunDelay,))

    def joystickCheck_Callback(self,checkValue):
        """
        Callback for the checkbox of the joystickGroupBox. This is called when the user
        selects/deselects joystick control mode.
        """
        if checkValue:
            self.modeGroupBox.setEnabled(False)
            self.feedbackGroupBox.setEnabled(False)
            self.startPushButton.setEnabled(True)
            self.writeStatusMessage('joystick positioning enabled')
            self.controlMode = 'joystick' 
        else:
            self.controlGroupBoxSetEnabled(True)
            self.startPushButton.setEnabled(False)
            self.writeStatusMessage('joystick positioning disabled')
            self.controlMode = None

    def joystickMaxVeloChanged_Callback(self):
        """
        Callback function for when joystick positioning mode max velocity
        line edit. 
        """
        joystickMaxVeloNew = self.joystickMaxVeloLineEdit.text()
        joystickMaxVeloNew = float(joystickMaxVeloNew)
        joystickMaxVeloOld = self.joystickMaxVelo
        if joystickMaxVeloNew != joystickMaxVeloOld:
            flag = self.robotControl.setJoystickMaxVelo(joystickMaxVeloNew)
            if flag:
                # Value set succefully
                self.joystickMaxVelo = joystickMaxVeloNew
                msg = 'joystick max velocity set to %d'%(int(joystickMaxVeloNew),)
                self.writeStatusMessage(msg)
            else:
                # Value set failed.
                msg = 'setting joystick max velocity failed'
                self.writeStatusMessage(msg)

    def feedbackCheck_Callback(self,checkValue):
        """
        Callback for the checkbox of the feedbackGroupBox. This is called when the user
        selects/deselects feedback position control mode.
        """
        if checkValue:
            self.modeGroupBox.setEnabled(False)
            self.joystickGroupBox.setEnabled(False)
            self.startPushButton.setEnabled(True)
            self.writeStatusMessage('feedback positioning enabled')
            self.controlMode = 'feedback'
        else:
            self.controlGroupBoxSetEnabled(True)
            self.startPushButton.setEnabled(False)
            self.writeStatusMessage('feedback positioning disabled')
            self.controlMode = None

    def boundStr(self, value):
        return '%1.2f'%(value,)

    @property
    def lowerBoundStr(self):
        return self.boundStr(self.lowerBound,)

    @property
    def upperBoundStr(self):
        return self.boundStr(self.upperBound,) 
    
    @property
    def lowerBoundMinStr(self):
        return self.boundStr(self.lowerBoundMin,)

    @property
    def upperBoundMaxStr(self):
        return self.boundStr(self.upperBoundMax,)


    def lowerBoundChanged_Callback(self):
        """
        Callback for changes to the lower bound line edit.
        """
        lowerBoundOld = self.lowerBound
        lowerBoundNew = float(self.lowerBoundLineEdit.text())
        if lowerBoundNew != lowerBoundOld:
            flag = self.robotControl.setBounds(lowerBoundNew,self.upperBound)
            if flag:
                # Value set successfully
                self.lowerBound = lowerBoundNew
                self.setBoundValidators()
                if lowerBoundNew != lowerBoundOld:
                    msg = 'lower bound changed to %s m'%(self.lowerBoundStr,)
                    self.writeStatusMessage(msg)
            else:
                # Value set failed
                msg = 'setting lower bound failed'
                self.writeStatusMessage(msg)
        self.lowerBoundLineEdit.setText('%s'%(self.lowerBoundStr,))
        self.checkStartPosOnBoundChange()
        self.setStartPositionValidator()
            
    def upperBoundChanged_Callback(self):
        """
        Callback for changes to the upper bound line edit.
        """
        upperBoundOld = self.upperBound
        upperBoundNew = float(self.upperBoundLineEdit.text())
        if upperBoundNew != upperBoundOld:
            flag = self.robotControl.setBounds(self.lowerBound,upperBoundNew)
            if flag:
                # Value set successfully
                self.upperBound = upperBoundNew
                self.setBoundValidators()
                if upperBoundNew != upperBoundOld:
                    msg = 'upper bound changed to %s m'%(self.upperBoundStr,)
                    self.writeStatusMessage(msg)
            else:
                # Value set failed
                msg = 'setting upper bound failed'
                self.writeStatusMessage(msg)
        self.upperBoundLineEdit.setText('%s'%(self.upperBoundStr,))
        self.checkStartPosOnBoundChange()
        self.setStartPositionValidator()

    def setBoundValidators(self):
        """
        Set validators for upper and lower bound line entries
        """
        lowerBoundValidator = QtGui.QDoubleValidator(self.lowerBoundLineEdit)
        lowerBoundValidator.setRange(self.lowerBoundMin, self.upperBound, 2)
        lowerBoundValidator.fixup = self.lowerBoundFixup
        self.lowerBoundLineEdit.setValidator(lowerBoundValidator)

        upperBoundValidator = QtGui.QDoubleValidator(self.upperBoundLineEdit)
        upperBoundValidator.setRange(self.lowerBound, self.upperBoundMax, 2)
        upperBoundValidator.fixup = self.upperBoundFixup
        self.upperBoundLineEdit.setValidator(upperBoundValidator)

    def lowerBoundFixup(self,value):
        """
        Fixup function for position lower bounds line edit entry. 
        """
        value = float(value)
        if value == self.lowerBoundMin:
            self.lowerBound = value
        self.lowerBoundLineEdit.setText('%s'%(self.lowerBoundStr,))
        
    def upperBoundFixup(self,value):
        """
        Fixup function for position upper bound line edit enty.
        """
        self.upperBoundLineEdit.setText('%s'%(self.upperBoundStr,))

    def checkStartPosOnBoundChange(self):
        """
        Checks the start position when the lower and upper bounds are changed. Adjusts
        the start position if it is out side of the bounds.
        """
        changed = False
        if self.startPosition < self.lowerBound:
            self.startPosition = self.lowerBound
            changed = True
            msg = 'start position < lower bound, changed to %s m'%(self.startPositionStr,)
        if self.startPosition > self.upperBound:
            self.startPosition = self.upperBound
            changed = True
            msg = 'start position > upper bound, changed to %s m'%(self.startPositionStr,)
        if changed:
            self.startPositionLineEdit.setText('%s'%(self.startPositionStr,))
            self.writeStatusMessage(msg)

    def controlGroupBoxSetEnabled(self,value,uncheck_on_disable=True,enable_only_checked=False):
        """
        Enable/Disable the group boxes on the control tab. If uncheck_on_disable is True than
        all groupBoxes are unchecked when they are disabled. If enalbe_only_checked = True than
        only groupBoxes which are checked will be enabled.
        """
        if value and enable_only_checked and self.isControlGroupBoxChecked():
            if self.joystickGroupBox.isChecked():
                self.joystickGroupBox.setEnabled(True)
            if self.feedbackGroupBox.isChecked():
                self.feedbackGroupBox.setEnabled(True)
            if self.modeGroupBox.isChecked() and (self.runFileReader is not None):
                # We have a run file reader - enable
                self.modeGroupBox.setEnabled(True)
            else:
                # No run file reader - disable and enable remaining
                self.modeGroupBox.setEnabled(False)
                self.modeGroupBox.setChecked(False)
                if (not self.joystickGroupBox.isChecked()) and (not self.feedbackGroupBox.isChecked()):
                    self.joystickGroupBox.setEnabled(True)
                    self.feedbackGroupBox.setEnabled(True)
        else:
            # Only enable mode only runfile is loaded
            if value and (self.runFileReader is not None): 
                self.modeGroupBox.setEnabled(True)
            else:
                self.modeGroupBox.setEnabled(False)
            self.joystickGroupBox.setEnabled(value)
            self.feedbackGroupBox.setEnabled(value)

        if not value and uncheck_on_disable:
            self.controlGroupBoxSetChecked(False)
            
    def isControlGroupBoxChecked(self):
        """
        Returns true if one of the control group boxes is checked.
        """
        value = False
        value = value or self.modeGroupBox.isChecked()
        value = value or self.joystickGroupBox.isChecked()
        value = value or self.feedbackGroupBox.isChecked()
        return value

    def controlGroupBoxSetChecked(self,value):
        """
        Sets the checked state of all the groupboxes on the control tab.
        """
        self.modeGroupBox.setChecked(value)
        self.joystickGroupBox.setChecked(value)
        self.feedbackGroupBox.setChecked(value)

    def start_Callback(self):
        """
        Callback for when the start button on the Control tab is clicked.
        """
        self.stopPushButton.setEnabled(True)
        self.startPushButton.setEnabled(False)
        self.positionBoundsGroupBox.setEnabled(False)
        self.controlGroupBoxSetEnabled(False,uncheck_on_disable=False)
        with self.lock:
            self.outscanInProgress = True
        self.enableRobotControlMode()
        
    def stop_Callback(self):
        """
        Callback for when the stop button in the Control tab is clicked.
        """
        self.outscanStopActions()

    def outscanStopActions(self):
        """
        Actions which which should be performed when an outscan is stopped.
        """
        with self.lock:
            self.outscanStopSignal = False
            self.outscanInProgress = False
            self.startPushButton.setEnabled(True)
            self.stopPushButton.setEnabled(False)
            self.positionBoundsGroupBox.setEnabled(True)
            self.controlGroupBoxSetEnabled(True,enable_only_checked=True)
            self.disableRobotControlMode()

    def disableRobotControlMode(self):
        """
        Disable current control mode via robotControl
        """
        if self.controlMode == 'joystick':
            self.robotControl.disableJoystickMode()
            self.writeStatusMessage('joystick positioning stopped')
        elif self.controlMode == 'feedback':
            self.writeStatusMessage('feedback positioning stopped')
            self.progressBar.setVisible(False)
        elif self.controlMode == 'startupMode':
            self.robotControl.stopSetptOutscan()
            self.progressBar.setVisible(False)
            self.writeStatusMessage('%s stopped'%(self.startupMode,))

    def enableRobotControlMode(self):
        """
        Enable current control mode via roboControl
        """
        if self.controlMode == 'joystick':
            self.robotControl.enableJoystickMode()
            self.writeStatusMessage('joystick positioning started')
        elif self.controlMode == 'feedback':
            self.writeStatusMessage('feedback positioning started')
            self.progressBar.setVisible(True)
        elif self.controlMode == 'startupMode':
            self.writeStatusMessage('%s started'%(self.startupMode,))
            self.progressBar.setVisible(True)
            self.progressBar.setValue(0)
            self.writeStatusMessage('loading run # %d for outscan'%(self.runNumber,))
            self.startSetptOutscan()

    def startSetptOutscan(self): 
        """
        Starts a set point outscan using the roboControl object.
        """
        # Load run based on run number
        setptValues = numpy.arange(1000,dtype=numpy.float)
        setptValues[0] = self.robotControl.position

        # Start setpt outscan
        self.writeStatusMessage('running outscan')
        with self.lock:
            self.outscanPercentComplete = 0
        try:
            self.robotControl.startSetptOutscan(
                    setptValues,
                    feedback_cb = self.outscanFeedback_Callback,
                    done_cb = self.outscanDone_Callback,
                    )
        except ValueError, e:
            self.writeStatusMessage('error: %s'%(str(e),))
            with self.lock:
                self.outscanStopSignal = True
        
    def outscanFeedback_Callback(self,data):
        """
        Callback funtion for outscan feed back. Gets the percent complete and
        stores it for display by the progressBarTimer_Callback.
        """
        with self.lock:
            self.outscanPercentComplete = int(data.percent_complete)

    def outscanDone_Callback(self,state,result):
        """
        Callback function for signaling when a run is complete.
        """
        if state == 'succeeded':
            self.statusMessageQueue.put('outscan complete')
        elif state == 'aborted':
            self.updateAutorun(False)
            self.statusMessageQueue.put('outscan aborted')
        else:
            self.updateAutorun(False)
            self.statusMessageQueue.put('unknow state messsage - aborting')

        with self.lock:
            self.outscanPercentComplete = 100
            # Take action based on autorun setting
            if self.autorun == False:
                # Check if the system thinks an outscan is inProgress - if so send
                # the outscan stop signal to take appropriate actions.
                if self.outscanInProgress:
                    self.outscanStopSignal = True
            else:
                # Autorun is enabled - move to the starting position and load the
                # next run.
                pass

    def enableDisable_Callback(self):
        """
        Callback for when the enable/disable button is clicked.  Enables sled
        IO by putting the sled_io_node into motor command mode. Disables sled
        IO by puttin the seld_io_nod into off mode. 
        """
        if self.enabled:
            self.robotControl.disableSledIO()
            self.ioModeCheckTimer.stop()
            self.outscanStopActions()
            self.enabled = False
        else:
            self.robotControl.enableSledIO()
            self.ioModeCheckTimer.start()
            self.enabled = True
        self.updateUIEnabledDisabled()

    def updateUIEnabledDisabled(self):
        """
        Sets the text of the display and enables/disables the appropriate 
        widgets based the the enabled state of the system.
        """
        if self.enabled:
            self.writeStatusMessage('system enabled')
            self.enableDisablePushButton.setText('Disable')
            self.controlGroupBoxSetEnabled(True)
        else:
            self.writeStatusMessage('system disabled')
            self.enableDisablePushButton.setText('Enable')
            self.startPushButton.setEnabled(False)
            self.stopPushButton.setEnabled(False)
            self.controlGroupBoxSetEnabled(False)
            self.positionBoundsGroupBox.setEnabled(True)

    def loadRunFile_Callback(self):
        """
        Callback for loading run files - if the run file is valid it will populate 
        the runFileTreeWidget from the data in the selected run file using the populateRunTree
        method.
        """
        # Try to load run file
        filename = QtGui.QFileDialog.getOpenFileName(None,'Select run file',self.lastRunFileDir)
        if filename:
            filename = str(filename)
            try:
                self.runFileReader = HDF5_Run_Reader(filename)
            except h5py.h5e.LowLevelIOError, e:
                self.cleanUpRunFile()
                self.writeStatusMessage('run file load failed: %s'%(str(e),))
                return

            # File has beed successfully loaded check that mode is compatible
            runFileMode = self.runFileReader.get_mode()
            if self.startupMode != runFileMode:
                self.cleanUpRunFile()
                self.writeStatusMessage('file incompatible: mode = %s'%(runFileMode,))
                return

            # Mode matches populate run tree
            self.populateRunTree()
            self.loadRunFilePushButton.setEnabled(False)
            self.clearRunFilePushButton.setEnabled(True)
            pathname, filename = os.path.split(filename)
            self.writeStatusMessage('run file loaded: %s'%(filename,))
            runNumber_validator = QtGui.QIntValidator(
                        0,
                        self.runFileReader.number_of_runs-1,
                        self.runNumberLineEdit,
                        )
            self.runNumberLineEdit.setValidator(runNumber_validator)
            self.setRunNumber(0)
            if self.enabled:
                self.controlGroupBoxSetEnabled(True,enable_only_checked=True)

    def cleanUpRunFile(self): 
        self.runFileReader.close()
        self.runFileReader = None
        self.runTreeWidget.clear()
        self.loadRunFilePushButton.setEnabled(True)
        self.clearRunFilePushButton.setEnabled(False)
        self.statusbar.showMessage('')
        self.runNumberLineEdit.setText('')


    def populateRunTree(self):
        """
        Populates the the runTreeWidget from the current hdf5 run file.
        """
        self.runTreeWidget.clear()
        self.statusbar.showMessage('Run File: %s'%(self.runFileReader.filename,))
        for run in self.runFileReader:
            item = QtGui.QTreeWidgetItem(self.runTreeWidget,0)
            item.setText(0,run.name[1:])
            item.setText(1,run.attrs['type'])
            self.addChildToRunTree(item,run)

    def addChildToRunTree(self,parent,hdf5Item):
        """
        Add child nodes to the run tree. 
        """
        name_list = list(hdf5Item)
        name_list.sort()
        for name in name_list:
            obj = hdf5Item[name]
            item = QtGui.QTreeWidgetItem(parent,0)
            item.setText(0,name)
            self.addAttrsToItem(item,obj)
            if isinstance(obj,h5py.highlevel.Dataset):
                if obj.shape[0] == 1:
                    value = float(obj[0])
                    valueStr = '%1.3f'%(value,)
                    item.setText(1,valueStr)
                else:
                    shape = obj.shape
                    shapeStr = '%s'%(shape,)
                    item.setText(1,shapeStr)
            else:
                self.addChildToRunTree(item, obj)

    def addAttrsToItem(self,item,obj):
        """
        Adds child nodes to the run tree item for hdf5 attributes of the obj.  
        """
        for name, value in obj.attrs.iteritems():
            attrItem = QtGui.QTreeWidgetItem(item,0)
            attrItem.setText(0,name)
            attrItem.setText(1,value)

    def clearRunFile_Callback(self):
        """
        Clear loaded run file.
        """
        self.cleanUpRunFile()
        self.writeStatusMessage('run file cleared')
        if self.enabled:
            self.controlGroupBoxSetEnabled(True,enable_only_checked=True)

    def runTreeItemClicked_Callback(self,item):
        print 'run tree - item clicked'
        topLevelParent = self.getRunTreeTopLevelParent(item)
        print item.text(0)
        print topLevelParent.text(0)

    def getRunTreeTopLevelParent(self,item):
        while item.parent() is not None:
            item = item.parent()
        return item

    #def addItemsToRunTree(self):
    #    """
    #    Temporary test function.
    #    """
    #    topItem = QtGui.QTreeWidgetItem(self.runTreeWidget,0)
    #    topItem.setText(0,'Filename')
    #    for i in range(0,100):
    #        item = QtGui.QTreeWidgetItem(topItem,0)
    #        item.setText(0,'run %d'%(i,))

    def setLogFile_Callback(self):
        print 'set log file'

    def deleteLogItem_Callback(self):
        print 'delete log item'


    def writeStatusMessage(self,msg):
        """
        Writes a status message to the status window.
        """
        # Write status message
        statusMsg = '%d: %s'%(self.statusMessageCnt,msg)
        self.statusMessageCnt += 1
        self.statusWindowTextEdit.append(statusMsg)
        # Move cursor to end of document
        cursor = self.statusWindowTextEdit.textCursor()
        cursor.movePosition(QtGui.QTextCursor.End)
        self.statusWindowTextEdit.setTextCursor(cursor)

    def setRunNumber(self,value):
        """
        Sets the current run number
        """
        self.runNumber = value
        self.runNumberLineEdit.setText('%d'%(value,))

    def checkStartupMode(self):
        if not self.startupMode in ALLOWED_STARTUP_MODES:
            raise ValueError, 'unknown startup mode: %s'%(self.startupMode,)

    def main(self):
        self.show()
        

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    startupMode = sys.argv[1]
    sledControl = SledControl_MainWindow(startupMode)
    sledControl.main()
    app.exec_()

