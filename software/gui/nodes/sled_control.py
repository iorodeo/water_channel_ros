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
import matplotlib
matplotlib.use('Qt4Agg')
import pylab
from PyQt4 import QtCore
from PyQt4 import QtGui
from sled_control_ui import Ui_SledControl_MainWindow
from utilities.hdf5_run_reader import HDF5_Run_Reader
from utilities.robot_control import Robot_Control 
from utilities import run_defs
from utilities import run_converter
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
        pylab.ioff()
        pylab.close()
        self.robotControl.disableLogger()
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
        self.setupAutorunDelayTimer()

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
        Setup outscan monitor timer. This timer is used to monitor the outscanStopSignal
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
                stopSignal = self.outscanStopSignal
            if stopSignal:
                self.outscanStopActions()
                return
            if self.runInProgress: 
                with self.lock:
                    outscanInProgress = self.outscanInProgress
                if outscanInProgress == False:
                    if self.logFileName is not None:
                        self.populateLogTree()
                    if self.controlMode == 'startupMode':
                        self.updateTrialState()
                        if self.runState != 'done':
                            self.startNextOutscan()
                        else:
                            self.outscanStopActions()
                    else:
                        if self.pluginRunning:
                            if self.runState != 'done':
                                self.pluginObj.startNextOutscan()
                            else:
                                self.pluginObj.cleanup()
                                self.outscanStopActions()
                        else:
                            self.outscanStopActions()

    def setupAutorunDelayTimer(self):
        """
        Creates the timer used for handling the autorun delays. 
        """
        self.autorunDelayTimer = QtCore.QTimer(self)
        self.updateAutorunDelayTimerInterval(self.autorunDelay)
        self.autorunDelayTimer.timeout.connect(self.autorunDelayTimer_Callback)

    def updateAutorunDelayTimerInterval(self,value):
        if value > 0:
            autorunTimerDelay = sec2ms(value)/AUTORUN_DELAY_TIMER_MAX_COUNT
        else:
            autorunTimerDelay
        self.autorunDelayTimer.setInterval(autorunTimerDelay)

    def autorunDelayTimer_Callback(self):
        """
        Autorun delay  timer callback.
        """
        self.autorunDelayCount+=1
        with self.lock:
            self.outscanPercentComplete = self.autorunDelayCount
        if self.autorunDelayCount == AUTORUN_DELAY_TIMER_MAX_COUNT:
            self.autorunDelayTimer.stop()
            with self.lock:
                self.outscanInProgress = False

    def initialize(self,startupMode):
        """
        Initialize the state of the GUI and system.
        """
        pylab.ion()
        
        self.lock = threading.Lock()
        self.enabled = False
        self.statusMessageCnt = 0
        self.statusMessageQueue = Queue.Queue()
        self.startupMode = startupMode 
        self.controlMode = None
        self.runFileReader = None 
        self.logFileName = None 
        self.outscanPercentComplete = 0
        self.outscanInProgress = False
        self.outscanStopSignal = False
        self.runInProgress = False
        self.autorunDelay = int(DEFAULT_AUTORUN_DELAY)
        self.autorunDelayCount = 0
        self.runNumber = None
        self.updateAutorun(DEFAULT_AUTORUN_CHECK)

        # Start ros node and initialize robot control
        rospy.init_node('gui')
        self.robotControl = Robot_Control(self.startupMode)
        self.robotControl.setPwmToDefault()

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
        self.startPosition = 0.5*(self.lowerBound + self.upperBound) 
        
        # Create bound validators and set text values
        self.setBoundValidators()
        self.lowerBoundLineEdit.setText(self.lowerBoundStr)   
        self.upperBoundLineEdit.setText(self.upperBoundStr)
        self.lowerBoundMinLabel.setText('Lower Min: %s'%(self.lowerBoundMinStr,))
        self.upperBoundMaxLabel.setText('Upper Max: %s'%(self.upperBoundMaxStr,))
        
        # Set feedback positioning default values
        self.setFeedbackPositionValidator()
        self.feedbackPosition = 0.5*(self.upperBound + self.lowerBound)
        self.feedbackPositionLineEdit.setText(self.feedbackPositionStr)

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
        self.startPositionLineEdit.setText('%s'%(self.startPositionStr,))

        # Enable/Disable appropriate widgets based on enabled state
        self.updateUIEnabledDisabled()

        # Setup default directories
        self.defaultRunFileDir = os.getenv('HOME')
        self.defaultLogFileDir = os.getenv('HOME')
        self.lastRunFileDir = self.defaultRunFileDir
        self.lastLogFileDir = self.defaultLogFileDir
        self.robotControl.setLogFile(DEFAULT_LOG_FILE)

        # Enable load and clear run file pushbutton enable values
        self.loadRunFilePushButton.setEnabled(True)
        self.clearRunFilePushButton.setEnabled(False)

        # Enable set, del item and clear log file push button values
        self.setLogFilePushButton.setEnabled(True)
        self.deleteLogItemPushButton.setEnabled(False)
        self.clearLogFilePushButton.setEnabled(False)

        # Set up run tree widget
        self.runTreeWidget.setColumnCount(2) 
        self.runTreeWidget.setHeaderLabels(['name', 'type/value'])

        self.logTreeWidget.setColumnCount(2)
        self.logTreeWidget.setHeaderLabels(['trial', 'type/value'])

        self.statusbar.showMessage('System Disabled')
        self.progressBar.setValue(0)
        self.progressBar.setVisible(False)

        # Setup plugins
        self.pluginObj = None
        self.pluginRunning = False
        self.pluginDict = self.importPlugins()
        for pluginName in self.pluginDict:
            listItem = QtGui.QListWidgetItem(self.pluginListWidget)
            listItem.setText(pluginName)
            self.pluginListWidget.addItem(listItem)

        self.pluginStartPushButton.setEnabled(False)
        self.pluginStopPushButton.setEnabled(False)

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
        self.feedbackPositionLineEdit.editingFinished.connect(self.feedbackPositionChanged_Callback)

        # Actions for runs tab
        self.loadRunFilePushButton.clicked.connect(self.loadRunFile_Callback)
        self.clearRunFilePushButton.clicked.connect(self.clearRunFile_Callback)

        # Run tree actions - plot run on right click
        self.runTreeWidget.contextMenuEvent = self.runTreeItemRightClicked_Callback

        # Actions for log tab
        self.setLogFilePushButton.clicked.connect(self.setLogFile_Callback)
        self.deleteLogItemPushButton.clicked.connect(self.deleteLogItem_Callback)
        self.clearLogFilePushButton.clicked.connect(self.clearLogFile_Callback)

        #Log tree actions - plot run on right click
        self.logTreeWidget.contextMenuEvent = self.logTreeItemRightClicked_Callback
        self.logTreeWidget.itemClicked.connect(self.logTreeItemClicked_Callback)

        # Actions for the plugin tab
        self.pluginStartPushButton.clicked.connect(self.pluginStart_Callback)
        self.pluginStopPushButton.clicked.connect(self.pluginStop_Callback)
        self.pluginListWidget.itemClicked.connect(self.pluginItemClicked_Callback)

    def pluginStart_Callback(self):
        currentItem = self.pluginListWidget.currentItem()
        if currentItem is not None:
            self.pluginStartPushButton.setEnabled(False)
            self.pluginStopPushButton.setEnabled(True)
            pluginName = str(currentItem.text())
            pluginModule = self.pluginDict[pluginName]
            self.pluginObj = pluginModule.SledControlPlugin(
                    self.robotControl, 
                    self.pluginSetOutscanInProgress,
                    self.pluginSetRunStateDone,
                    self.writeStatusMessage,
                    )
            self.runInProgress = True
            self.pluginRunning = True
            self.runState = 'plugin running'
            with self.lock:
                self.outscanInProgress = False
            self.controlTab.setEnabled(False)
            self.runsTab.setEnabled(False)
            self.logTab.setEnabled(False)
            self.statusbar.showMessage('System Enabled - Plugin Running')

    def pluginStop_Callback(self):
        self.pluginStartPushButton.setEnabled(True)
        self.pluginStopPushButton.setEnabled(False)

    def pluginItemClicked_Callback(self):
        currentItem = self.pluginListWidget.currentItem()
        self.pluginStartPushButton.setEnabled(True)

    def pluginSetOutscanInProgress(self,value):
        value = bool(value)
        with self.lock:
            self.outscanInProgress = value

    def pluginSetRunStateDone(self):
        with self.lock:
            self.runState = 'done'

    def pluginWriteStatusMessage(self,msg):
        self.writeStatusMessage(msg)
        self.statusWindowTextEdit.repaint()

    def logTreeItemClicked_Callback(self):
        currentItem = self.logTreeWidget.currentItem()
        if currentItem.parent() is None:
            self.deleteLogItemPushButton.setEnabled(True)
        else:
            self.deleteLogItemPushButton.setEnabled(False)

    def modeCheck_Callback(self,checkValue):
        """
        Callback for checkbox of modeGroupBox - this is called the the user
        selects the current control mode (which depends on the startup
        conditions) e.g. captive trajectory, interial run, positon control.
        """
        if checkValue:
            self.joystickGroupBox.setEnabled(False)
            self.feedbackGroupBox.setEnabled(False)
            self.pluginTab.setEnabled(False)
            msg = '%s enabled'%(self.startupMode,)
            self.writeStatusMessage(msg)
            self.startPushButton.setEnabled(True)
            self.controlMode = 'startupMode'
        else:
            self.controlGroupBoxSetEnabled(True)
            self.pluginTab.setEnabled(True)
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
            self.startPosition = startPositionNew
            self.writeStatusMessage('start position set to %s m'%(self.startPositionStr,))
        self.startPositionLineEdit.setText(self.startPositionStr)

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
            self.updateAutorunDelayTimerInterval(self.autorunDelay)
            self.writeStatusMessage('autorun delay set to %d s'%(self.autorunDelay,))

    def joystickCheck_Callback(self,checkValue):
        """
        Callback for the checkbox of the joystickGroupBox. This is called when the user
        selects/deselects joystick control mode.
        """
        if checkValue:
            self.modeGroupBox.setEnabled(False)
            self.feedbackGroupBox.setEnabled(False)
            self.pluginTab.setEnabled(False)
            self.startPushButton.setEnabled(True)
            self.writeStatusMessage('joystick positioning enabled')
            self.controlMode = 'joystick' 
        else:
            self.controlGroupBoxSetEnabled(True)
            self.pluginTab.setEnabled(True)
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
            self.pluginTab.setEnabled(False)
            self.startPushButton.setEnabled(True)
            self.writeStatusMessage('feedback positioning enabled')
            self.controlMode = 'feedback'
        else:
            self.controlGroupBoxSetEnabled(True)
            self.pluginTab.setEnabled(True)
            self.startPushButton.setEnabled(False)
            self.writeStatusMessage('feedback positioning disabled')
            self.controlMode = None

    @property
    def feedbackPositionStr(self):
        return '%1.3f'%(self.feedbackPosition,)

    def feedbackPositionChanged_Callback(self):
        """
        Callback for changes to the feedback position line edit text.
        """
        feedbackPositionNew = self.feedbackPositionLineEdit.text()
        feedbackPositionNew = float(feedbackPositionNew)
        if feedbackPositionNew != self.feedbackPosition:
            self.feedbackPosition = feedbackPositionNew
            self.writeStatusMessage('feedback position set to %s'%(self.feedbackPositionStr,))
        self.feedbackPositionLineEdit.setText(self.feedbackPositionStr)

    def setFeedbackPositionValidator(self):
        """
        Set the validator for the feedback position line edit, based on the
        upper and lower bounds.
        """
        feedbackPositionValidator = QtGui.QDoubleValidator(self.feedbackPositionLineEdit)
        feedbackPositionValidator.setRange(self.lowerBound, self.upperBound, 3)
        feedbackPositionValidator.fixup = self.feedbackPositionFixup
        self.feedbackPositionLineEdit.setValidator(feedbackPositionValidator)

    def feedbackPositionFixup(self,value):
        """
        Fixup function for the feedback position line edit.
        """
        value = float(value)
        if value == self.lowerBound: 
            self.feedbackPosition = value
        self.feedbackPositionLineEdit.setText(self.feedbackPositionStr)


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
                self.setFeedbackPositionValidator()
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
                self.setFeedbackPositionValidator()
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
        self.runsTab.setEnabled(False)
        self.logTab.setEnabled(False)
        self.pluginTab.setEnabled(False)
        self.enableRobotControlMode()
        self.statusbar.showMessage('System Enabled - Running')
        
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
            self.runInProgress = False
            self.controlTab.setEnabled(True)
            self.logTab.setEnabled(True)
            self.runsTab.setEnabled(True)
            self.pluginTab.setEnabled(True)
            if not self.pluginRunning:
                self.startPushButton.setEnabled(True)
                self.stopPushButton.setEnabled(False)
                self.positionBoundsGroupBox.setEnabled(True)
                self.controlGroupBoxSetEnabled(True,enable_only_checked=True)
            else:
                self.pluginRunning = False
                self.pluginStartPushButton.setEnabled(True)
                self.pluginStopPushButton.setEnabled(False)

            # Stop robot actions
            self.disableRobotControlMode()
            self.robotControl.disableLogger()
            self.robotControl.disableDynamics()
            self.robotControl.setPwmToDefault()
            self.robotControl.disableLaserSetpt()

            self.statusbar.showMessage('System Enabled - Stopped')
            if self.logFileName is not None:
                self.populateLogTree()

    def disableRobotControlMode(self):
        """
        Disable current control mode via robotControl
        """
        if self.controlMode == 'joystick':
            self.robotControl.disableJoystickMode()
            self.writeStatusMessage('joystick positioning stopped')
        elif self.controlMode == 'feedback':
            self.robotControl.disableControllerMode()
            self.writeStatusMessage('feedback positioning stopped')
            self.progressBar.setVisible(False)
        elif self.controlMode == 'startupMode':
            self.robotControl.stopSetptOutscan()
            self.robotControl.stopActuatorOutscan()
            self.robotControl.disableControllerMode()
            self.robotControl.disableDynamics()
            self.robotControl.disableLaserSetpt()
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
            self.runInProgress = True
            with self.lock:
                self.outscanInProgress = False
            self.startFeedbackPositioning()
        elif self.controlMode == 'startupMode':
            self.writeStatusMessage('%s started'%(self.startupMode,))
            self.runInProgress = True
            with self.lock:
                self.outscanInProgress = False
            self.runState = 'move to start'
            self.startNextOutscan()

    def startFeedbackPositioning(self):
        """
        Start feedback positioning move.
        """
        self.progressBar.setVisible(True)
        setptValues = run_defs.get_ramp( 
                self.robotControl.position,
                self.feedbackPosition,
                FEEDBACK_POSITIONING_VELOCITY,
                FEEDBACK_POSITIONING_ACCELERATION,
                self.robotControl.dt,
                )
        #pylab.plot(setptValues)
        self.startOutscan(setptValues,'setpt')

    def updateTrialState(self):
        """
        Update the trial state of the system. 
        """
        if self.runState == 'move to start':
            self.runState = 'autorun delay';
        elif self.runState == 'autorun delay':
            self.runState = 'trial outscan'
        elif self.runState == 'trial outscan': 
            maxRunNumber = self.runFileReader.number_of_runs - 1
            if not self.autorun or self.runNumber >= maxRunNumber:
                self.runState = 'done'
            else:
                self.runState = 'move to start'
                self.setRunNumber(self.runNumber+1)
        else:
            raise ValueError, 'uknown runState %s'%(self.runState,)

    def startNextOutscan(self):
        """
        Starts the next outscan based on the current runState of the 
        system.
        """
        if self.runState == 'move to start':
            self.startMoveToStartOutscan()
        elif self.runState == 'autorun delay':
            self.startAutorunDelay()
        elif self.runState == 'trial outscan':
            self.startTrialOutscan()
        else:
            raise ValueError, 'unknown runState %s'%(self.runState,)
            
    def startMoveToStartOutscan(self):
        """
        Starts the move to starting position outscan
        """
        # Move to starting position
        self.progressBar.setValue(0)
        self.progressBar.setVisible(True)
        self.writeStatusMessage('moving to start position %s m'%(self.startPositionStr,))
        setptValues = run_defs.get_ramp(
                self.robotControl.position,
                self.startPosition,
                FEEDBACK_POSITIONING_VELOCITY,
                FEEDBACK_POSITIONING_ACCELERATION,
                self.robotControl.dt,
                )
        self.startOutscan(setptValues, 'setpt')

    def startAutorunDelay(self):
        """
        Starts the autorun delay timer.
        """
        self.autorunDelayCount = 0
        self.progressBar.setValue(0)
        self.progressBar.setVisible(True)

        self.writeStatusMessage('starting delay of %d s'%(self.autorunDelay,))
        with self.lock:
            self.outscanInProgress = True
        self.autorunDelayTimer.start()

    def startTrialOutscan(self):
        """
        Starts an outscan of the curent run number form the currently loaded 
        run file.
        """
        # Setup progress bar 
        self.progressBar.setValue(0)
        self.progressBar.setVisible(True)

        # Load run from file
        self.writeStatusMessage('loading run # %d for outscan'%(self.runNumber,))
        run = self.runFileReader.get_run(self.runNumber)
        runConverter = run_converter.Run_Converter(self.startupMode,self.robotControl.dt)
        # Start outscan - should I use actual position or start position?
        values = runConverter.get(run,self.robotControl.position)
        self.robotControl.enableLogger()
        if self.startupMode == 'position trajectory':
            self.startOutscan(values, 'setpt')
        elif self.startupMode == 'captive trajectory':
            # Need to set mass and dampling parameters 
            mass = run['mass'][0]
            damping = run['damping'][0]
            self.robotControl.setDynamicsParams(mass,damping)
            self.startOutscan(values, 'actuator')
        elif self.startupMode == 'inertial trajectory':
            pass
            # ---------------------------------------------------
            # Disabled to prevent the curious for breaking shit
            #self.startOutscan(values, 'actuator')
            # ----------------------------------------------------
        else:
            raise ValueError, 'unknown startup mode'

    def startOutscan(self,values,outscanType):
        """
        Starts outscan using the roboControl object.
        """
        # Start setpt outscan
        self.writeStatusMessage('running outscan')
        with self.lock:
            self.outscanInProgress = True
            self.outscanPercentComplete = 0
            
        if outscanType == 'setpt':
            outscanFunc = self.robotControl.startSetptOutscan
        elif outscanType == 'actuator':
            outscanFunc = self.robotControl.startActuatorOutscan
        else:
            raise ValueError, 'unknown outscan type'

        try:
            outscanFunc( 
                    values,
                    feedback_cb = self.outscanProgress_Callback,
                    done_cb = self.outscanDone_Callback,
                    )
        except ValueError, e:
            self.writeStatusMessage('error: %s'%(str(e),))
            with self.lock:
                self.outscanStopSignal = True
        
    def outscanProgress_Callback(self,data):
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
            self.outscanInProgress = False

        self.robotControl.disableControllerMode()
        self.robotControl.disableLogger()

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
            self.statusbar.showMessage('System Disabled')
        else:
            self.robotControl.enableSledIO()
            self.ioModeCheckTimer.start()
            self.enabled = True
            self.statusbar.showMessage('System Enabled - Stopped')
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
            self.pluginTab.setEnabled(True)
        else:
            self.writeStatusMessage('system disabled')
            self.enableDisablePushButton.setText('Enable')
            self.startPushButton.setEnabled(False)
            self.stopPushButton.setEnabled(False)
            self.controlGroupBoxSetEnabled(False)
            self.positionBoundsGroupBox.setEnabled(True)
            self.pluginTab.setEnabled(False)

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
                QtGui.QMessageBox.critical(self,'Error', 'run file load failed: %s'%(str(e),))
                return
            try:
                runFileMode = self.runFileReader.get_mode()
            except KeyError:
                self.cleanUpRunFile()
                msg = "run file load failed: can't read mode"
                self.writeStatusMessage(msg)
                QtGui.QMessageBox.critical(self,'Error',msg)
                return

            # File has beed successfully loaded check that mode is compatible
            if self.startupMode != runFileMode:
                self.cleanUpRunFile()
                msg = 'file incompatible: mode = %s'%(runFileMode,)
                self.writeStatusMessage(msg)
                QtGui.QMessageBox.critical(self,'Error',msg)
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
        if self.runFileReader:
            self.runFileReader.close()
        self.runFileReader = None
        self.runTreeWidget.clear()
        self.loadRunFilePushButton.setEnabled(True)
        self.clearRunFilePushButton.setEnabled(False)
        self.runFileLabel.setText('Run File:')
        self.runNumberLineEdit.setText('')

    def populateRunTree(self):
        """
        Populates the the runTreeWidget from the current hdf5 run file.
        """
        self.runTreeWidget.clear()
        self.runFileLabel.setText('Run File: %s'%(self.runFileReader.filename,))
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

    def runTreeItemRightClicked_Callback(self,event):
        """
        Plot run on right click of item in run tree
        """
        # Extract run number
        item = self.runTreeWidget.itemAt(event.pos())
        topLevelParent = self.getTreeWidgetTopLevelParent(item)
        indexObj = self.runTreeWidget.indexFromItem(topLevelParent)
        runNumber = indexObj.row()

        # Load run data and create time array
        run = self.runFileReader.get_run(runNumber)
        runConverter = run_converter.Run_Converter(self.startupMode,self.robotControl.dt)
        values = runConverter.get(run,self.startPosition)
        numPts = values.shape[0]
        t = numpy.arange(0,numPts)*self.robotControl.dt

        # Plot run data
        pylab.clf()
        pylab.plot(t,values)
        pylab.title('Run %s'%(runNumber,))
        if self.startupMode == 'position trajectory':
            pylab.xlabel('time (s)')
            pylab.ylabel('position (m)')
        elif self.startupMode == 'captive trajectory':
            pylab.xlabel('time (s)')
            pylab.ylabel('actuator value (us)')
        pylab.grid('on')
        pylab.show()

    def getTreeWidgetTopLevelParent(self,item):
        """
        Returns the top level parent of an item in a QTreeWidget.
        """
        while item.parent() is not None:
            item = item.parent()
        return item

    def setLogFile_Callback(self):
        filename = QtGui.QFileDialog.getSaveFileName(
                None,
                'Select log file',
                self.lastLogFileDir, 
                options=QtGui.QFileDialog.DontConfirmOverwrite
                )
        if filename:
            filename = str(filename)
            if os.path.isfile(filename):
                try:
                    f = h5py.File(filename,'r')
                except h5py.h5e.LowLevelIOError, e:
                    self.logFileName = None
                    QtGui.QMessageBox.critical(self,'Error', '%s'%(str(e),))
                    self.writeStatusMessage('unable to set log file: IO error')
                    return 

            status, message = self.robotControl.setLogFile(filename)
            self.logFileName = filename
            self.clearLogFilePushButton.setEnabled(True)
            self.setLogFilePushButton.setEnabled(False)
            self.logFileLabel.setText('Log File: %s'%(self.logFileName,))
            self.writeStatusMessage('log file set: %s'%(self.logFileName,))
            self.populateLogTree()

    def deleteLogItem_Callback(self):
        msgBoxTitle = 'Delete Log Item'
        currentItem = self.logTreeWidget.currentItem()
        if currentItem is not None:
            if currentItem.parent() is None:
                # Top level item - Ok to delete
                try:
                    f = h5py.File(self.logFileName)
                except h5py.h5e.LowLevelIOError, e:
                    msg = 'unable to delete log item: %s'%(str(e),)
                    QtGui.QMessageBox.critical(self,msgBoxtitle,msg)
                    return
                itemName = str(currentItem.text(0))
                del f[itemName]
                self.populateLogTree()
            else:
                # Note a top level item - dont't delete
                msg = 'unable to delete log item - select top level item'
                QtGui.QMessageBox.information(self, msgBoxTitle, msg) 
                return 
        else:
            msg = 'unable to delete log item - no item selected'
            QtGui.QMessageBox.information(self, msgBoxTitle, msg) 
            return

    def clearLogFile_Callback(self):
        self.logFileName = None
        self.logTreeWidget.clear()
        self.logFileLabel.setText('Log File:')
        self.writeStatusMessage('log file cleared')
        self.clearLogFilePushButton.setEnabled(False)
        self.deleteLogItemPushButton.setEnabled(False)
        self.setLogFilePushButton.setEnabled(True)

    def populateLogTree(self):
        self.logTreeWidget.clear()
        if os.path.isfile(self.logFileName):
            try:
                f = h5py.File(self.logFileName,'r')
            except h5py.h5e.LowLevelIOError, e:
                return
            logTopLevelList = list(f)
            logTopLevelList.sort(cmp=logFileCmpFunc)
            for name in logTopLevelList:
                treeItem = QtGui.QTreeWidgetItem(self.logTreeWidget,0)
                treeItem.setText(0,name)
                logData = f[name]
                self.addChildToLogTree(treeItem,logData)
        else:
            treeItem = QtGui.QTreeWidgetItem(self.logTreeWidget,0)
            treeItem.setText(0,'log file empty')

    def addChildToLogTree(self,parent,hdf5Item):
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
                self.addChildToLogTree(item, obj)

    def logTreeItemRightClicked_Callback(self,event):
        currentItem = self.logTreeWidget.currentItem()
        if currentItem is not None:
            fullName = self.getTreeWidgetItemFullName(currentItem)
            trialName = fullName.split('/')[1]
            timeName = '/{0}/data/time'.format(trialName)
            if fullName == timeName:
                return
            try:
                f = h5py.File(self.logFileName,'r')
            except h5py.h5e.LowLevelIOError, e:
                return
            try:
                dataValues = f[fullName]
            except KeyError, e:
                return
            try: 
                timeValues = f[timeName]
            except KeyError:
                return
            try:
                dataShape = dataValues.shape
                timeShape = timeValues.shape
            except AttributeError, e:
                return
           # if not dataShape[0] == timeShape[0]:
           #     return
            try:
                dataUnits = dataValues.attrs['unit']
                timeUnits = timeValues.attrs['unit']
            except AttributeError, e:
                return
            pylab.clf()
            for i in range(dataShape[1]): 
                n = min([timeValues.shape[0], dataValues.shape[0]])
                pylab.plot(timeValues[:n,0],dataValues[:n,i]) 
                pylab.xlabel(timeUnits)
                pylab.ylabel(dataUnits)
                pylab.title(fullName) 
            pylab.grid('on')
            pylab.show()


    def getTreeWidgetItemFullName(self,item):
        fullName = [str(item.text(0))]
        while item.parent() is not None:
            item = item.parent()
            fullName.append(str(item.text(0)))
        fullName.reverse()
        fullName.insert(0,'')
        fullName = '/'.join(fullName)
        return fullName
            
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

    def getPluginFiles(self):
        """
        Returns list of plugin files.
        """
        fileList = os.listdir(PLUGIN_DIRECTORY)
        pluginList = []
        for f in fileList:
            base, ext = os.path.splitext(f)
            if ext == '.py':
                pluginList.append(base)
        return pluginList

    def importPlugins(self):
        """
        Imports plugins
        """
        pluginDict = {}
        pluginFiles = self.getPluginFiles()
        if pluginFiles:
            sys.path.append(PLUGIN_DIRECTORY) 
            for f in pluginFiles:
                try:
                    pluginModule = __import__(f)
                    pluginDict[pluginModule.PLUGIN_NAME] = pluginModule
                except Exception, e:
                    errorMsg = 'unable to import plugin {0}: {1}'.format(f,str(e))
                    QtGui.QMessageBox.critical(self,'Plugin Import Error', errorMsg) 
        return pluginDict

    def main(self):
        self.show()
# -----------------------------------------------------------------------------

def sec2ms(x):
    return 1000*x

def logFileCmpFunc(x,y):
    """
    Comparison function for sorting the log file names.
    """
    xNum = int(x.split('_')[1])
    yNum = int(y.split('_')[1])
    if xNum > yNum:
        value = 1
    elif yNum > xNum:
        value = -1
    else:
        value = 0
    return value


# -----------------------------------------------------------------------------
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    startupMode = sys.argv[1]
    sledControl = SledControl_MainWindow(startupMode)
    sledControl.main()
    app.exec_()

