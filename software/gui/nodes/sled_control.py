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
from PyQt4 import QtCore
from PyQt4 import QtGui
from sled_control_ui import Ui_SledControl_MainWindow
from utilities import HDF5_Run_Reader
from utilities import RobotControl 
from gui_constants import *


# Messages
from msg_and_srv.msg import DistMsg

class SledControl_MainWindow(QtGui.QMainWindow,Ui_SledControl_MainWindow):

    def __init__(self,startupMode,parent=None):
        super(SledControl_MainWindow,self).__init__(parent)
        self.setupUi(self)
        self.initialize(startupMode)
        self.connectActions()
        self.subscribeToROSMessages()
        self.setupIOModeCheckTimer()

    def resizeEvent(self,event):
        super(SledControl_MainWindow,self).resizeEvent(event)

    def closeEvent(self,event):
        #######################################################################
        #
        # TODO ....
        # Put robot into know state ... e.g. disable sled io, current mode, etc.
        # prior to shutdown of gui. 
        # 
        ########################################################################
        self.disableRobotControlMode()
        self.ioModeCheckTimer.stop()

        rospy.signal_shutdown('gui closed')
        event.accept()

    def subscribeToROSMessages(self):
        """
        Subscribe to ROS messages
        """
        self.distSubscriber = rospy.Subscriber('distance', DistMsg, self.distMsg_Handler)

    def distMsg_Handler(self,data):
        """
        Handler for messages from the distance sensor. Displays the current position and 
        velocity in the labels of the right hand side of the GUI.
        """
        with self.lock:
            position = data.distance_kalman
            velocity = data.velocity_kalman
            # I'm getting some weird occasional crash ... try explicitly calling QString 
            positionText = QtCore.QString('Position:  {0:1.3f} (m)'.format(position))
            velocityText = QtCore.QString('Velocity: {0:+1.3f} (m/s)'.format(velocity))

        self.positionLabel.setText(positionText)
        self.velocityLabel.setText(velocityText)

    def setupIOModeCheckTimer(self):
        """
        Setup timer which is used to check the whether or not the sled io mode has changed
        - for example when the sled has gone out of bounds etc. This timer is started when
        the sled io is enabled and stoped when the sled io is stopped.
        """
        self.ioModeCheckTimer = QtCore.QTimer(self)
        self.ioModeCheckTimer.setInterval(IO_MODE_CHECK_TIMER_DT)
        self.ioModeCheckTimer.timeout.connect(self.ioModeCheckTimer_Callback)

    def ioModeCheckTimer_Callback(self,):
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

    def initialize(self,startupMode):
        """
        Initialize the state of the GUI and system.
        """
        
        self.lock = threading.Lock()
        self.enabled = False
        self.statusMessageCnt = 0
        self.startupMode = startupMode 
        self.controlMode = None
        self.runFileReader = None 
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

        # Set default autorun delay and max velocity
        self.autorunDelay = DEFAULT_AUTORUN_DELAY
        self.autorun = DEFAULT_AUTORUN_CHECK
        self.startPosition = DEFAULT_START_POSITION
        self.runNumber = None
        self.autorunDelayLineEdit.setText('%1.2f'%(DEFAULT_AUTORUN_DELAY,))
        self.autorunCheckBox.setChecked(DEFAULT_AUTORUN_CHECK)
        self.startPositionLineEdit.setText('%1.3f'%(DEFAULT_START_POSITION,))

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
            print 'autorun checked'
        else:
            print 'autorun unchecked'

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
        self.enableRobotControlMode()
        
    def stop_Callback(self):
        """
        Callback for when the stop button in the Control tab is clicked.
        """
        self.startPushButton.setEnabled(True)
        self.stopPushButton.setEnabled(False)
        self.positionBoundsGroupBox.setEnabled(True)
        self.controlGroupBoxSetEnabled(True,enable_only_checked=True)
        self.disableRobotControlMode()

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
            self.writeStatusMessage('%s stopped'%(self.startupMode,))
            self.progressBar.setVisible(False)

    def enableDisable_Callback(self):
        """
        Callback for when the enable/disable button is clicked.  Enables sled
        IO by putting the sled_io_node into motor command mode. Disables sled
        IO by puttin the seld_io_nod into off mode. 
        """
        if self.enabled:
            self.robotControl.disableSledIO()
            self.ioModeCheckTimer.stop()
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
    rospy.init_node('gui')
    app.exec_()

