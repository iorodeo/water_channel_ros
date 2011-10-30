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

DEFAULT_AUTORUN_DELAY = 5.0
DEFAULT_JOYSTICK_MAX_VELOCITY = 1.0
STATUS_WINDOW_BACKGROUND_COLOR = (40,40,40)
STATUS_WINDOW_TEXT_COLOR = (255,140,0)

# Messages
from msg_and_srv.msg import DistMsg

class SledControl_MainWindow(QtGui.QMainWindow,Ui_SledControl_MainWindow):

    def __init__(self,parent=None):
        super(SledControl_MainWindow,self).__init__(parent)
        self.setupUi(self)
        self.initialize()
        self.connectActions()
        self.subscribeToROSMessages()

    def resizeEvent(self,event):
        super(SledControl_MainWindow,self).resizeEvent(event)

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
            position = 1.0e-3*position  # Convert position to meters
            velocity = 1.0e-3*velocity  # Convert velocity to meters per second

            # I'm getting some weird occasional crash ... try explicitly calling QString 
            positionText = QtCore.QString('Position:  {0:1.3f} (m)'.format(position))
            velocityText = QtCore.QString('Velocity: {0:+1.3f} (m/s)'.format(velocity))

        self.positionLabel.setText(positionText)
        self.velocityLabel.setText(velocityText)

    def initialize(self):
        """
        Initialize the state of the GUI and system.
        """
        self.lock = threading.Lock()
        self.enabled = False
        self.statusMessageCnt = 0
        self.startupMode = 'captive trajectory'
        self.controlMode = None
        self.runFileReader = None 

        self.writeStatusMessage('initializing')

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

        # Set default autorun delay and max velocity
        self.autorunDelayLineEdit.setText('%1.2f'%(DEFAULT_AUTORUN_DELAY,))
        self.joystickMaxVelocityLineEdit.setText('%1.2f'%(DEFAULT_JOYSTICK_MAX_VELOCITY,))

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

    def controlGroupBoxSetEnabled(self,value,uncheck_on_disable=True,enable_only_checked=False):
        """
        Enable/Disable the group boxes on the control tab. If uncheck_on_disable is True than
        all groupBoxes are unchecked when they are disabled. If enalbe_only_checked = True than
        only groupBoxes which are checked will be enabled.
        """
        if value and enable_only_checked and self.isControlGroupBoxChecked():
            if self.modeGroupBox.isChecked():
                self.modeGroupBox.setEnabled(True)
            if self.joystickGroupBox.isChecked():
                self.joystickGroupBox.setEnabled(True)
            if self.feedbackGroupBox.isChecked():
                self.feedbackGroupBox.setEnabled(True)
        else:
            # Only enable mode only runfile is loaded
            if value and self.runFileReader is not None: 
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
        print 'start run'
        self.stopPushButton.setEnabled(True)
        self.startPushButton.setEnabled(False)
        self.controlGroupBoxSetEnabled(False,uncheck_on_disable=False)

    def stop_Callback(self):
        """
        Callback for when the stop button in the Control tab is clicked.
        """
        print 'stop'
        self.startPushButton.setEnabled(True)
        self.stopPushButton.setEnabled(False)
        self.controlGroupBoxSetEnabled(True,enable_only_checked=True)

    def enableDisable_Callback(self):
        """
        Callback for when the enable/disable button is clicked.
        """
        if self.enabled:
            self.enabled = False
        else:
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

    def loadRunFile_Callback(self):
        """
        Callback for loading run files - if the run file is valid it will populate 
        the runFileTreeWidget from the data in the selected run file using the populateRunTree
        method.
        """
        print 'load run file'
        filename = QtGui.QFileDialog.getOpenFileName(None,'Select run file',self.lastRunFileDir)
        success = True
        if filename:
            filename = str(filename)
            try:
                self.runFileReader = HDF5_Run_Reader(filename)
            except h5py.h5e.LowLevelIOError, e:
                success = False
                
        if success:
            self.populateRunTree()
            self.loadRunFilePushButton.setEnabled(False)
            self.clearRunFilePushButton.setEnabled(True)
            pathname, filename = os.path.split(filename)
            self.writeStatusMessage('run file loaded: %s'%(filename,))
            if self.enabled:
                self.controlGroupBoxSetEnabled(True,enable_only_checked=True)
        else:
            self.runFileReader.close()
            self.runFileReader = None
            self.runTreeWidget.clear()
            self.loadRunFilePushButton.setEnabled(True)
            self.clearRunFilePushButton.setEnabled(False)
            self.statusbar.showMessage('')

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
        print 'clear run file'
        self.runFileReader.close()
        self.runFileReader = None
        self.runTreeWidget.clear()
        self.loadRunFilePushButton.setEnabled(True)
        self.clearRunFilePushButton.setEnabled(False)
        self.statusbar.showMessage('')
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

    def main(self):
        self.show()
        

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    sledControl = SledControl_MainWindow()
    sledControl.main()
    rospy.init_node('gui')
    app.exec_()

