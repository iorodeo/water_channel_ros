#!/usr/bin/env python
import roslib 
roslib.load_manifest('gui')
import rospy
import threading
import time
import sys
from PyQt4 import QtCore
from PyQt4 import QtGui
from sled_control_ui import Ui_SledControl_MainWindow
from utilities import HDF5_Run_Reader

# Messages
from msg_and_srv.msg import DistMsg

class SledControl_MainWindow(QtGui.QMainWindow,Ui_SledControl_MainWindow):

    def __init__(self,parent=None):
        super(SledControl_MainWindow,self).__init__(parent)
        self.setupUi(self)
        self.initialize()
        self.connectActions()

        self.subscribeToMessages()

    def subscribeToMessages(self):
        self.distSubscriber= rospy.Subscriber('distance', DistMsg, self.distMsg_Handler)

    def distMsg_Handler(self,data):
        position = data.distance_kalman
        velocity = data.velocity_kalman
        position = 1.0e-3*position
        velocity = 1.0e-3*velocity
        positionText = 'Position:  {0:1.3f} (m)'.format(position)
        velocityText = 'Velocity: {0:+1.3f} (m/s)'.format(velocity)
        self.positionLabel.setText(positionText)
        self.velocityLabel.setText(velocityText)

    def initialize(self):
        # Set text for mode group box - this will change based on startup mode 
        self.modeGroupBox.setTitle('Captive Trajectory Mode')

        # Set current tab to control tab - can this be done by name
        self.mainTabWidget.setCurrentWidget(self.controlTab)
        self.modeGroupBox.setChecked(False)
        self.joystickGroupBox.setChecked(False)
        self.feedbackGroupBox.setChecked(False)

        # Set background and text colors of status window
        bgColor = QtGui.QColor(0,0,0)
        textColor = QtGui.QColor(0,255,0)
        palette = self.statusWindowTextEdit.palette()
        palette.setColor(QtGui.QPalette.Active, QtGui.QPalette.Base, bgColor)
        palette.setColor(QtGui.QPalette.Active, QtGui.QPalette.Text, textColor)
        self.statusWindowTextEdit.setPalette(palette)
        self.statusWindowTextEdit.append('1 > initializing ... done')
        for i in range(0,200):
            self.statusWindowTextEdit.append('%d > do something'%(i,))


    def connectActions(self):

        # Main window actions
        self.enabledPushButton.clicked.connect(self.enabled_Callback)
        self.disabledPushButton.clicked.connect(self.disabled_Callback)

        # Actions for Controls tab
        self.modeGroupBox.clicked.connect(self.modeCheck_Callback)
        self.joystickGroupBox.clicked.connect(self.joystickCheck_Callback)
        self.feedbackGroupBox.clicked.connect(self.feedbackCheck_Callback)
        self.stopPushButton.clicked.connect(self.stop_Callback)

        # Actions for runs tab
        self.loadRunFilePushButton.clicked.connect(self.loadRunFile_Callback)
        self.startRunPushButton.clicked.connect(self.startRun_Callback)

        # Actions for log tab
        self.setLogFilePushButton.clicked.connect(self.setLogFile_Callback)
        self.deleteLogItemPushButton.clicked.connect(self.deleteLogItem_Callback)

        # Run tree actions
        self.runTreeWidget.itemClicked.connect(self.runTreeItemClicked_Callback)


    def modeCheck_Callback(self,checkValue):
        if checkValue:
            self.joystickGroupBox.setEnabled(False)
            self.feedbackGroupBox.setEnabled(False)
        else:
            self.controlGroupBoxEnableAll()

    def joystickCheck_Callback(self,checkValue):
        if checkValue:
            self.modeGroupBox.setEnabled(False)
            self.feedbackGroupBox.setEnabled(False)
        else:
            self.controlGroupBoxEnableAll()

    def feedbackCheck_Callback(self,checkValue):
        if checkValue:
            self.modeGroupBox.setEnabled(False)
            self.joystickGroupBox.setEnabled(False)
        else:
            self.controlGroupBoxEnableAll()
        
    def controlGroupBoxEnableAll(self):
        self.modeGroupBox.setEnabled(True)
        self.joystickGroupBox.setEnabled(True)
        self.feedbackGroupBox.setEnabled(True)

    def runTreeItemClicked_Callback(self,item):
        print 'run tree - item clicked'
        print item.text(0)


    def loadRunFile_Callback(self):
        print 'load run file'
        self.runTreeWidget.setEnabled(True)
        self.addItemsToRunTree()

    def startRun_Callback(self):
        print 'start run'
        self.runTreeWidget.setEnabled(False)

    def stop_Callback(self):
        print 'stop'

    def enabled_Callback(self):
        print 'enabled'
        self.runTreeWidget.setEnabled(True)

    def disabled_Callback(self):
        print 'disabled'
        print self.statusWindowTextEdit.height()
    

    def setLogFile_Callback(self):
        print 'set log file'

    def deleteLogItem_Callback(self):
        print 'delete log item'

    def addItemsToRunTree(self):
        topItem = QtGui.QTreeWidgetItem(self.runTreeWidget,0)
        topItem.setText(0,'Filename')
        for i in range(0,100):
            item = QtGui.QTreeWidgetItem(topItem,0)
            item.setText(0,'run %d'%(i,))

    def main(self):
        self.show()
        

# -----------------------------------------------------------------------------
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    sledControl = SledControl_MainWindow()
    sledControl.main()
    rospy.init_node('gui')
    app.exec_()

