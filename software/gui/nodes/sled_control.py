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

# Messages
from msg_and_srv.msg import DistMsg

class SledControl_MainWindow(QtGui.QMainWindow,Ui_SledControl_MainWindow):

    def __init__(self,parent=None):
        super(SledControl_MainWindow,self).__init__(parent)
        self.setupUi(self)
        self.connectActions()
        self.mainTabWidget.setCurrentIndex(0)

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


    def connectActions(self):

        # Main window actions
        self.stopPushButton.clicked.connect(self.stop_Callback)
        self.enablePushButton.clicked.connect(self.enable_Callback)

        # Actions for run tab
        self.loadRunFilePushButton.clicked.connect(self.loadRunFile_Callback)
        self.startRunPushButton.clicked.connect(self.startRun_Callback)

        # Actions for log tab
        self.setLogFilePushButton.clicked.connect(self.setLogFile_Callback)
        self.deleteLogItemPushButton.clicked.connect(self.deleteLogItem_Callback)

        # Run tree actions
        self.runTreeWidget.itemClicked.connect(self.runTreeItemClicked_Callback)

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

    def enable_Callback(self):
        print 'enable'
        self.runTreeWidget.setEnabled(True)

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

