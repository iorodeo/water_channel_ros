# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'sled_control.ui'
#
# Created: Sat Oct 15 17:35:29 2011
#      by: PyQt4 UI code generator 4.7.2
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_SledControl_MainWindow(object):
    def setupUi(self, SledControl_MainWindow):
        SledControl_MainWindow.setObjectName("SledControl_MainWindow")
        SledControl_MainWindow.resize(745, 559)
        self.centralwidget = QtGui.QWidget(SledControl_MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.verticalLayout_10 = QtGui.QVBoxLayout(self.centralwidget)
        self.verticalLayout_10.setObjectName("verticalLayout_10")
        self.widget_6 = QtGui.QWidget(self.centralwidget)
        self.widget_6.setObjectName("widget_6")
        self.horizontalLayout_7 = QtGui.QHBoxLayout(self.widget_6)
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.widget_7 = QtGui.QWidget(self.widget_6)
        self.widget_7.setObjectName("widget_7")
        self.verticalLayout_9 = QtGui.QVBoxLayout(self.widget_7)
        self.verticalLayout_9.setObjectName("verticalLayout_9")
        self.mainTabWidget = QtGui.QTabWidget(self.widget_7)
        self.mainTabWidget.setObjectName("mainTabWidget")
        self.controlTab = QtGui.QWidget()
        self.controlTab.setObjectName("controlTab")
        self.verticalLayout_6 = QtGui.QVBoxLayout(self.controlTab)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        spacerItem = QtGui.QSpacerItem(20, 5, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.verticalLayout_6.addItem(spacerItem)
        self.modeGroupBox = QtGui.QGroupBox(self.controlTab)
        self.modeGroupBox.setCheckable(True)
        self.modeGroupBox.setChecked(True)
        self.modeGroupBox.setObjectName("modeGroupBox")
        self.verticalLayout_4 = QtGui.QVBoxLayout(self.modeGroupBox)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.widget_13 = QtGui.QWidget(self.modeGroupBox)
        self.widget_13.setObjectName("widget_13")
        self.horizontalLayout_8 = QtGui.QHBoxLayout(self.widget_13)
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.delayLabel = QtGui.QLabel(self.widget_13)
        self.delayLabel.setObjectName("delayLabel")
        self.horizontalLayout_8.addWidget(self.delayLabel)
        self.autorunDelayLineEdit = QtGui.QLineEdit(self.widget_13)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.autorunDelayLineEdit.sizePolicy().hasHeightForWidth())
        self.autorunDelayLineEdit.setSizePolicy(sizePolicy)
        self.autorunDelayLineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.autorunDelayLineEdit.setObjectName("autorunDelayLineEdit")
        self.horizontalLayout_8.addWidget(self.autorunDelayLineEdit)
        self.label = QtGui.QLabel(self.widget_13)
        self.label.setObjectName("label")
        self.horizontalLayout_8.addWidget(self.label)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_8.addItem(spacerItem1)
        self.autorunCheckBox = QtGui.QCheckBox(self.widget_13)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.autorunCheckBox.sizePolicy().hasHeightForWidth())
        self.autorunCheckBox.setSizePolicy(sizePolicy)
        self.autorunCheckBox.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.autorunCheckBox.setObjectName("autorunCheckBox")
        self.horizontalLayout_8.addWidget(self.autorunCheckBox)
        self.verticalLayout_4.addWidget(self.widget_13)
        spacerItem2 = QtGui.QSpacerItem(20, 5, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem2)
        self.verticalLayout_6.addWidget(self.modeGroupBox)
        self.line_3 = QtGui.QFrame(self.controlTab)
        self.line_3.setFrameShape(QtGui.QFrame.HLine)
        self.line_3.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_3.setObjectName("line_3")
        self.verticalLayout_6.addWidget(self.line_3)
        self.joystickGroupBox = QtGui.QGroupBox(self.controlTab)
        self.joystickGroupBox.setCheckable(True)
        self.joystickGroupBox.setObjectName("joystickGroupBox")
        self.verticalLayout_7 = QtGui.QVBoxLayout(self.joystickGroupBox)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.widget = QtGui.QWidget(self.joystickGroupBox)
        self.widget.setObjectName("widget")
        self.horizontalLayout = QtGui.QHBoxLayout(self.widget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtGui.QLabel(self.widget)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.joystickMaxVelocityLineEdit = QtGui.QLineEdit(self.widget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joystickMaxVelocityLineEdit.sizePolicy().hasHeightForWidth())
        self.joystickMaxVelocityLineEdit.setSizePolicy(sizePolicy)
        self.joystickMaxVelocityLineEdit.setMaximumSize(QtCore.QSize(70, 16777215))
        self.joystickMaxVelocityLineEdit.setObjectName("joystickMaxVelocityLineEdit")
        self.horizontalLayout.addWidget(self.joystickMaxVelocityLineEdit)
        spacerItem3 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem3)
        self.verticalLayout_7.addWidget(self.widget)
        spacerItem4 = QtGui.QSpacerItem(20, 5, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.verticalLayout_7.addItem(spacerItem4)
        self.verticalLayout_6.addWidget(self.joystickGroupBox)
        self.line = QtGui.QFrame(self.controlTab)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName("line")
        self.verticalLayout_6.addWidget(self.line)
        self.feedbackGroupBox = QtGui.QGroupBox(self.controlTab)
        self.feedbackGroupBox.setCheckable(True)
        self.feedbackGroupBox.setObjectName("feedbackGroupBox")
        self.verticalLayout_8 = QtGui.QVBoxLayout(self.feedbackGroupBox)
        self.verticalLayout_8.setObjectName("verticalLayout_8")
        self.widget_4 = QtGui.QWidget(self.feedbackGroupBox)
        self.widget_4.setObjectName("widget_4")
        self.horizontalLayout_5 = QtGui.QHBoxLayout(self.widget_4)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_3 = QtGui.QLabel(self.widget_4)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_5.addWidget(self.label_3)
        self.joystickMaxVelocityLineEdit_2 = QtGui.QLineEdit(self.widget_4)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.joystickMaxVelocityLineEdit_2.sizePolicy().hasHeightForWidth())
        self.joystickMaxVelocityLineEdit_2.setSizePolicy(sizePolicy)
        self.joystickMaxVelocityLineEdit_2.setMaximumSize(QtCore.QSize(70, 16777215))
        self.joystickMaxVelocityLineEdit_2.setObjectName("joystickMaxVelocityLineEdit_2")
        self.horizontalLayout_5.addWidget(self.joystickMaxVelocityLineEdit_2)
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_5.addItem(spacerItem5)
        self.verticalLayout_8.addWidget(self.widget_4)
        self.verticalLayout_6.addWidget(self.feedbackGroupBox)
        spacerItem6 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.verticalLayout_6.addItem(spacerItem6)
        self.widget_3 = QtGui.QWidget(self.controlTab)
        self.widget_3.setObjectName("widget_3")
        self.horizontalLayout_2 = QtGui.QHBoxLayout(self.widget_3)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.startRunPushButton = QtGui.QPushButton(self.widget_3)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.startRunPushButton.sizePolicy().hasHeightForWidth())
        self.startRunPushButton.setSizePolicy(sizePolicy)
        self.startRunPushButton.setObjectName("startRunPushButton")
        self.horizontalLayout_2.addWidget(self.startRunPushButton)
        self.stopPushButton = QtGui.QPushButton(self.widget_3)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stopPushButton.sizePolicy().hasHeightForWidth())
        self.stopPushButton.setSizePolicy(sizePolicy)
        self.stopPushButton.setObjectName("stopPushButton")
        self.horizontalLayout_2.addWidget(self.stopPushButton)
        spacerItem7 = QtGui.QSpacerItem(130, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem7)
        self.verticalLayout_6.addWidget(self.widget_3)
        self.mainTabWidget.addTab(self.controlTab, "")
        self.runsTab = QtGui.QWidget()
        self.runsTab.setObjectName("runsTab")
        self.verticalLayout_3 = QtGui.QVBoxLayout(self.runsTab)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.runTreeWidget = QtGui.QTreeWidget(self.runsTab)
        self.runTreeWidget.setObjectName("runTreeWidget")
        self.runTreeWidget.headerItem().setText(0, "1")
        self.verticalLayout_3.addWidget(self.runTreeWidget)
        self.widget_5 = QtGui.QWidget(self.runsTab)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_5.sizePolicy().hasHeightForWidth())
        self.widget_5.setSizePolicy(sizePolicy)
        self.widget_5.setObjectName("widget_5")
        self.horizontalLayout_4 = QtGui.QHBoxLayout(self.widget_5)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.loadRunFilePushButton = QtGui.QPushButton(self.widget_5)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.loadRunFilePushButton.sizePolicy().hasHeightForWidth())
        self.loadRunFilePushButton.setSizePolicy(sizePolicy)
        self.loadRunFilePushButton.setObjectName("loadRunFilePushButton")
        self.horizontalLayout_4.addWidget(self.loadRunFilePushButton)
        self.pushButton = QtGui.QPushButton(self.widget_5)
        self.pushButton.setObjectName("pushButton")
        self.horizontalLayout_4.addWidget(self.pushButton)
        spacerItem8 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_4.addItem(spacerItem8)
        self.verticalLayout_3.addWidget(self.widget_5)
        self.mainTabWidget.addTab(self.runsTab, "")
        self.logTab = QtGui.QWidget()
        self.logTab.setObjectName("logTab")
        self.verticalLayout_2 = QtGui.QVBoxLayout(self.logTab)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.logTreeWidget = QtGui.QTreeWidget(self.logTab)
        self.logTreeWidget.setObjectName("logTreeWidget")
        self.logTreeWidget.headerItem().setText(0, "1")
        self.verticalLayout_2.addWidget(self.logTreeWidget)
        self.widget_10 = QtGui.QWidget(self.logTab)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_10.sizePolicy().hasHeightForWidth())
        self.widget_10.setSizePolicy(sizePolicy)
        self.widget_10.setObjectName("widget_10")
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.widget_10)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.setLogFilePushButton = QtGui.QPushButton(self.widget_10)
        self.setLogFilePushButton.setObjectName("setLogFilePushButton")
        self.horizontalLayout_3.addWidget(self.setLogFilePushButton)
        spacerItem9 = QtGui.QSpacerItem(15, 20, QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem9)
        self.deleteLogItemPushButton = QtGui.QPushButton(self.widget_10)
        self.deleteLogItemPushButton.setObjectName("deleteLogItemPushButton")
        self.horizontalLayout_3.addWidget(self.deleteLogItemPushButton)
        spacerItem10 = QtGui.QSpacerItem(15, 20, QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem10)
        self.editLogNotesPushButton = QtGui.QPushButton(self.widget_10)
        self.editLogNotesPushButton.setObjectName("editLogNotesPushButton")
        self.horizontalLayout_3.addWidget(self.editLogNotesPushButton)
        spacerItem11 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem11)
        self.verticalLayout_2.addWidget(self.widget_10)
        self.mainTabWidget.addTab(self.logTab, "")
        self.plotTab = QtGui.QWidget()
        self.plotTab.setObjectName("plotTab")
        self.mainTabWidget.addTab(self.plotTab, "")
        self.verticalLayout_9.addWidget(self.mainTabWidget)
        self.horizontalLayout_7.addWidget(self.widget_7)
        self.widget_2 = QtGui.QWidget(self.widget_6)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_2.sizePolicy().hasHeightForWidth())
        self.widget_2.setSizePolicy(sizePolicy)
        self.widget_2.setObjectName("widget_2")
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.widget_2)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        spacerItem12 = QtGui.QSpacerItem(20, 5, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.verticalLayout_5.addItem(spacerItem12)
        self.widget_11 = QtGui.QWidget(self.widget_2)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget_11.sizePolicy().hasHeightForWidth())
        self.widget_11.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(18)
        self.widget_11.setFont(font)
        self.widget_11.setObjectName("widget_11")
        self.verticalLayout = QtGui.QVBoxLayout(self.widget_11)
        self.verticalLayout.setObjectName("verticalLayout")
        self.positionLabel = QtGui.QLabel(self.widget_11)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.positionLabel.sizePolicy().hasHeightForWidth())
        self.positionLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Monospace")
        font.setPointSize(18)
        font.setWeight(75)
        font.setBold(True)
        self.positionLabel.setFont(font)
        self.positionLabel.setObjectName("positionLabel")
        self.verticalLayout.addWidget(self.positionLabel)
        self.velocityLabel = QtGui.QLabel(self.widget_11)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Fixed, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.velocityLabel.sizePolicy().hasHeightForWidth())
        self.velocityLabel.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily("Monospace")
        font.setPointSize(18)
        font.setWeight(75)
        font.setBold(True)
        self.velocityLabel.setFont(font)
        self.velocityLabel.setObjectName("velocityLabel")
        self.verticalLayout.addWidget(self.velocityLabel)
        self.verticalLayout_5.addWidget(self.widget_11)
        spacerItem13 = QtGui.QSpacerItem(20, 10, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.verticalLayout_5.addItem(spacerItem13)
        self.label_4 = QtGui.QLabel(self.widget_2)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_4.setFont(font)
        self.label_4.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_4.setObjectName("label_4")
        self.verticalLayout_5.addWidget(self.label_4)
        self.statusWindowTextEdit = QtGui.QTextEdit(self.widget_2)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.statusWindowTextEdit.sizePolicy().hasHeightForWidth())
        self.statusWindowTextEdit.setSizePolicy(sizePolicy)
        self.statusWindowTextEdit.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.statusWindowTextEdit.setReadOnly(True)
        self.statusWindowTextEdit.setObjectName("statusWindowTextEdit")
        self.verticalLayout_5.addWidget(self.statusWindowTextEdit)
        self.frame_6 = QtGui.QFrame(self.widget_2)
        self.frame_6.setFrameShape(QtGui.QFrame.StyledPanel)
        self.frame_6.setObjectName("frame_6")
        self.horizontalLayout_6 = QtGui.QHBoxLayout(self.frame_6)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        spacerItem14 = QtGui.QSpacerItem(216, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout_6.addItem(spacerItem14)
        self.enabledPushButton = QtGui.QPushButton(self.frame_6)
        self.enabledPushButton.setCheckable(True)
        self.enabledPushButton.setChecked(False)
        self.enabledPushButton.setDefault(False)
        self.enabledPushButton.setFlat(False)
        self.enabledPushButton.setObjectName("enabledPushButton")
        self.horizontalLayout_6.addWidget(self.enabledPushButton)
        self.disabledPushButton = QtGui.QPushButton(self.frame_6)
        self.disabledPushButton.setCheckable(True)
        self.disabledPushButton.setChecked(False)
        self.disabledPushButton.setDefault(False)
        self.disabledPushButton.setFlat(False)
        self.disabledPushButton.setObjectName("disabledPushButton")
        self.horizontalLayout_6.addWidget(self.disabledPushButton)
        self.verticalLayout_5.addWidget(self.frame_6)
        self.horizontalLayout_7.addWidget(self.widget_2)
        self.verticalLayout_10.addWidget(self.widget_6)
        SledControl_MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(SledControl_MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 745, 23))
        self.menubar.setObjectName("menubar")
        SledControl_MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtGui.QStatusBar(SledControl_MainWindow)
        self.statusbar.setObjectName("statusbar")
        SledControl_MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(SledControl_MainWindow)
        self.mainTabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(SledControl_MainWindow)

    def retranslateUi(self, SledControl_MainWindow):
        SledControl_MainWindow.setWindowTitle(QtGui.QApplication.translate("SledControl_MainWindow", "Water Channel - Sled Control", None, QtGui.QApplication.UnicodeUTF8))
        self.modeGroupBox.setTitle(QtGui.QApplication.translate("SledControl_MainWindow", "Captive Trajectory", None, QtGui.QApplication.UnicodeUTF8))
        self.delayLabel.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Delay ", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("SledControl_MainWindow", "(s)", None, QtGui.QApplication.UnicodeUTF8))
        self.autorunCheckBox.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Autorun", None, QtGui.QApplication.UnicodeUTF8))
        self.joystickGroupBox.setTitle(QtGui.QApplication.translate("SledControl_MainWindow", "Joystick Positioning", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Max Velocity", None, QtGui.QApplication.UnicodeUTF8))
        self.feedbackGroupBox.setTitle(QtGui.QApplication.translate("SledControl_MainWindow", "Feedback Positioning", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Position", None, QtGui.QApplication.UnicodeUTF8))
        self.startRunPushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Start", None, QtGui.QApplication.UnicodeUTF8))
        self.stopPushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "  Stop ", None, QtGui.QApplication.UnicodeUTF8))
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.controlTab), QtGui.QApplication.translate("SledControl_MainWindow", "Control", None, QtGui.QApplication.UnicodeUTF8))
        self.loadRunFilePushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Load ", None, QtGui.QApplication.UnicodeUTF8))
        self.pushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Remove", None, QtGui.QApplication.UnicodeUTF8))
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.runsTab), QtGui.QApplication.translate("SledControl_MainWindow", "Runs", None, QtGui.QApplication.UnicodeUTF8))
        self.setLogFilePushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Set Log ", None, QtGui.QApplication.UnicodeUTF8))
        self.deleteLogItemPushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Del Item", None, QtGui.QApplication.UnicodeUTF8))
        self.editLogNotesPushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Notes", None, QtGui.QApplication.UnicodeUTF8))
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.logTab), QtGui.QApplication.translate("SledControl_MainWindow", "Log", None, QtGui.QApplication.UnicodeUTF8))
        self.mainTabWidget.setTabText(self.mainTabWidget.indexOf(self.plotTab), QtGui.QApplication.translate("SledControl_MainWindow", "Plot", None, QtGui.QApplication.UnicodeUTF8))
        self.positionLabel.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Position: 14.234 (m)", None, QtGui.QApplication.UnicodeUTF8))
        self.velocityLabel.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Velocity: 1.234 (m/s)", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Status Window", None, QtGui.QApplication.UnicodeUTF8))
        self.enabledPushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Enabled", None, QtGui.QApplication.UnicodeUTF8))
        self.disabledPushButton.setText(QtGui.QApplication.translate("SledControl_MainWindow", "Disabled", None, QtGui.QApplication.UnicodeUTF8))

