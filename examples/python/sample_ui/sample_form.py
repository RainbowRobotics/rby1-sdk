# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'sample_form.ui'
##
## Created by: Qt User Interface Compiler version 6.7.2
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QCheckBox, QComboBox, QDoubleSpinBox,
    QFrame, QGridLayout, QGroupBox, QHBoxLayout,
    QHeaderView, QLabel, QLineEdit, QPushButton,
    QSizePolicy, QSlider, QSpacerItem, QTabWidget,
    QTableView, QVBoxLayout, QWidget)

class Ui_MainForm(object):
    def setupUi(self, MainForm):
        if not MainForm.objectName():
            MainForm.setObjectName(u"MainForm")
        MainForm.resize(1700, 1100)
        self.horizontalLayout = QHBoxLayout(MainForm)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.label = QLabel(MainForm)
        self.label.setObjectName(u"label")
        sizePolicy = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)

        self.verticalLayout.addWidget(self.label)

        self.line = QFrame(MainForm)
        self.line.setObjectName(u"line")
        self.line.setLineWidth(1)
        self.line.setFrameShape(QFrame.Shape.VLine)
        self.line.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout.addWidget(self.line)

        self.LE_RobotConnection = QLineEdit(MainForm)
        self.LE_RobotConnection.setObjectName(u"LE_RobotConnection")
        self.LE_RobotConnection.setEnabled(True)
        sizePolicy1 = QSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.LE_RobotConnection.sizePolicy().hasHeightForWidth())
        self.LE_RobotConnection.setSizePolicy(sizePolicy1)
        self.LE_RobotConnection.setFocusPolicy(Qt.NoFocus)
        self.LE_RobotConnection.setReadOnly(True)

        self.verticalLayout.addWidget(self.LE_RobotConnection)

        self.LE_RobotAddress = QLineEdit(MainForm)
        self.LE_RobotAddress.setObjectName(u"LE_RobotAddress")
        sizePolicy1.setHeightForWidth(self.LE_RobotAddress.sizePolicy().hasHeightForWidth())
        self.LE_RobotAddress.setSizePolicy(sizePolicy1)

        self.verticalLayout.addWidget(self.LE_RobotAddress)

        self.BTN_RobotConnect = QPushButton(MainForm)
        self.BTN_RobotConnect.setObjectName(u"BTN_RobotConnect")
        sizePolicy2 = QSizePolicy(QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.BTN_RobotConnect.sizePolicy().hasHeightForWidth())
        self.BTN_RobotConnect.setSizePolicy(sizePolicy2)

        self.verticalLayout.addWidget(self.BTN_RobotConnect)

        self.groupBox = QGroupBox(MainForm)
        self.groupBox.setObjectName(u"groupBox")
        sizePolicy.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy)
        self.verticalLayout_4 = QVBoxLayout(self.groupBox)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.CB_PowerList = QComboBox(self.groupBox)
        self.CB_PowerList.setObjectName(u"CB_PowerList")

        self.verticalLayout_4.addWidget(self.CB_PowerList)

        self.horizontalLayout_2 = QHBoxLayout()
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.PB_PowerOn = QPushButton(self.groupBox)
        self.PB_PowerOn.setObjectName(u"PB_PowerOn")

        self.horizontalLayout_2.addWidget(self.PB_PowerOn)

        self.PB_PowerOff = QPushButton(self.groupBox)
        self.PB_PowerOff.setObjectName(u"PB_PowerOff")

        self.horizontalLayout_2.addWidget(self.PB_PowerOff)


        self.verticalLayout_4.addLayout(self.horizontalLayout_2)


        self.verticalLayout.addWidget(self.groupBox)

        self.groupBox_2 = QGroupBox(MainForm)
        self.groupBox_2.setObjectName(u"groupBox_2")
        sizePolicy.setHeightForWidth(self.groupBox_2.sizePolicy().hasHeightForWidth())
        self.groupBox_2.setSizePolicy(sizePolicy)
        self.verticalLayout_9 = QVBoxLayout(self.groupBox_2)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.CB_JointList = QComboBox(self.groupBox_2)
        self.CB_JointList.setObjectName(u"CB_JointList")

        self.verticalLayout_9.addWidget(self.CB_JointList)

        self.PB_JointServoOn = QPushButton(self.groupBox_2)
        self.PB_JointServoOn.setObjectName(u"PB_JointServoOn")

        self.verticalLayout_9.addWidget(self.PB_JointServoOn)

        self.horizontalLayout_3 = QHBoxLayout()
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.PB_BrakeEngage = QPushButton(self.groupBox_2)
        self.PB_BrakeEngage.setObjectName(u"PB_BrakeEngage")

        self.horizontalLayout_3.addWidget(self.PB_BrakeEngage)

        self.PB_BrakeRelease = QPushButton(self.groupBox_2)
        self.PB_BrakeRelease.setObjectName(u"PB_BrakeRelease")

        self.horizontalLayout_3.addWidget(self.PB_BrakeRelease)


        self.verticalLayout_9.addLayout(self.horizontalLayout_3)

        self.PB_ZPReset = QPushButton(self.groupBox_2)
        self.PB_ZPReset.setObjectName(u"PB_ZPReset")

        self.verticalLayout_9.addWidget(self.PB_ZPReset)

        self.line_5 = QFrame(self.groupBox_2)
        self.line_5.setObjectName(u"line_5")
        self.line_5.setFrameShape(QFrame.Shape.HLine)
        self.line_5.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_9.addWidget(self.line_5)

        self.label_15 = QLabel(self.groupBox_2)
        self.label_15.setObjectName(u"label_15")

        self.verticalLayout_9.addWidget(self.label_15)

        self.horizontalLayout_10 = QHBoxLayout()
        self.horizontalLayout_10.setObjectName(u"horizontalLayout_10")
        self.PB_JogNegative = QPushButton(self.groupBox_2)
        self.PB_JogNegative.setObjectName(u"PB_JogNegative")

        self.horizontalLayout_10.addWidget(self.PB_JogNegative)

        self.PB_JogPositive = QPushButton(self.groupBox_2)
        self.PB_JogPositive.setObjectName(u"PB_JogPositive")

        self.horizontalLayout_10.addWidget(self.PB_JogPositive)


        self.verticalLayout_9.addLayout(self.horizontalLayout_10)

        self.label_14 = QLabel(self.groupBox_2)
        self.label_14.setObjectName(u"label_14")

        self.verticalLayout_9.addWidget(self.label_14)

        self.horizontalLayout_11 = QHBoxLayout()
        self.horizontalLayout_11.setObjectName(u"horizontalLayout_11")
        self.SB_JogRelative = QDoubleSpinBox(self.groupBox_2)
        self.SB_JogRelative.setObjectName(u"SB_JogRelative")
        self.SB_JogRelative.setDecimals(3)
        self.SB_JogRelative.setMinimum(-88.000000000000000)
        self.SB_JogRelative.setMaximum(88.000000000000000)
        self.SB_JogRelative.setSingleStep(0.100000000000000)

        self.horizontalLayout_11.addWidget(self.SB_JogRelative)

        self.PB_JogRelative = QPushButton(self.groupBox_2)
        self.PB_JogRelative.setObjectName(u"PB_JogRelative")
        sizePolicy3 = QSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        sizePolicy3.setHorizontalStretch(0)
        sizePolicy3.setVerticalStretch(0)
        sizePolicy3.setHeightForWidth(self.PB_JogRelative.sizePolicy().hasHeightForWidth())
        self.PB_JogRelative.setSizePolicy(sizePolicy3)

        self.horizontalLayout_11.addWidget(self.PB_JogRelative)

        self.horizontalLayout_11.setStretch(0, 1)
        self.horizontalLayout_11.setStretch(1, 1)

        self.verticalLayout_9.addLayout(self.horizontalLayout_11)


        self.verticalLayout.addWidget(self.groupBox_2)

        self.groupBox_3 = QGroupBox(MainForm)
        self.groupBox_3.setObjectName(u"groupBox_3")
        sizePolicy.setHeightForWidth(self.groupBox_3.sizePolicy().hasHeightForWidth())
        self.groupBox_3.setSizePolicy(sizePolicy)
        self.verticalLayout_5 = QVBoxLayout(self.groupBox_3)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.LE_CMState = QLineEdit(self.groupBox_3)
        self.LE_CMState.setObjectName(u"LE_CMState")
        self.LE_CMState.setFocusPolicy(Qt.NoFocus)
        self.LE_CMState.setReadOnly(True)

        self.verticalLayout_5.addWidget(self.LE_CMState)

        self.PB_CMEnable = QPushButton(self.groupBox_3)
        self.PB_CMEnable.setObjectName(u"PB_CMEnable")

        self.verticalLayout_5.addWidget(self.PB_CMEnable)

        self.PB_CMDisable = QPushButton(self.groupBox_3)
        self.PB_CMDisable.setObjectName(u"PB_CMDisable")

        self.verticalLayout_5.addWidget(self.PB_CMDisable)

        self.PB_CMResetFault = QPushButton(self.groupBox_3)
        self.PB_CMResetFault.setObjectName(u"PB_CMResetFault")

        self.verticalLayout_5.addWidget(self.PB_CMResetFault)

        self.horizontalLayout_5 = QHBoxLayout()
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.label_6 = QLabel(self.groupBox_3)
        self.label_6.setObjectName(u"label_6")

        self.horizontalLayout_5.addWidget(self.label_6)

        self.LE_TimeScale = QLineEdit(self.groupBox_3)
        self.LE_TimeScale.setObjectName(u"LE_TimeScale")
        self.LE_TimeScale.setReadOnly(True)

        self.horizontalLayout_5.addWidget(self.LE_TimeScale)


        self.verticalLayout_5.addLayout(self.horizontalLayout_5)

        self.HS_TimeScale = QSlider(self.groupBox_3)
        self.HS_TimeScale.setObjectName(u"HS_TimeScale")
        self.HS_TimeScale.setMaximum(100)
        self.HS_TimeScale.setSingleStep(1)
        self.HS_TimeScale.setValue(80)
        self.HS_TimeScale.setOrientation(Qt.Horizontal)

        self.verticalLayout_5.addWidget(self.HS_TimeScale)


        self.verticalLayout.addWidget(self.groupBox_3)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)


        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.groupBox_4 = QGroupBox(MainForm)
        self.groupBox_4.setObjectName(u"groupBox_4")
        self.gridLayout = QGridLayout(self.groupBox_4)
        self.gridLayout.setObjectName(u"gridLayout")
        self.label_2 = QLabel(self.groupBox_4)
        self.label_2.setObjectName(u"label_2")

        self.gridLayout.addWidget(self.label_2, 0, 0, 1, 1)

        self.label_3 = QLabel(self.groupBox_4)
        self.label_3.setObjectName(u"label_3")

        self.gridLayout.addWidget(self.label_3, 1, 0, 1, 1)

        self.LE_BatteryCurrent = QLineEdit(self.groupBox_4)
        self.LE_BatteryCurrent.setObjectName(u"LE_BatteryCurrent")
        self.LE_BatteryCurrent.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_BatteryCurrent, 1, 1, 1, 1)

        self.LE_BatteryVoltage = QLineEdit(self.groupBox_4)
        self.LE_BatteryVoltage.setObjectName(u"LE_BatteryVoltage")
        self.LE_BatteryVoltage.setReadOnly(True)

        self.gridLayout.addWidget(self.LE_BatteryVoltage, 0, 1, 1, 1)


        self.verticalLayout_2.addWidget(self.groupBox_4)

        self.groupBox_5 = QGroupBox(MainForm)
        self.groupBox_5.setObjectName(u"groupBox_5")
        self.verticalLayout_6 = QVBoxLayout(self.groupBox_5)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.TV_PowerStates = QTableView(self.groupBox_5)
        self.TV_PowerStates.setObjectName(u"TV_PowerStates")

        self.verticalLayout_6.addWidget(self.TV_PowerStates)


        self.verticalLayout_2.addWidget(self.groupBox_5)

        self.horizontalLayout_12 = QHBoxLayout()
        self.horizontalLayout_12.setObjectName(u"horizontalLayout_12")
        self.label_13 = QLabel(MainForm)
        self.label_13.setObjectName(u"label_13")

        self.horizontalLayout_12.addWidget(self.label_13)

        self.LE_OdometryX = QLineEdit(MainForm)
        self.LE_OdometryX.setObjectName(u"LE_OdometryX")
        self.LE_OdometryX.setReadOnly(True)

        self.horizontalLayout_12.addWidget(self.LE_OdometryX)

        self.LE_OdometryY = QLineEdit(MainForm)
        self.LE_OdometryY.setObjectName(u"LE_OdometryY")
        self.LE_OdometryY.setReadOnly(True)

        self.horizontalLayout_12.addWidget(self.LE_OdometryY)

        self.LE_OdometryAngle = QLineEdit(MainForm)
        self.LE_OdometryAngle.setObjectName(u"LE_OdometryAngle")
        self.LE_OdometryAngle.setReadOnly(True)

        self.horizontalLayout_12.addWidget(self.LE_OdometryAngle)

        self.PB_ResetOdometry = QPushButton(MainForm)
        self.PB_ResetOdometry.setObjectName(u"PB_ResetOdometry")

        self.horizontalLayout_12.addWidget(self.PB_ResetOdometry)


        self.verticalLayout_2.addLayout(self.horizontalLayout_12)

        self.TV_JointStates = QTableView(MainForm)
        self.TV_JointStates.setObjectName(u"TV_JointStates")
        self.TV_JointStates.horizontalHeader().setMinimumSectionSize(0)

        self.verticalLayout_2.addWidget(self.TV_JointStates)

        self.verticalLayout_2.setStretch(0, 1)
        self.verticalLayout_2.setStretch(1, 1)
        self.verticalLayout_2.setStretch(3, 10)

        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.TW_RobotCommand = QTabWidget(MainForm)
        self.TW_RobotCommand.setObjectName(u"TW_RobotCommand")
        self.TB_WBC = QWidget()
        self.TB_WBC.setObjectName(u"TB_WBC")
        self.verticalLayout_8 = QVBoxLayout(self.TB_WBC)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.pushButton = QPushButton(self.TB_WBC)
        self.pushButton.setObjectName(u"pushButton")

        self.verticalLayout_8.addWidget(self.pushButton)

        self.TW_RobotCommand.addTab(self.TB_WBC, "")
        self.TB_CBC = QWidget()
        self.TB_CBC.setObjectName(u"TB_CBC")
        self.verticalLayout_7 = QVBoxLayout(self.TB_CBC)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.CB_EnableMobility = QCheckBox(self.TB_CBC)
        self.CB_EnableMobility.setObjectName(u"CB_EnableMobility")
        self.CB_EnableMobility.setChecked(False)

        self.verticalLayout_7.addWidget(self.CB_EnableMobility)

        self.tabWidget_2 = QTabWidget(self.TB_CBC)
        self.tabWidget_2.setObjectName(u"tabWidget_2")
        self.tabWidget_2.setEnabled(True)
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.tabWidget_2.addTab(self.tab_3, "")
        self.tab_4 = QWidget()
        self.tab_4.setObjectName(u"tab_4")
        self.tabWidget_2.addTab(self.tab_4, "")

        self.verticalLayout_7.addWidget(self.tabWidget_2)

        self.CB_EnableBody = QCheckBox(self.TB_CBC)
        self.CB_EnableBody.setObjectName(u"CB_EnableBody")
        self.CB_EnableBody.setChecked(False)

        self.verticalLayout_7.addWidget(self.CB_EnableBody)

        self.TB_CBCBody = QTabWidget(self.TB_CBC)
        self.TB_CBCBody.setObjectName(u"TB_CBCBody")
        self.TB_CBCBody.setEnabled(True)
        self.TB_CBCJP = QWidget()
        self.TB_CBCJP.setObjectName(u"TB_CBCJP")
        self.verticalLayout_10 = QVBoxLayout(self.TB_CBCJP)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.horizontalLayout_6 = QHBoxLayout()
        self.horizontalLayout_6.setObjectName(u"horizontalLayout_6")
        self.label_7 = QLabel(self.TB_CBCJP)
        self.label_7.setObjectName(u"label_7")

        self.horizontalLayout_6.addWidget(self.label_7)

        self.SB_JP_MiminumTime = QDoubleSpinBox(self.TB_CBCJP)
        self.SB_JP_MiminumTime.setObjectName(u"SB_JP_MiminumTime")
        self.SB_JP_MiminumTime.setSingleStep(0.010000000000000)
        self.SB_JP_MiminumTime.setValue(2.000000000000000)

        self.horizontalLayout_6.addWidget(self.SB_JP_MiminumTime)

        self.horizontalLayout_6.setStretch(0, 1)
        self.horizontalLayout_6.setStretch(1, 4)

        self.verticalLayout_10.addLayout(self.horizontalLayout_6)

        self.horizontalLayout_7 = QHBoxLayout()
        self.horizontalLayout_7.setObjectName(u"horizontalLayout_7")
        self.label_8 = QLabel(self.TB_CBCJP)
        self.label_8.setObjectName(u"label_8")

        self.horizontalLayout_7.addWidget(self.label_8)

        self.SB_JP_ControlHoldTime = QDoubleSpinBox(self.TB_CBCJP)
        self.SB_JP_ControlHoldTime.setObjectName(u"SB_JP_ControlHoldTime")
        self.SB_JP_ControlHoldTime.setSingleStep(0.010000000000000)

        self.horizontalLayout_7.addWidget(self.SB_JP_ControlHoldTime)

        self.horizontalLayout_7.setStretch(0, 1)
        self.horizontalLayout_7.setStretch(1, 4)

        self.verticalLayout_10.addLayout(self.horizontalLayout_7)

        self.line_3 = QFrame(self.TB_CBCJP)
        self.line_3.setObjectName(u"line_3")
        self.line_3.setFrameShape(QFrame.Shape.HLine)
        self.line_3.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_10.addWidget(self.line_3)

        self.horizontalLayout_8 = QHBoxLayout()
        self.horizontalLayout_8.setObjectName(u"horizontalLayout_8")
        self.verticalLayout_11 = QVBoxLayout()
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.label_9 = QLabel(self.TB_CBCJP)
        self.label_9.setObjectName(u"label_9")

        self.verticalLayout_11.addWidget(self.label_9)

        self.TV_JPTargetPosition = QTableView(self.TB_CBCJP)
        self.TV_JPTargetPosition.setObjectName(u"TV_JPTargetPosition")
        self.TV_JPTargetPosition.horizontalHeader().setMinimumSectionSize(0)

        self.verticalLayout_11.addWidget(self.TV_JPTargetPosition)

        self.PB_Reflect = QPushButton(self.TB_CBCJP)
        self.PB_Reflect.setObjectName(u"PB_Reflect")

        self.verticalLayout_11.addWidget(self.PB_Reflect)


        self.horizontalLayout_8.addLayout(self.verticalLayout_11)

        self.verticalLayout_12 = QVBoxLayout()
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.label_10 = QLabel(self.TB_CBCJP)
        self.label_10.setObjectName(u"label_10")

        self.verticalLayout_12.addWidget(self.label_10)

        self.TV_JPVelcityLimit = QTableView(self.TB_CBCJP)
        self.TV_JPVelcityLimit.setObjectName(u"TV_JPVelcityLimit")
        self.TV_JPVelcityLimit.horizontalHeader().setMinimumSectionSize(0)

        self.verticalLayout_12.addWidget(self.TV_JPVelcityLimit)


        self.horizontalLayout_8.addLayout(self.verticalLayout_12)

        self.verticalLayout_13 = QVBoxLayout()
        self.verticalLayout_13.setObjectName(u"verticalLayout_13")
        self.label_11 = QLabel(self.TB_CBCJP)
        self.label_11.setObjectName(u"label_11")

        self.verticalLayout_13.addWidget(self.label_11)

        self.TV_JPAccelerationLimit = QTableView(self.TB_CBCJP)
        self.TV_JPAccelerationLimit.setObjectName(u"TV_JPAccelerationLimit")
        self.TV_JPAccelerationLimit.horizontalHeader().setMinimumSectionSize(0)

        self.verticalLayout_13.addWidget(self.TV_JPAccelerationLimit)


        self.horizontalLayout_8.addLayout(self.verticalLayout_13)


        self.verticalLayout_10.addLayout(self.horizontalLayout_8)

        self.TB_CBCBody.addTab(self.TB_CBCJP, "")
        self.TB_CBCGC = QWidget()
        self.TB_CBCGC.setObjectName(u"TB_CBCGC")
        self.verticalLayout_14 = QVBoxLayout(self.TB_CBCGC)
        self.verticalLayout_14.setObjectName(u"verticalLayout_14")
        self.horizontalLayout_9 = QHBoxLayout()
        self.horizontalLayout_9.setObjectName(u"horizontalLayout_9")
        self.label_12 = QLabel(self.TB_CBCGC)
        self.label_12.setObjectName(u"label_12")

        self.horizontalLayout_9.addWidget(self.label_12)

        self.doubleSpinBox_2 = QDoubleSpinBox(self.TB_CBCGC)
        self.doubleSpinBox_2.setObjectName(u"doubleSpinBox_2")
        self.doubleSpinBox_2.setSingleStep(0.010000000000000)

        self.horizontalLayout_9.addWidget(self.doubleSpinBox_2)


        self.verticalLayout_14.addLayout(self.horizontalLayout_9)

        self.line_4 = QFrame(self.TB_CBCGC)
        self.line_4.setObjectName(u"line_4")
        self.line_4.setFrameShape(QFrame.Shape.HLine)
        self.line_4.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_14.addWidget(self.line_4)

        self.checkBox_4 = QCheckBox(self.TB_CBCGC)
        self.checkBox_4.setObjectName(u"checkBox_4")

        self.verticalLayout_14.addWidget(self.checkBox_4)

        self.verticalSpacer_2 = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout_14.addItem(self.verticalSpacer_2)

        self.TB_CBCBody.addTab(self.TB_CBCGC, "")

        self.verticalLayout_7.addWidget(self.TB_CBCBody)

        self.CB_EnableHead = QCheckBox(self.TB_CBC)
        self.CB_EnableHead.setObjectName(u"CB_EnableHead")

        self.verticalLayout_7.addWidget(self.CB_EnableHead)

        self.tabWidget_4 = QTabWidget(self.TB_CBC)
        self.tabWidget_4.setObjectName(u"tabWidget_4")
        self.tab_7 = QWidget()
        self.tab_7.setObjectName(u"tab_7")
        self.tabWidget_4.addTab(self.tab_7, "")

        self.verticalLayout_7.addWidget(self.tabWidget_4)

        self.line_2 = QFrame(self.TB_CBC)
        self.line_2.setObjectName(u"line_2")
        self.line_2.setFrameShape(QFrame.Shape.VLine)
        self.line_2.setFrameShadow(QFrame.Shadow.Sunken)

        self.verticalLayout_7.addWidget(self.line_2)

        self.TW_RobotCommand.addTab(self.TB_CBC, "")

        self.verticalLayout_3.addWidget(self.TW_RobotCommand)

        self.horizontalLayout_4 = QHBoxLayout()
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.label_4 = QLabel(MainForm)
        self.label_4.setObjectName(u"label_4")

        self.horizontalLayout_4.addWidget(self.label_4)

        self.DSB_StreamingRate = QDoubleSpinBox(MainForm)
        self.DSB_StreamingRate.setObjectName(u"DSB_StreamingRate")
        self.DSB_StreamingRate.setMinimum(1.000000000000000)
        self.DSB_StreamingRate.setMaximum(100.000000000000000)
        self.DSB_StreamingRate.setValue(10.000000000000000)

        self.horizontalLayout_4.addWidget(self.DSB_StreamingRate)

        self.label_5 = QLabel(MainForm)
        self.label_5.setObjectName(u"label_5")

        self.horizontalLayout_4.addWidget(self.label_5)

        self.PB_StreamingStart = QPushButton(MainForm)
        self.PB_StreamingStart.setObjectName(u"PB_StreamingStart")

        self.horizontalLayout_4.addWidget(self.PB_StreamingStart)

        self.Pb_StreamingStop = QPushButton(MainForm)
        self.Pb_StreamingStop.setObjectName(u"Pb_StreamingStop")

        self.horizontalLayout_4.addWidget(self.Pb_StreamingStop)


        self.verticalLayout_3.addLayout(self.horizontalLayout_4)

        self.PB_SendOnce = QPushButton(MainForm)
        self.PB_SendOnce.setObjectName(u"PB_SendOnce")

        self.verticalLayout_3.addWidget(self.PB_SendOnce)


        self.horizontalLayout.addLayout(self.verticalLayout_3)

        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 4)
        self.horizontalLayout.setStretch(2, 5)

        self.retranslateUi(MainForm)

        self.TW_RobotCommand.setCurrentIndex(1)
        self.tabWidget_2.setCurrentIndex(1)
        self.TB_CBCBody.setCurrentIndex(0)
        self.tabWidget_4.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(MainForm)
    # setupUi

    def retranslateUi(self, MainForm):
        MainForm.setWindowTitle(QCoreApplication.translate("MainForm", u"Sample GUI", None))
        self.label.setText(QCoreApplication.translate("MainForm", u"Settings", None))
        self.LE_RobotAddress.setText(QCoreApplication.translate("MainForm", u"localhost:50051", None))
        self.BTN_RobotConnect.setText(QCoreApplication.translate("MainForm", u"Connect", None))
        self.groupBox.setTitle(QCoreApplication.translate("MainForm", u"Power", None))
        self.PB_PowerOn.setText(QCoreApplication.translate("MainForm", u"ON", None))
        self.PB_PowerOff.setText(QCoreApplication.translate("MainForm", u"OFF", None))
        self.groupBox_2.setTitle(QCoreApplication.translate("MainForm", u"Joint", None))
        self.PB_JointServoOn.setText(QCoreApplication.translate("MainForm", u"Servo ON", None))
        self.PB_BrakeEngage.setText(QCoreApplication.translate("MainForm", u"Brake ENG", None))
        self.PB_BrakeRelease.setText(QCoreApplication.translate("MainForm", u"Brake REL", None))
        self.PB_ZPReset.setText(QCoreApplication.translate("MainForm", u"ZP RST", None))
        self.label_15.setText(QCoreApplication.translate("MainForm", u"One Step Jog Motion", None))
        self.PB_JogNegative.setText(QCoreApplication.translate("MainForm", u"-", None))
        self.PB_JogPositive.setText(QCoreApplication.translate("MainForm", u"+", None))
        self.label_14.setText(QCoreApplication.translate("MainForm", u"Relative Jog Motion(deg)", None))
        self.PB_JogRelative.setText(QCoreApplication.translate("MainForm", u"Move", None))
        self.groupBox_3.setTitle(QCoreApplication.translate("MainForm", u"Control Manager", None))
        self.PB_CMEnable.setText(QCoreApplication.translate("MainForm", u"Enable", None))
        self.PB_CMDisable.setText(QCoreApplication.translate("MainForm", u"Disable", None))
        self.PB_CMResetFault.setText(QCoreApplication.translate("MainForm", u"Reset Fault", None))
        self.label_6.setText(QCoreApplication.translate("MainForm", u"Time Scale", None))
        self.groupBox_4.setTitle(QCoreApplication.translate("MainForm", u"Battery", None))
        self.label_2.setText(QCoreApplication.translate("MainForm", u"Voltage", None))
        self.label_3.setText(QCoreApplication.translate("MainForm", u"Current", None))
        self.groupBox_5.setTitle(QCoreApplication.translate("MainForm", u"Power", None))
        self.label_13.setText(QCoreApplication.translate("MainForm", u"Odometry", None))
        self.PB_ResetOdometry.setText(QCoreApplication.translate("MainForm", u"Reset", None))
        self.pushButton.setText(QCoreApplication.translate("MainForm", u"Stop", None))
        self.TW_RobotCommand.setTabText(self.TW_RobotCommand.indexOf(self.TB_WBC), QCoreApplication.translate("MainForm", u"Whole Body Command", None))
        self.CB_EnableMobility.setText(QCoreApplication.translate("MainForm", u"Enable Mobility", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_3), QCoreApplication.translate("MainForm", u"Joint Velocity", None))
        self.tabWidget_2.setTabText(self.tabWidget_2.indexOf(self.tab_4), QCoreApplication.translate("MainForm", u"SE2 Velocity", None))
        self.CB_EnableBody.setText(QCoreApplication.translate("MainForm", u"Enable Body", None))
        self.label_7.setText(QCoreApplication.translate("MainForm", u"Minimum time", None))
        self.label_8.setText(QCoreApplication.translate("MainForm", u"Control hold time", None))
        self.label_9.setText(QCoreApplication.translate("MainForm", u"Target Position (deg)", None))
        self.PB_Reflect.setText(QCoreApplication.translate("MainForm", u"Reflect", None))
        self.label_10.setText(QCoreApplication.translate("MainForm", u"Velocity Limit", None))
        self.label_11.setText(QCoreApplication.translate("MainForm", u"Acceleration Limit", None))
        self.TB_CBCBody.setTabText(self.TB_CBCBody.indexOf(self.TB_CBCJP), QCoreApplication.translate("MainForm", u"Joint Position", None))
        self.label_12.setText(QCoreApplication.translate("MainForm", u"Control hold time", None))
        self.checkBox_4.setText(QCoreApplication.translate("MainForm", u"On", None))
        self.TB_CBCBody.setTabText(self.TB_CBCBody.indexOf(self.TB_CBCGC), QCoreApplication.translate("MainForm", u"Gravity Compensation", None))
        self.CB_EnableHead.setText(QCoreApplication.translate("MainForm", u"Enable Head", None))
        self.tabWidget_4.setTabText(self.tabWidget_4.indexOf(self.tab_7), QCoreApplication.translate("MainForm", u"Joint Position", None))
        self.TW_RobotCommand.setTabText(self.TW_RobotCommand.indexOf(self.TB_CBC), QCoreApplication.translate("MainForm", u"Component Based Command", None))
        self.label_4.setText(QCoreApplication.translate("MainForm", u"Streaming", None))
        self.label_5.setText(QCoreApplication.translate("MainForm", u"Hz", None))
        self.PB_StreamingStart.setText(QCoreApplication.translate("MainForm", u"Start", None))
        self.Pb_StreamingStop.setText(QCoreApplication.translate("MainForm", u"Stop", None))
        self.PB_SendOnce.setText(QCoreApplication.translate("MainForm", u"Send Once", None))
    # retranslateUi
