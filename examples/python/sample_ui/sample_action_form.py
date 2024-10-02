# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'sample_action_form.ui'
##
## Created by: Qt User Interface Compiler version 6.7.3
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
from PySide6.QtWidgets import (QApplication, QDial, QHBoxLayout, QLabel,
    QLineEdit, QMainWindow, QMenuBar, QPushButton,
    QSizePolicy, QSlider, QSpacerItem, QStatusBar,
    QTabWidget, QVBoxLayout, QWidget)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 600)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        self.horizontalLayout = QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.verticalLayout = QVBoxLayout()
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.LE_State = QLineEdit(self.centralwidget)
        self.LE_State.setObjectName(u"LE_State")
        self.LE_State.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.LE_State.setReadOnly(True)

        self.verticalLayout.addWidget(self.LE_State)

        self.PB_Zero = QPushButton(self.centralwidget)
        self.PB_Zero.setObjectName(u"PB_Zero")

        self.verticalLayout.addWidget(self.PB_Zero)

        self.PB_Ready = QPushButton(self.centralwidget)
        self.PB_Ready.setObjectName(u"PB_Ready")

        self.verticalLayout.addWidget(self.PB_Ready)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)

        self.verticalLayout.addItem(self.verticalSpacer)


        self.horizontalLayout.addLayout(self.verticalLayout)

        self.verticalLayout_2 = QVBoxLayout()
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.tabWidget = QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.label = QLabel(self.tab)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(20, 10, 401, 31))
        self.PB_Tab1Start = QPushButton(self.tab)
        self.PB_Tab1Start.setObjectName(u"PB_Tab1Start")
        self.PB_Tab1Start.setGeometry(QRect(20, 80, 89, 25))
        self.PB_Tab1Stop = QPushButton(self.tab)
        self.PB_Tab1Stop.setObjectName(u"PB_Tab1Stop")
        self.PB_Tab1Stop.setEnabled(False)
        self.PB_Tab1Stop.setGeometry(QRect(150, 80, 89, 25))
        self.tabWidget.addTab(self.tab, "")
        self.tab_2 = QWidget()
        self.tab_2.setObjectName(u"tab_2")
        self.label_2 = QLabel(self.tab_2)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setGeometry(QRect(10, 10, 401, 31))
        self.PB_Tab2Start = QPushButton(self.tab_2)
        self.PB_Tab2Start.setObjectName(u"PB_Tab2Start")
        self.PB_Tab2Start.setGeometry(QRect(20, 80, 89, 25))
        self.PB_Tab2Stop = QPushButton(self.tab_2)
        self.PB_Tab2Stop.setObjectName(u"PB_Tab2Stop")
        self.PB_Tab2Stop.setEnabled(False)
        self.PB_Tab2Stop.setGeometry(QRect(150, 80, 89, 25))
        self.S_Tab2YAxis = QSlider(self.tab_2)
        self.S_Tab2YAxis.setObjectName(u"S_Tab2YAxis")
        self.S_Tab2YAxis.setGeometry(QRect(120, 250, 160, 16))
        self.S_Tab2YAxis.setMinimum(-50)
        self.S_Tab2YAxis.setMaximum(50)
        self.S_Tab2YAxis.setOrientation(Qt.Orientation.Horizontal)
        self.S_Tab2ZAxis = QSlider(self.tab_2)
        self.S_Tab2ZAxis.setObjectName(u"S_Tab2ZAxis")
        self.S_Tab2ZAxis.setGeometry(QRect(90, 200, 16, 160))
        self.S_Tab2ZAxis.setMinimum(-50)
        self.S_Tab2ZAxis.setMaximum(50)
        self.S_Tab2ZAxis.setOrientation(Qt.Orientation.Vertical)
        self.S_Tab2XAxis = QSlider(self.tab_2)
        self.S_Tab2XAxis.setObjectName(u"S_Tab2XAxis")
        self.S_Tab2XAxis.setGeometry(QRect(60, 380, 160, 16))
        self.S_Tab2XAxis.setMinimum(-50)
        self.S_Tab2XAxis.setMaximum(50)
        self.S_Tab2XAxis.setOrientation(Qt.Orientation.Horizontal)
        self.label_4 = QLabel(self.tab_2)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(60, 180, 41, 17))
        self.label_5 = QLabel(self.tab_2)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(120, 230, 41, 17))
        self.label_6 = QLabel(self.tab_2)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(60, 410, 41, 17))
        self.horizontalLayoutWidget = QWidget(self.tab_2)
        self.horizontalLayoutWidget.setObjectName(u"horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(330, 180, 221, 151))
        self.horizontalLayout_2 = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3 = QVBoxLayout()
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.D_Tab2Roll = QDial(self.horizontalLayoutWidget)
        self.D_Tab2Roll.setObjectName(u"D_Tab2Roll")
        self.D_Tab2Roll.setMinimum(-50)
        self.D_Tab2Roll.setMaximum(50)

        self.verticalLayout_3.addWidget(self.D_Tab2Roll)

        self.label_7 = QLabel(self.horizontalLayoutWidget)
        self.label_7.setObjectName(u"label_7")
        self.label_7.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_3.addWidget(self.label_7)

        self.LE_Tab2Roll = QLineEdit(self.horizontalLayoutWidget)
        self.LE_Tab2Roll.setObjectName(u"LE_Tab2Roll")

        self.verticalLayout_3.addWidget(self.LE_Tab2Roll)


        self.horizontalLayout_2.addLayout(self.verticalLayout_3)

        self.verticalLayout_4 = QVBoxLayout()
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.D_Tab2Yaw = QDial(self.horizontalLayoutWidget)
        self.D_Tab2Yaw.setObjectName(u"D_Tab2Yaw")
        self.D_Tab2Yaw.setMinimum(-50)
        self.D_Tab2Yaw.setMaximum(50)

        self.verticalLayout_4.addWidget(self.D_Tab2Yaw)

        self.label_8 = QLabel(self.horizontalLayoutWidget)
        self.label_8.setObjectName(u"label_8")
        self.label_8.setAlignment(Qt.AlignmentFlag.AlignCenter)

        self.verticalLayout_4.addWidget(self.label_8)

        self.LE_Tab2Yaw = QLineEdit(self.horizontalLayoutWidget)
        self.LE_Tab2Yaw.setObjectName(u"LE_Tab2Yaw")

        self.verticalLayout_4.addWidget(self.LE_Tab2Yaw)


        self.horizontalLayout_2.addLayout(self.verticalLayout_4)

        self.LE_Tab2ZAxis = QLineEdit(self.tab_2)
        self.LE_Tab2ZAxis.setObjectName(u"LE_Tab2ZAxis")
        self.LE_Tab2ZAxis.setGeometry(QRect(110, 170, 61, 25))
        self.LE_Tab2ZAxis.setReadOnly(True)
        self.LE_Tab2YAxis = QLineEdit(self.tab_2)
        self.LE_Tab2YAxis.setObjectName(u"LE_Tab2YAxis")
        self.LE_Tab2YAxis.setGeometry(QRect(170, 220, 61, 25))
        self.LE_Tab2YAxis.setReadOnly(True)
        self.LE_Tab2XAxis = QLineEdit(self.tab_2)
        self.LE_Tab2XAxis.setObjectName(u"LE_Tab2XAxis")
        self.LE_Tab2XAxis.setGeometry(QRect(110, 400, 61, 25))
        self.LE_Tab2XAxis.setReadOnly(True)
        self.PB_Tab2Reset = QPushButton(self.tab_2)
        self.PB_Tab2Reset.setObjectName(u"PB_Tab2Reset")
        self.PB_Tab2Reset.setGeometry(QRect(470, 400, 75, 24))
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.label_3 = QLabel(self.tab_3)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 10, 401, 31))
        self.PB_Tab3Start = QPushButton(self.tab_3)
        self.PB_Tab3Start.setObjectName(u"PB_Tab3Start")
        self.PB_Tab3Start.setGeometry(QRect(20, 80, 89, 25))
        self.PB_Tab3Stop = QPushButton(self.tab_3)
        self.PB_Tab3Stop.setObjectName(u"PB_Tab3Stop")
        self.PB_Tab3Stop.setEnabled(False)
        self.PB_Tab3Stop.setGeometry(QRect(150, 80, 89, 25))
        self.tabWidget.addTab(self.tab_3, "")

        self.verticalLayout_2.addWidget(self.tabWidget)


        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 4)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 800, 33))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(2)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"RBY1-A Sample", None))
        self.PB_Zero.setText(QCoreApplication.translate("MainWindow", u"Zero", None))
        self.PB_Ready.setText(QCoreApplication.translate("MainWindow", u"Ready", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Torso: COM, Right/Left : Gravity Compensation", None))
        self.PB_Tab1Start.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.PB_Tab1Stop.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("MainWindow", u"Tab 1", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Torso: Cartesian,\n"
"Right/Left Arm: Cartesian", None))
        self.PB_Tab2Start.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.PB_Tab2Stop.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Z-axis", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Y-axis", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"X-axis", None))
        self.label_7.setText(QCoreApplication.translate("MainWindow", u"Roll", None))
        self.label_8.setText(QCoreApplication.translate("MainWindow", u"Yaw", None))
        self.PB_Tab2Reset.setText(QCoreApplication.translate("MainWindow", u"Reset", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Tab 2", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Torso: Position Lock,\n"
"Right/Left Arm: Impedance Control", None))
        self.PB_Tab3Start.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.PB_Tab3Stop.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Tab 3", None))
    # retranslateUi

