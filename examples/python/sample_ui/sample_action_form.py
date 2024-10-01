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
        self.pushButton_2 = QPushButton(self.tab_2)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(20, 80, 89, 25))
        self.pushButton_4 = QPushButton(self.tab_2)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(150, 80, 89, 25))
        self.horizontalSlider = QSlider(self.tab_2)
        self.horizontalSlider.setObjectName(u"horizontalSlider")
        self.horizontalSlider.setGeometry(QRect(90, 200, 160, 16))
        self.horizontalSlider.setOrientation(Qt.Orientation.Horizontal)
        self.verticalSlider = QSlider(self.tab_2)
        self.verticalSlider.setObjectName(u"verticalSlider")
        self.verticalSlider.setGeometry(QRect(60, 150, 16, 160))
        self.verticalSlider.setOrientation(Qt.Orientation.Vertical)
        self.horizontalSlider_2 = QSlider(self.tab_2)
        self.horizontalSlider_2.setObjectName(u"horizontalSlider_2")
        self.horizontalSlider_2.setGeometry(QRect(30, 330, 160, 16))
        self.horizontalSlider_2.setOrientation(Qt.Orientation.Horizontal)
        self.label_4 = QLabel(self.tab_2)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setGeometry(QRect(30, 130, 41, 17))
        self.label_5 = QLabel(self.tab_2)
        self.label_5.setObjectName(u"label_5")
        self.label_5.setGeometry(QRect(90, 180, 41, 17))
        self.label_6 = QLabel(self.tab_2)
        self.label_6.setObjectName(u"label_6")
        self.label_6.setGeometry(QRect(30, 350, 41, 17))
        self.dial = QDial(self.tab_2)
        self.dial.setObjectName(u"dial")
        self.dial.setGeometry(QRect(320, 170, 81, 81))
        self.dial_2 = QDial(self.tab_2)
        self.dial_2.setObjectName(u"dial_2")
        self.dial_2.setGeometry(QRect(420, 170, 81, 81))
        self.tabWidget.addTab(self.tab_2, "")
        self.tab_3 = QWidget()
        self.tab_3.setObjectName(u"tab_3")
        self.label_3 = QLabel(self.tab_3)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setGeometry(QRect(10, 10, 401, 31))
        self.tabWidget.addTab(self.tab_3, "")

        self.verticalLayout_2.addWidget(self.tabWidget)


        self.horizontalLayout.addLayout(self.verticalLayout_2)

        self.horizontalLayout.setStretch(0, 1)
        self.horizontalLayout.setStretch(1, 4)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 800, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        self.tabWidget.setCurrentIndex(1)


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
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"Start", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"Stop", None))
        self.label_4.setText(QCoreApplication.translate("MainWindow", u"Z-axis", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", u"Y-axis", None))
        self.label_6.setText(QCoreApplication.translate("MainWindow", u"X-axis", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_2), QCoreApplication.translate("MainWindow", u"Tab 2", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Torso: Position Lock,\n"
"Right/Left Arm: Impedance Control", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_3), QCoreApplication.translate("MainWindow", u"Tab 3", None))
    # retranslateUi

