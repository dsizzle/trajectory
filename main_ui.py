# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'main_ui.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(809, 564)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(1, 1, 552, 508))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_2.setSpacing(10)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.frame = Canvas(self.layoutWidget)
        self.frame.setMinimumSize(QtCore.QSize(550, 150))
        self.frame.setAutoFillBackground(False)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.frame.setObjectName("frame")
        self.verticalLayout_2.addWidget(self.frame)
        self.frame_2 = Canvas2(self.layoutWidget)
        self.frame_2.setMinimumSize(QtCore.QSize(550, 280))
        self.frame_2.setAutoFillBackground(False)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Sunken)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_2.addWidget(self.frame_2)
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setContentsMargins(-1, 5, -1, 5)
        self.gridLayout.setObjectName("gridLayout")
        self.distanceFieldLabel = QtWidgets.QLabel(self.layoutWidget)
        self.distanceFieldLabel.setObjectName("distanceFieldLabel")
        self.gridLayout.addWidget(self.distanceFieldLabel, 0, 0, 1, 1)
        self.heightFieldLabel = QtWidgets.QLabel(self.layoutWidget)
        self.heightFieldLabel.setObjectName("heightFieldLabel")
        self.gridLayout.addWidget(self.heightFieldLabel, 1, 0, 1, 1)
        self.heightField = QtWidgets.QLabel(self.layoutWidget)
        self.heightField.setMinimumSize(QtCore.QSize(0, 21))
        self.heightField.setObjectName("heightField")
        self.gridLayout.addWidget(self.heightField, 1, 1, 1, 1)
        self.distanceField = QtWidgets.QLabel(self.layoutWidget)
        self.distanceField.setMinimumSize(QtCore.QSize(0, 21))
        self.distanceField.setObjectName("distanceField")
        self.gridLayout.addWidget(self.distanceField, 0, 1, 1, 1)
        self.verticalLayout_2.addLayout(self.gridLayout)
        self.layoutWidget1 = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget1.setGeometry(QtCore.QRect(561, 3, 245, 503))
        self.layoutWidget1.setObjectName("layoutWidget1")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.layoutWidget1)
        self.verticalLayout.setObjectName("verticalLayout")
        self.clubComboBox = QtWidgets.QComboBox(self.layoutWidget1)
        self.clubComboBox.setObjectName("clubComboBox")
        self.verticalLayout.addWidget(self.clubComboBox)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label = QtWidgets.QLabel(self.layoutWidget1)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.angleSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.angleSpinBox.setEnabled(False)
        self.angleSpinBox.setMaximum(90.0)
        self.angleSpinBox.setProperty("value", 45.0)
        self.angleSpinBox.setObjectName("angleSpinBox")
        self.horizontalLayout.addWidget(self.angleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_2.addWidget(self.label_2)
        self.mphSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.mphSpinBox.setEnabled(True)
        self.mphSpinBox.setReadOnly(True)
        self.mphSpinBox.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.mphSpinBox.setMaximum(1000.0)
        self.mphSpinBox.setProperty("value", 1000.0)
        self.mphSpinBox.setObjectName("mphSpinBox")
        self.horizontalLayout_2.addWidget(self.mphSpinBox)
        self.mpsSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.mpsSpinBox.setEnabled(True)
        self.mpsSpinBox.setReadOnly(True)
        self.mpsSpinBox.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.mpsSpinBox.setMaximum(1000.0)
        self.mpsSpinBox.setProperty("value", 1000.0)
        self.mpsSpinBox.setObjectName("mpsSpinBox")
        self.horizontalLayout_2.addWidget(self.mpsSpinBox)
        self.forceSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.forceSpinBox.setEnabled(False)
        self.forceSpinBox.setMaximum(10000.0)
        self.forceSpinBox.setProperty("value", 4190.0)
        self.forceSpinBox.setObjectName("forceSpinBox")
        self.horizontalLayout_2.addWidget(self.forceSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_3.addWidget(self.label_3)
        self.spinSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.spinSpinBox.setEnabled(False)
        self.spinSpinBox.setDecimals(0)
        self.spinSpinBox.setMinimum(-10000.0)
        self.spinSpinBox.setMaximum(10000.0)
        self.spinSpinBox.setProperty("value", 2400.0)
        self.spinSpinBox.setObjectName("spinSpinBox")
        self.horizontalLayout_3.addWidget(self.spinSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_4 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_4.addWidget(self.label_4)
        self.gravitySpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.gravitySpinBox.setMinimum(-10.0)
        self.gravitySpinBox.setMaximum(10.0)
        self.gravitySpinBox.setProperty("value", -9.8)
        self.gravitySpinBox.setObjectName("gravitySpinBox")
        self.horizontalLayout_4.addWidget(self.gravitySpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_19 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_19.setObjectName("horizontalLayout_19")
        self.label_14 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_19.addWidget(self.label_14)
        self.clubheadVelocitySpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.clubheadVelocitySpinBox.setDecimals(2)
        self.clubheadVelocitySpinBox.setMinimum(0.0)
        self.clubheadVelocitySpinBox.setMaximum(120.0)
        self.clubheadVelocitySpinBox.setSingleStep(0.1)
        self.clubheadVelocitySpinBox.setProperty("value", 107.0)
        self.clubheadVelocitySpinBox.setObjectName("clubheadVelocitySpinBox")
        self.horizontalLayout_19.addWidget(self.clubheadVelocitySpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_19)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_5 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_5.addWidget(self.label_5)
        self.aoaSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.aoaSpinBox.setDecimals(2)
        self.aoaSpinBox.setMinimum(-90.0)
        self.aoaSpinBox.setMaximum(90.0)
        self.aoaSpinBox.setSingleStep(0.1)
        self.aoaSpinBox.setProperty("value", -2.9)
        self.aoaSpinBox.setObjectName("aoaSpinBox")
        self.horizontalLayout_5.addWidget(self.aoaSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_6 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_6.setObjectName("label_6")
        self.horizontalLayout_11.addWidget(self.label_6)
        self.loftSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.loftSpinBox.setDecimals(2)
        self.loftSpinBox.setMinimum(0.0)
        self.loftSpinBox.setMaximum(60.0)
        self.loftSpinBox.setSingleStep(0.1)
        self.loftSpinBox.setProperty("value", 15.0)
        self.loftSpinBox.setObjectName("loftSpinBox")
        self.horizontalLayout_11.addWidget(self.loftSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_11)
        self.horizontalLayout_14 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_14.setObjectName("horizontalLayout_14")
        self.label_9 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_14.addWidget(self.label_9)
        self.pathAngleSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.pathAngleSpinBox.setDecimals(2)
        self.pathAngleSpinBox.setMinimum(-45.0)
        self.pathAngleSpinBox.setMaximum(45.0)
        self.pathAngleSpinBox.setSingleStep(0.1)
        self.pathAngleSpinBox.setObjectName("pathAngleSpinBox")
        self.horizontalLayout_14.addWidget(self.pathAngleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_14)
        self.horizontalLayout_12 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_12.setObjectName("horizontalLayout_12")
        self.label_7 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_7.setObjectName("label_7")
        self.horizontalLayout_12.addWidget(self.label_7)
        self.faceAngleSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.faceAngleSpinBox.setDecimals(2)
        self.faceAngleSpinBox.setMinimum(-45.0)
        self.faceAngleSpinBox.setMaximum(45.0)
        self.faceAngleSpinBox.setSingleStep(0.1)
        self.faceAngleSpinBox.setObjectName("faceAngleSpinBox")
        self.horizontalLayout_12.addWidget(self.faceAngleSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_12)
        self.horizontalLayout_15 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_15.setObjectName("horizontalLayout_15")
        self.label_10 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_10.setObjectName("label_10")
        self.horizontalLayout_15.addWidget(self.label_10)
        self.clubheadMassSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.clubheadMassSpinBox.setDecimals(2)
        self.clubheadMassSpinBox.setMinimum(0.0)
        self.clubheadMassSpinBox.setMaximum(800.0)
        self.clubheadMassSpinBox.setSingleStep(0.1)
        self.clubheadMassSpinBox.setProperty("value", 200.0)
        self.clubheadMassSpinBox.setObjectName("clubheadMassSpinBox")
        self.horizontalLayout_15.addWidget(self.clubheadMassSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_15)
        self.horizontalLayout_18 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_18.setObjectName("horizontalLayout_18")
        self.label_13 = QtWidgets.QLabel(self.layoutWidget1)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_18.addWidget(self.label_13)
        self.clubheadESpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.clubheadESpinBox.setDecimals(2)
        self.clubheadESpinBox.setMinimum(0.0)
        self.clubheadESpinBox.setMaximum(1.0)
        self.clubheadESpinBox.setSingleStep(0.01)
        self.clubheadESpinBox.setProperty("value", 0.83)
        self.clubheadESpinBox.setObjectName("clubheadESpinBox")
        self.horizontalLayout_18.addWidget(self.clubheadESpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_18)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.cdSpinBoxLabel = QtWidgets.QLabel(self.layoutWidget1)
        self.cdSpinBoxLabel.setObjectName("cdSpinBoxLabel")
        self.horizontalLayout_6.addWidget(self.cdSpinBoxLabel)
        self.cdSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.cdSpinBox.setDecimals(3)
        self.cdSpinBox.setMaximum(1.0)
        self.cdSpinBox.setSingleStep(0.001)
        self.cdSpinBox.setProperty("value", 0.21)
        self.cdSpinBox.setObjectName("cdSpinBox")
        self.horizontalLayout_6.addWidget(self.cdSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.cdSpinBoxLabel_2 = QtWidgets.QLabel(self.layoutWidget1)
        self.cdSpinBoxLabel_2.setObjectName("cdSpinBoxLabel_2")
        self.horizontalLayout_7.addWidget(self.cdSpinBoxLabel_2)
        self.factorSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.factorSpinBox.setDecimals(5)
        self.factorSpinBox.setMaximum(1.0)
        self.factorSpinBox.setSingleStep(0.001)
        self.factorSpinBox.setProperty("value", 0.36)
        self.factorSpinBox.setObjectName("factorSpinBox")
        self.horizontalLayout_7.addWidget(self.factorSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_13 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_13.setObjectName("horizontalLayout_13")
        self.cdSpinBoxLabel_6 = QtWidgets.QLabel(self.layoutWidget1)
        self.cdSpinBoxLabel_6.setObjectName("cdSpinBoxLabel_6")
        self.horizontalLayout_13.addWidget(self.cdSpinBoxLabel_6)
        self.sideFactorSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.sideFactorSpinBox.setWrapping(True)
        self.sideFactorSpinBox.setDecimals(5)
        self.sideFactorSpinBox.setMaximum(1.0)
        self.sideFactorSpinBox.setSingleStep(0.001)
        self.sideFactorSpinBox.setProperty("value", 0.36)
        self.sideFactorSpinBox.setObjectName("sideFactorSpinBox")
        self.horizontalLayout_13.addWidget(self.sideFactorSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_13)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.cdSpinBoxLabel_3 = QtWidgets.QLabel(self.layoutWidget1)
        self.cdSpinBoxLabel_3.setObjectName("cdSpinBoxLabel_3")
        self.horizontalLayout_8.addWidget(self.cdSpinBoxLabel_3)
        self.muSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.muSpinBox.setMaximum(1.0)
        self.muSpinBox.setSingleStep(0.01)
        self.muSpinBox.setProperty("value", 0.24)
        self.muSpinBox.setObjectName("muSpinBox")
        self.horizontalLayout_8.addWidget(self.muSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.cdSpinBoxLabel_4 = QtWidgets.QLabel(self.layoutWidget1)
        self.cdSpinBoxLabel_4.setObjectName("cdSpinBoxLabel_4")
        self.horizontalLayout_9.addWidget(self.cdSpinBoxLabel_4)
        self.windSpeedSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.windSpeedSpinBox.setMaximum(30.0)
        self.windSpeedSpinBox.setSingleStep(0.5)
        self.windSpeedSpinBox.setProperty("value", 0.0)
        self.windSpeedSpinBox.setObjectName("windSpeedSpinBox")
        self.horizontalLayout_9.addWidget(self.windSpeedSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_9)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.cdSpinBoxLabel_5 = QtWidgets.QLabel(self.layoutWidget1)
        self.cdSpinBoxLabel_5.setObjectName("cdSpinBoxLabel_5")
        self.horizontalLayout_10.addWidget(self.cdSpinBoxLabel_5)
        self.windDirSpinBox = QtWidgets.QDoubleSpinBox(self.layoutWidget1)
        self.windDirSpinBox.setWrapping(True)
        self.windDirSpinBox.setDecimals(0)
        self.windDirSpinBox.setMaximum(359.0)
        self.windDirSpinBox.setSingleStep(1.0)
        self.windDirSpinBox.setProperty("value", 0.0)
        self.windDirSpinBox.setObjectName("windDirSpinBox")
        self.horizontalLayout_10.addWidget(self.windDirSpinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_10)
        self.pushButton = QtWidgets.QPushButton(self.layoutWidget1)
        self.pushButton.setObjectName("pushButton")
        self.verticalLayout.addWidget(self.pushButton)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 809, 15))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.distanceFieldLabel.setText(_translate("MainWindow", "Distance"))
        self.heightFieldLabel.setText(_translate("MainWindow", "Height"))
        self.heightField.setText(_translate("MainWindow", "Height"))
        self.distanceField.setText(_translate("MainWindow", "Distance"))
        self.label.setText(_translate("MainWindow", "Angle"))
        self.label_2.setText(_translate("MainWindow", "Force"))
        self.label_3.setText(_translate("MainWindow", "Spin"))
        self.label_4.setText(_translate("MainWindow", "Gravity"))
        self.label_14.setText(_translate("MainWindow", "Clubhead Velocity(mph)"))
        self.label_5.setText(_translate("MainWindow", "Angle of Attack"))
        self.label_6.setText(_translate("MainWindow", "Loft"))
        self.label_9.setText(_translate("MainWindow", "Path Angle"))
        self.label_7.setText(_translate("MainWindow", "Face Angle"))
        self.label_10.setText(_translate("MainWindow", "Clubhead Mass"))
        self.label_13.setText(_translate("MainWindow", "Clubhead e"))
        self.cdSpinBoxLabel.setText(_translate("MainWindow", "Cd"))
        self.cdSpinBoxLabel_2.setText(_translate("MainWindow", "Factor?"))
        self.cdSpinBoxLabel_6.setText(_translate("MainWindow", "Side Factor?"))
        self.cdSpinBoxLabel_3.setText(_translate("MainWindow", "Mu"))
        self.cdSpinBoxLabel_4.setText(_translate("MainWindow", "Wind Velocity"))
        self.cdSpinBoxLabel_5.setText(_translate("MainWindow", "Wind Direction"))
        self.pushButton.setText(_translate("MainWindow", "Go"))

from view import Canvas, Canvas2
