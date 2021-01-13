#!/usr/bin/python
#

from PyQt5 import QtCore, QtGui, uic, QtWidgets

import main_ui

#import view.paper

clubs = {
    "Driver" : [6854, 10.9, 2686, .20, .09],  #     cd .20 cl_factor .09
    "3-wood" : [6485,  9.2, 3655, .25, .17],  #     cd .25 cl_factor .17
    "5-wood" : [6238,  9.4, 4350, .27, .19],  #     cd .27 cl_factor .19
    "3-iron" : [5828, 10.4, 4630, .27, .17],  #     cd .27 cl_factor .17
    "4-iron" : [5623, 11.0, 4836, .28, .19],  #     cd .28 cl_factor .19
    "5-iron" : [5417, 12.1, 5361, .31, .26],  #132  cd .31 cl_factor .26
    "6-iron" : [5212, 14.1, 6231, .32, .21],  #     cd .32 cl_factor .21
    "7-iron" : [4925, 16.3, 7097, .33, .22],  #120  cd .33 cl_factor .22 
    "8-iron" : [4720, 18.1, 7998, .34, .18],  #120  cd .34 cl_factor .18  
    "9-iron" : [4473, 20.4, 8647, .35, .17],  #109  cd .35 cl_factor .17
    "PWedge" : [4186, 24.2, 9304, .35, .13],  #102  cd .35 cl_factor .13
}

class MainInterface(QtWidgets.QMainWindow, main_ui.Ui_MainWindow):
    def __init__(self, parent, width, height, label):
        QtWidgets.QMainWindow.__init__(self)
       
        self.__parent = parent

        self.setupUi(self)
        
        self.setWindowTitle(label)

        
    def create_ui(self):
        self.pushButton.clicked.connect( \
            self.__parent.go)
        self.clearButton.clicked.connect( \
            self.__parent.clear)
        
        self.forceSpinBox.valueChanged.connect(self.updateForceMph)

        for club_name in clubs:
            data = clubs[club_name]

            self.clubComboBox.addItem(club_name, data)

        self.clubComboBox.currentIndexChanged.connect(self.updateSettings)
        self.clubComboBox.setCurrentIndex(1)

        self.create_menu()


    def create_menu(self):
        pass
       
        # self.file_quit.triggered.connect(self.__parent.quit_cb)
       

    def about_cb(self, event):
        reply = QtWidgets.QMessageBox.information(self, 'About', \
            "Trajectory\nby Dale Cieslak\n(c) 2020" + \
            "\n\nhttps://github.com/dsizzle/trajectory", \
            QtWidgets.QMessageBox.Ok)

    def mouseMoveEvent(self, event):
        self.__parent.mouse_event(event)

    def mousePressEvent(self, event):
        self.__parent.mouse_event(event)

    def mouseReleaseEvent(self, event):
        self.__parent.mouse_event(event)

    def wheelEvent(self, event):
        self.__parent.wheel_event(event)

    def keyReleaseEvent(self, event):
        self.__parent.key_event(event)

    def paintEvent(self, event):
        QtWidgets.QMainWindow.paintEvent(self, event)

    def closeEvent(self, event):
        close = self.__parent.quit_cb(event)

        if close:
            event.accept()
        else:
            event.ignore()

    def updateForceMph(self, value):
        ui = self.__parent.getUI()
        v0 = value * ui.frame.contact_t / ui.frame.mass
        self.mpsSpinBox.setValue(v0)
        self.mphSpinBox.setValue(v0 / .44704)

    def updateSettings(self, index):
        itemData = self.clubComboBox.currentData(QtCore.Qt.UserRole) 

        self.forceSpinBox.setValue(itemData[0])
        self.angleSpinBox.setValue(itemData[1])
        self.spinSpinBox.setValue(itemData[2])
        self.cdSpinBox.setValue(itemData[3])
        self.factorSpinBox.setValue(itemData[4])
