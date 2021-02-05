#!/usr/bin/python
#

from PyQt5 import QtCore, QtGui, uic, QtWidgets

import main_ui

#import view.paper

#trajectoware: .205, .425  t/30
#                .201, .422 t/20

clubs = {
    # force, launch angle, spin, cd, factor, clubhead speed, aoa, loft, clubhead mass, clubhead e, 
    "Driver" : [6854, 10.9, 2686, .228, .667, 113, -1.3, 8.55, 205, .830],  # .197, .084],  #     275, 32      loft should be 10    
    "3-wood" : [6485,  9.2, 3655, .260, .524, 107, -2.9, 12.33, 218, .830],  # .247, .159],  #     243, 30   loft should be 15
    "5-wood" : [6238,  9.4, 4350, .272, .446, 103, -3.3, 15.3, 234.5, .830],  # .268, .187],  #     230, 31  loft should be 19
    "3-iron" : [5828, 10.4, 4630, .270, .372,  98, -3.1, 17.18, 222.5, .830],  # .270, .167],  #     212, 27
    "4-iron" : [5623, 11.0, 4836, .285, .411,  96, -3.4, 18.35, 212, .830],  # .285, .200],  #     203, 28
    "5-iron" : [5417, 12.1, 5361, .303, .439,  94, -3.7, 20.89, 211, .830],  # .306, .247],  #     194, 31
    "6-iron" : [5212, 14.1, 6231, .308, .339,  92, -4.1, 25.05, 228.5, .830],  # .313, .195],  #     183, 30
    "7-iron" : [4925, 16.3, 7097, .318, .331,  90, -4.3, 29.54, 237.0, .830],  # .325, .209],  #     172, 32 
    "8-iron" : [4720, 18.1, 7998, .328, .264,  87, -4.5, 35.07, 345.0, .830],  # .341, .187],  #     160, 31  
    "9-iron" : [4473, 20.4, 8647, .341, .250,  85, -4.7, 39.48, 450.2, .830],  # .352, .169],  #     148, 30
    "PWedge" : [4186, 24.2, 9304, .354, .257,  83, -5.0, 44.5, 750.0, .830],  # .347, .124],  #     136, 29
}

club_order = ["Driver", "3-wood", "5-wood", \
    "3-iron", "4-iron", "5-iron", "6-iron", "7-iron", "8-iron", "9-iron", \
    "PWedge"]

class MainInterface(QtWidgets.QMainWindow, main_ui.Ui_MainWindow):
    def __init__(self, parent, width, height, label):
        QtWidgets.QMainWindow.__init__(self)
       
        self.__parent = parent

        self.setupUi(self)
        
        self.setWindowTitle(label)

        
    def create_ui(self):
        self.pushButton.clicked.connect( \
            self.__parent.go)
        # self.clearButton.clicked.connect( \
        #     self.__parent.clear)
        
        self.forceSpinBox.valueChanged.connect(self.updateForceMph)

        for club_name in club_order:
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
        if hasattr(self.__parent, 'contact_t'):
            v0 = value * self.__parent.contact_t / self.__parent.mass
            self.mpsSpinBox.setValue(v0)
            self.mphSpinBox.setValue(v0 / .44704)

    def updateSettings(self, index):
        itemData = self.clubComboBox.currentData(QtCore.Qt.UserRole) 

        self.forceSpinBox.setValue(itemData[0])
        self.angleSpinBox.setValue(itemData[1])
        self.spinSpinBox.setValue(itemData[2])
        self.cdSpinBox.setValue(itemData[3])
        self.factorSpinBox.setValue(itemData[4])
        self.clubheadVelocitySpinBox.setValue(itemData[5])
        self.aoaSpinBox.setValue(itemData[6])
        self.loftSpinBox.setValue(itemData[7])
        self.clubheadMassSpinBox.setValue(itemData[8])
        self.clubheadESpinBox.setValue(itemData[9])

