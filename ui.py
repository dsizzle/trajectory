#!/usr/bin/python
#

from PyQt5 import QtCore, QtGui, uic, QtWidgets

import main_ui

#import view.paper

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

        self.forceSpinBox.setValue(6854)
        self.create_menu()


    def create_menu(self):
        pass
       
        # self.file_quit.triggered.connect(self.__parent.quit_cb)
       

    def about_cb(self, event):
        reply = QtWidgets.QMessageBox.information(self, 'About', \
            "Trajectory\nby Dale Cieslak\n(c) 2020" + \
            "\n\nhttps://github.com/dsizzle/", \
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
