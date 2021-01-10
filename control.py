"""
edit_control.py

Contains EditorController class.
"""
import os

from PyQt5 import QtGui, QtCore, QtWidgets

#import view.edit_ui
#import view.paper
import ui

class MainController(object):
    """
    EditorController is the main Controller for pyTalic Editor
    """
    def __init__(self, w, h, label, script_path):
        self.__label = label
        self.__ui = ui.MainInterface(self, w, h, label)
 #       self.__ui = view.edit_ui.EditInterface(self, w, h, label)
        

        self.__ui.create_ui()

  #      self.__ui.dwg_area.bitmap_size = view.shared_qt.ICON_SIZE
        
        self.__ui.repaint()

    def activate(self):
        self.__ui.show()
        self.__ui.activateWindow()
        self.__ui.raise_()
  
    def mouse_event(self, event):
        event.accept()

    def wheel_event(self, event):
        event.accept()

    def key_event(self, event):
        event.accept()

    def paintEvent(self, event):
        QtWidgets.QMainWindow.paintEvent(self, event)

    def quit_cb(self, event):
        self.__ui.close()

        return True

    def getUI(self):
        return self.__ui

    def go(self, event):
        print("GO")
        print(self.__ui.angleSpinBox.value())

        self.__ui.frame.angle = self.__ui.angleSpinBox.value()
        self.__ui.frame.force = self.__ui.forceSpinBox.value()
        self.__ui.frame.spin = self.__ui.spinSpinBox.value()
        self.__ui.frame.gravity = self.__ui.gravitySpinBox.value()
        self.__ui.frame.cd = self.__ui.cdSpinBox.value()
        self.__ui.frame.liftFactor = self.__ui.factorSpinBox.value()
        self.__ui.frame.mu = self.__ui.muSpinBox.value()

        self.__ui.frame.clear = False
        self.__ui.frame.hit() #repaint() 

    def clear(self, event):
        self.__ui.frame.clear = True
        self.__ui.frame.repaint()