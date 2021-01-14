import math

from PyQt5 import QtCore, QtGui, QtWidgets


class Canvas(QtWidgets.QFrame):
    def __init__(self, parent):
        QtWidgets.QFrame.__init__(self, parent)
        self.__bg_color = QtGui.QColor(120, 120, 120)
        self.__bg_brush = QtGui.QBrush(self.__bg_color, QtCore.Qt.SolidPattern)
        self.__pen = QtGui.QPen(QtGui.QColor(255, 128, 128), 2, QtCore.Qt.SolidLine)
        self.__pen2 = QtGui.QPen(QtGui.QColor(128, 128, 255), 2, QtCore.Qt.SolidLine)
        self.__grayPen = QtGui.QPen(QtGui.QColor(128, 128, 128), 1, QtCore.Qt.SolidLine)
        self.__grayPen2 = QtGui.QPen(QtGui.QColor(160, 160, 160), 1, QtCore.Qt.SolidLine)
        self.__parent = parent

        self.__paintPath = QtGui.QPainterPath()
        self.clear = False

    def setPath(self, path):
        botY = self.frameRect().bottom()
        self.__paintPath = QtGui.QPainterPath()
        self.__paintPath.moveTo(path[0].x(), botY-path[0].y())
        for point in path[1:]:
            #print(point)
            self.__paintPath.lineTo(point.x()*2, botY-point.y()*2)

        self.repaint()

    def resizeEvent(self, event):
        self.repaint()

    def paintEvent(self, event):
        dc = QtGui.QPainter()

        dc.begin(self)
        dc.setRenderHint(QtGui.QPainter.Antialiasing)

        dc.setBackground(self.__bg_brush)
        dc.eraseRect(self.frameRect())
        dc.save()
    
        botY = self.frameRect().bottom()
        topY = self.frameRect().top()

        dc.setPen(self.__grayPen)
        for i in range(0, 12):
            dc.drawLine(i*50, botY, i*50, topY)

        dc.setPen(self.__grayPen2)
        for i in range(0, 6):
            dc.drawLine(i*100, botY, i*100, topY)

        #dc.moveTo()
        dc.strokePath(self.__paintPath, self.__pen2)
    
        dc.restore()
        dc.end()

        QtWidgets.QFrame.paintEvent(self, event)
    
