# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'joystick2.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(820, 548)
        self.up = QtGui.QPushButton(Form)
        self.up.setGeometry(QtCore.QRect(200, 210, 31, 51))
        self.up.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.up.setText(_fromUtf8(""))
        self.up.setObjectName(_fromUtf8("up"))
        self.left = QtGui.QPushButton(Form)
        self.left.setGeometry(QtCore.QRect(150, 260, 51, 31))
        self.left.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.left.setText(_fromUtf8(""))
        self.left.setObjectName(_fromUtf8("left"))
        self.label = QtGui.QLabel(Form)
        self.label.setGeometry(QtCore.QRect(80, 30, 656, 484))
        self.label.setObjectName(_fromUtf8("label"))
        self.select = QtGui.QPushButton(Form)
        self.select.setGeometry(QtCore.QRect(340, 260, 51, 31))
        self.select.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.select.setText(_fromUtf8(""))
        self.select.setObjectName(_fromUtf8("select"))
        self.start = QtGui.QPushButton(Form)
        self.start.setGeometry(QtCore.QRect(430, 260, 51, 31))
        self.start.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.start.setText(_fromUtf8(""))
        self.start.setObjectName(_fromUtf8("start"))
        self.triangle = QtGui.QPushButton(Form)
        self.triangle.setGeometry(QtCore.QRect(580, 200, 51, 51))
        self.triangle.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.triangle.setText(_fromUtf8(""))
        self.triangle.setObjectName(_fromUtf8("triangle"))
        self.square = QtGui.QPushButton(Form)
        self.square.setGeometry(QtCore.QRect(530, 250, 51, 51))
        self.square.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.square.setText(_fromUtf8(""))
        self.square.setObjectName(_fromUtf8("square"))
        self.circle = QtGui.QPushButton(Form)
        self.circle.setGeometry(QtCore.QRect(630, 250, 51, 51))
        self.circle.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.circle.setText(_fromUtf8(""))
        self.circle.setObjectName(_fromUtf8("circle"))
        self.x = QtGui.QPushButton(Form)
        self.x.setGeometry(QtCore.QRect(580, 300, 51, 51))
        self.x.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.x.setText(_fromUtf8(""))
        self.x.setObjectName(_fromUtf8("x"))
        self.l1 = QtGui.QPushButton(Form)
        self.l1.setGeometry(QtCore.QRect(190, 110, 71, 27))
        self.l1.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.l1.setText(_fromUtf8(""))
        self.l1.setObjectName(_fromUtf8("l1"))
        self.l2 = QtGui.QPushButton(Form)
        self.l2.setGeometry(QtCore.QRect(190, 60, 61, 41))
        self.l2.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.l2.setText(_fromUtf8(""))
        self.l2.setObjectName(_fromUtf8("l2"))
        self.r1 = QtGui.QPushButton(Form)
        self.r1.setGeometry(QtCore.QRect(560, 110, 71, 27))
        self.r1.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.r1.setText(_fromUtf8(""))
        self.r1.setObjectName(_fromUtf8("r1"))
        self.r2 = QtGui.QPushButton(Form)
        self.r2.setGeometry(QtCore.QRect(560, 60, 71, 41))
        self.r2.setStyleSheet(_fromUtf8("background-color: rgba(255, 255, 255, 0);"))
        self.r2.setText(_fromUtf8(""))
        self.r2.setObjectName(_fromUtf8("r2"))
        self.analog_left_left = QtGui.QPushButton(Form)
        self.analog_left_left.setGeometry(QtCore.QRect(260, 350, 95, 20))
        self.analog_left_left.setText(_fromUtf8(""))
        self.analog_left_left.setObjectName(_fromUtf8("analog_left_left"))
        self.analog_left_up = QtGui.QPushButton(Form)
        self.analog_left_up.setGeometry(QtCore.QRect(300, 310, 21, 95))
        self.analog_left_up.setText(_fromUtf8(""))
        self.analog_left_up.setObjectName(_fromUtf8("analog_left_up"))
        self.analog_left_button = QtGui.QPushButton(Form)
        self.analog_left_button.setGeometry(QtCore.QRect(298, 410, 30, 30))
        self.analog_left_button.setText(_fromUtf8("L3"))
        self.analog_left_button.setObjectName(_fromUtf8("analog_right_button"))
        self.analog_right_up = QtGui.QPushButton(Form)
        self.analog_right_up.setGeometry(QtCore.QRect(500, 310, 21, 95))
        self.analog_right_up.setText(_fromUtf8(""))
        self.analog_right_up.setObjectName(_fromUtf8("analog_right_up"))
        self.analog_right_left = QtGui.QPushButton(Form)
        self.analog_right_left.setGeometry(QtCore.QRect(460, 350, 95, 20))
        self.analog_right_left.setText(_fromUtf8(""))
        self.analog_right_left.setObjectName(_fromUtf8("analog_right_left"))
        self.analog_right_button = QtGui.QPushButton(Form)
        self.analog_right_button.setGeometry(QtCore.QRect(498, 410, 30, 30))
        self.analog_right_button.setText(_fromUtf8("R3"))
        self.analog_right_button.setObjectName(_fromUtf8("analog_right_button"))

        self.save = QtGui.QPushButton(Form)
        self.save.setGeometry(QtCore.QRect(372, 460, 80, 60))
        self.save.setText(_fromUtf8("SAVE"))
        self.save.setObjectName(_fromUtf8("SAVE"))

        self.label_2 = QtGui.QLabel(Form)
        self.label_2.setGeometry(QtCore.QRect(300, 30, 300, 31))
        font = QtGui.QFont()
        font.setPointSize(16)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        self.label_2.setFont(font)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.label.raise_()
        self.up.raise_()
        self.left.raise_()
        self.select.raise_()
        self.start.raise_()
        self.triangle.raise_()
        self.square.raise_()
        self.circle.raise_()
        self.x.raise_()
        self.l1.raise_()
        self.l2.raise_()
        self.r2.raise_()
        self.r1.raise_()
        self.analog_left_left.raise_()
        self.analog_left_up.raise_()
        self.analog_left_button.raise_()
        self.analog_right_up.raise_()
        self.analog_right_left.raise_()
        self.analog_right_button.raise_()
        self.save.raise_()
        self.label_2.raise_()

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.label.setText(_translate("Form", "<html><head/><body><p><img src=\"joystick2.png\"/></p></body></html>", None))
        self.label_2.setText(_translate("Form", "set config joyevent", None))

import resource
