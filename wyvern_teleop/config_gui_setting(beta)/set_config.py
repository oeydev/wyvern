#!/usr/bin/env python

import sys
import yaml
from evdev import InputDevice, categorize, ecodes
from select import select
from PyQt4 import QtCore, QtGui
from std_msgs.msg import String
from gui import Ui_Form

keymap = {1: {}, 3: {}, 'digital_axes': {}, 'ignore': {'code': []}}


def get_data(mode, o, btn):
    dev = InputDevice('/dev/input/event15')
    global keymap
    print("press key")
    flag = True
    _name = btn
    _code = 0
    _type = 0
    _order = 0

    r, w, x = select([dev], [], [])

    for event in dev.read():
        if event.type in [1, 3] and flag:
            _code = event.code
            _type = event.type
            _order = o
            if mode == 'd' and event.code not in keymap['digital_axes'].keys():
                keymap['digital_axes'][event.code] = {
                    "name": btn, "type": event.type, "order": o}
                print("okay [ name: %s   code: %d   type: %d   order:   %d ]" % (
                    _name, _code, _type, _order))
                flag = False
            elif mode in ['a', 'b'] and event.code not in keymap[_type].keys():
                keymap[event.type][event.code] = {
                    "name": btn, "type": event.type, "order": o}
                print("okay [ name: %s   code: %d   type: %d   order:   %d ]" % (
                    _name, _code, _type, _order))
                flag = False
    dev.close()
    print('out')


class JoyUI(QtGui.QDialog):
    def __init__(self, parent=None):
        QtGui.QWidget.__init__(self, parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)


def Quit():
    global keymap
    with open('data.yaml', 'w') as outfile:
        yaml.dump(keymap, outfile, default_flow_style=False)
    print(keymap)
    exit()


if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    joyui = JoyUI()
    joyui.ui.up.clicked.connect(lambda: get_data('d', 1, 'up'))
    joyui.ui.left.clicked.connect(lambda: get_data('d', 0, 'left'))
    joyui.ui.square.clicked.connect(lambda: get_data('b', 3, 'square'))
    joyui.ui.triangle.clicked.connect(lambda: get_data('b', 0, 'triangle'))
    joyui.ui.circle.clicked.connect(lambda: get_data('b', 1, 'circle'))
    joyui.ui.x.clicked.connect(lambda: get_data('b', 2, 'x'))
    joyui.ui.l1.clicked.connect(lambda: get_data('b', 6, 'l1'))
    joyui.ui.l2.clicked.connect(lambda: get_data('b', 7, 'l2'))
    joyui.ui.r1.clicked.connect(lambda: get_data('b', 4, 'r1'))
    joyui.ui.r2.clicked.connect(lambda: get_data('b', 5, 'r2'))
    joyui.ui.analog_left_left.clicked.connect(
        lambda: get_data('a', 0, 'analog_left_left'))
    joyui.ui.analog_left_up.clicked.connect(
        lambda: get_data('a', 1, 'analog_left_up'))
    joyui.ui.analog_right_left.clicked.connect(
        lambda: get_data('a', 2, 'analog_right_left'))
    joyui.ui.analog_right_up.clicked.connect(
        lambda: get_data('a', 3, 'analog_right_up'))
    joyui.ui.select.clicked.connect(lambda: get_data('b', 11, 'select'))
    joyui.ui.start.clicked.connect(lambda: get_data('b', 10, 'start'))
    joyui.ui.analog_left_button.clicked.connect(lambda: get_data('b', 9, 'L3'))
    joyui.ui.analog_right_button.clicked.connect(
        lambda: get_data('b', 8, 'R3'))
    joyui.ui.save.clicked.connect(lambda: Quit())

    joyui.show()
    sys.exit(app.exec_())
