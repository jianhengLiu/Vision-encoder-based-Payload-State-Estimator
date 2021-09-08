#!/usr/bin/env python3

import rospy
import math
from wtr_can_motor.msg import motor_msgs
import sys
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import pyqtgraph as pg
import numpy as np
import string
import time
data = []  # 要画的数据存在这里


class Signal(QWidget):  # 先写GUI界面
    ptr1 = 0

    def __init__(self):
        super(Signal, self).__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(0, 0, 1200, 800)
        self.setWindowTitle('signal_analysis')
        layout_chart = QtWidgets.QGridLayout()
        self.setLayout(layout_chart)
        pg.setConfigOption('background', 'w')
        self.pw = pg.PlotWidget()
        self.pw.showGrid(x=True, y=True)
        self.curve = self.pw.plot(pen='k')
        layout_chart.addWidget(self.pw, 0, 0, 9, 10)

        bt1 = QPushButton('Button', self)
        layout_chart.addWidget(bt1, 10, 0, 1, 1)
        text1_edit = QLineEdit("", self)
        layout_chart.addWidget(text1_edit, 10, 1, 1, 2)

    def draw_signal(self, x, y):
        data.append(y)  # ROS上传来的y值存在data里
        self.ptr1 = x  # ROS上传来的x值赋给ptr1
        self.curve.setData(data)  # 纵坐标
        self.curve.setPos(self.ptr1, 0)  # 横坐标


def callback(data, gui):
    gui.draw_signal(time.time(),data.angle,)  # 回调函数里调用draw_signal画图


def main():
    app = QtWidgets.QApplication(sys.argv)
    gui = Signal()
    gui.show()

    rospy.init_node('pylistener', anonymous=True)  # ros节点初始化
    rospy.Subscriber('/wtr/motor/0', motor_msgs, callback, gui,queue_size=1,buff_size=102400)  # ros节点接收数据

    sys.exit(app.exec_())
    rospy.spin()


if __name__ == '__main__':
    main()
