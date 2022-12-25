from time import sleep
import sys
from PyQt5 import QtCore,  QtWidgets
from PyQt5.QtWidgets import (QPushButton, QLineEdit, QApplication)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtCore import ( QPoint, QSize,Qt)
from enum import Enum
import math
from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_LINEAR,jog
from pdhttp import Position,Point,Rotation


def position_sum(p1:Position,p2:Position):
    print(p1)
    print(p2)
    x = p1.point.x+p2.point.x
    y = p1.point.y+p2.point.y
    z = p1.point.z+p2.point.z

    u = check_angle(p1.rotation.pitch + p2.rotation.pitch)
    v = check_angle(p1.rotation.roll  + p2.rotation.roll)
    w = check_angle(p1.rotation.yaw   + p2.rotation.yaw)
    print(x,y,z,u,v,w)

    return [x,y,z] ,[u,v,w]
def check_angle(angle:float):
    if angle>math.pi:
        angle-=2*math.pi
    if angle<-math.pi:
        angle+=2*math.pi
    return angle

def position_sum2(p1:Position,p2:Position)->Position:

    x = p1.point.x+p2.point.x
    y = p1.point.y+p2.point.y
    z = p1.point.z+p2.point.z

    
    u = p1.rotation.roll  + p2.rotation.roll
    v = p1.rotation.pitch + p2.rotation.pitch
    w = p1.rotation.yaw   + p2.rotation.yaw

    return [x,y,z] ,[u,v,w]


host = "http://10.10.10.20:8081"

class ax(Enum):
    X="X"
    Y="Y"
    Z="Z"
    U = "U"
    V = "V"
    W = "W"

class PulseApp(QtWidgets.QWidget):
    
    pulse_robot = None
    count = 0

    def __init__(self, parent=None):
        # Передаём ссылку на родительский элемент и чтобы виджет
        # отображался как самостоятельное окно указываем тип окна
        super().__init__(parent, QtCore.Qt.Window)
        self.setWindowTitle("Интерфейс Pulse")
        self.resize(1750, 800)
        self.build()  

    
    def build(self):
        self.build_connection()
        self.build_position()


        self.lin_= QtWidgets.QLineEdit(self)
        self.lin_.setGeometry(QtCore.QRect(520, 220, 140, 30))
        self.lin_.setText("0")


        self.tex_ = QtWidgets.QTextEdit(self)
        self.tex_.setGeometry(QtCore.QRect(30, 320, 620, 800))
        self.tex_.setVisible(False)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setGeometry(QtCore.QRect(350, 16, 236, 50))
        self.label1.setText('Не подключено')


        self.cBauds_printer = QtWidgets.QComboBox(self)
        self.cBauds_printer.setGeometry(QtCore.QRect(0, 70, 120, 30))
        #self.cBauds_printer.addItems(self.bauds)
        self.cBauds_printer.setCurrentText = self.cBauds_printer.itemText(-1)

    def build_connection(self):
        self.but_connect_robot = QtWidgets.QPushButton('Подключиться', self)
        self.but_connect_robot.setGeometry(QtCore.QRect(100, 100, 140, 30))
        self.but_connect_robot.clicked.connect(self.connect_robot)

        self.but_connect_robot = QtWidgets.QPushButton('Отключиться', self)
        self.but_connect_robot.setGeometry(QtCore.QRect(100, 140, 140, 30))
        self.but_connect_robot.clicked.connect(self.disconnect_robot)

    def connect_robot(self):
        self.pulse_robot = RobotPulse(host)

    def disconnect_robot(self):
        self.pulse_robot = None

    def add_axis_buttons(self,name:ax, pos:QtCore.QRect,f_press):
        but_ax_positive = QtWidgets.QPushButton(str(name)[-1:]+'+', self)
        but_ax_positive.setGeometry(pos)
        but_ax_positive.pressed.connect(f_press)
        but_ax_positive.released.connect(self.stop_robot)
        pos2 = QtCore.QRect(pos.x(),pos.y()+pos.height()+10,pos.width(),pos.height())
        but_ax_negative = QtWidgets.QPushButton(str(name)[-1:]+'-', self)
        but_ax_negative.setGeometry(pos2)
        but_ax_negative.pressed.connect(f_press)
        but_ax_negative.released.connect(self.stop_robot)


    def build_position(self):
        self.but_home_position = QtWidgets.QPushButton('Изначальное положение', self)
        self.but_home_position.setGeometry(QtCore.QRect(100, 200, 140, 30))
        self.but_home_position.clicked.connect(self.home_position)

        self.but_work_position = QtWidgets.QPushButton('Рабочее положение', self)
        self.but_work_position.setGeometry(QtCore.QRect(100, 240, 140, 30))
        self.but_work_position.clicked.connect(self.work_position)

        self.add_axis_buttons(ax.X,QtCore.QRect(100, 340, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.Y,QtCore.QRect(140, 340, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.Z,QtCore.QRect(180, 340, 30, 30),self.axis_jog)

        self.add_axis_buttons(ax.U,QtCore.QRect(100, 440, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.V,QtCore.QRect(140, 440, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.W,QtCore.QRect(180, 440, 30, 30),self.axis_jog)

        self.add_axis_buttons(ax.X,QtCore.QRect(100, 540, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.Y,QtCore.QRect(140, 540, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.Z,QtCore.QRect(180, 540, 30, 30),self.axis_move)

        self.add_axis_buttons(ax.U,QtCore.QRect(100, 640, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.V,QtCore.QRect(140, 640, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.W,QtCore.QRect(180, 640, 30, 30),self.axis_move)



    def home_position(self):
        home_pose = pose([0, -90, 0, -90, -90, 0])
        SPEED = 20
        self.pulse_robot.set_pose(home_pose, speed=SPEED)

    def work_position(self):
        position_target = position([-0.42, -0.12, 0.35], [math.pi, 0, 0])
        SPEED = 10
        print(position_target)
        self.pulse_robot.set_position(position_target, velocity=SPEED, acceleration=10)


    def axis_jog(self):
        but = self.sender()
        acs = 0.5        
        x,y,z,Rx,Ry,Rz = self.mask_from_button(but.text())
        self.pulse_robot.jogging(jog(x=acs*x,y=acs*y,z=acs*z,rx=acs*Rx,ry=acs*Ry,rz=acs*Rz))   

    def axis_move(self):
        but = self.sender()
        acs = 5   
        vel = 4     
        step = 0.0001

        x,y,z,Rx,Ry,Rz = self.mask_from_button(but.text())        
        position_delt = position([step*x, step*y, step*z], [step*Rx, step*Ry, step*Rz])
        pos,rot = position_sum2(self.pulse_robot.get_position(),position_delt)
        pos_rel = position(pos,rot)
        #print(self.pulse_robot.get_position())
        #print(pos_rel)
        self.pulse_robot.set_position(pos_rel , velocity=vel, acceleration=acs)
        self.pulse_robot.await_stop()



    def mask_from_button(self,name):
        
        x = 0
        y = 0
        z = 0 
        Rx = 0
        Ry = 0
        Rz = 0
        sign = 1
        if name[1] == '-': sign = -1

        if name[0]=="X": x = sign
        elif name[0]=="Y": y = sign
        elif name[0]=="Z": z = sign
        elif name[0]=="U": Rz = sign
        elif name[0]=="V": Ry = sign
        elif name[0]=="W": Rz = sign
        return x,y,z,Rx,Ry,Rz

    def stop_robot(self):

        self.pulse_robot.freeze()
    

    

if __name__ == '__main__':    
    app = QApplication(sys.argv)
    pulse = PulseApp()
    pulse.show()
    sys.exit(app.exec_())