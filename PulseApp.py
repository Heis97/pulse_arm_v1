from time import sleep
import sys
from PyQt5 import QtCore,  QtWidgets
from PyQt5.QtWidgets import (QPushButton, QLineEdit, QApplication,QTextEdit,QLabel,QComboBox,QRadioButton)
from PyQt5.QtCore import Qt,QRect
from PyQt5.QtGui import QPainter, QPen
from PyQt5.QtCore import ( QPoint, QSize,Qt)
from enum import Enum
import json
import math
from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_LINEAR,jog,create_box_obstacle,LinearMotionParameters,InterpolationType,tool_info
from pdhttp import Position,Point,Rotation,Pose,MotorStatus
from g_code_parser import *
from PulseUtil import *

def position_sum(p1:Position,p2:Position):
    x = p1.point.x+p2.point.x
    y = p1.point.y+p2.point.y
    z = p1.point.z+p2.point.z

    u = check_angle(p1.rotation.pitch + p2.rotation.pitch)
    v = check_angle(p1.rotation.roll  + p2.rotation.roll)
    w = check_angle(p1.rotation.yaw   + p2.rotation.yaw)

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

def pose_to_str(p:Pose,separator:str ="\n")->str:
    p = p.to_dict()
    angles = p["angles"]
    pres = 2
    return "A1: "+str(round(angles[0],pres))+separator+"A2: "+str(round(angles[1],pres))+separator+"A3: "+str(round(angles[2],pres))+separator+"A4: "+str(round(angles[3],pres))+separator+"A5: "+str(round(angles[4],pres))+separator+"A6: "+str(round(angles[5],pres))

def pose_to_list(p:Pose)->list: 
    return p.angles

def position_to_str(p:Position,separator:str ="\n")->str:
    p = p.to_dict()
    pos = p["point"]
    rot = p["rotation"]

    pres = 2
    return "X: "+str(round(1000*pos["x"],pres))+separator+"Y: "+str(round(1000*pos["y"],pres))+separator+"Z: "+str(round(1000*pos["z"],pres))+separator+"Rx: "+str(round(rot["roll"],pres))+separator+"Ry: "+str(round(rot["pitch"],pres))+separator+"Rz: "+str(round(rot["yaw"],pres))


def position_to_list(p:Position)->list:
    pos = p.point
    rot = p.rotation
    return [pos.x,pos.y,pos.z,rot.roll,rot.pitch,rot.yaw]

def motor_state_to_str(ms:list[MotorStatus]):
    txt = "Current:\n"
    for m in ms:
        m_d = m.to_dict()
        txt += str(round(m_d['phase_current'],2) )+"\n"#+" "+str(round(m_d['rms_current'],2) )+" "+str(round(m_d['voltage'],2) )

    return txt

class SettingsPulse():
    tools:dict = None
    bases:dict = None
    work_poses:dict = None
    start_points:dict= None

    def __init__(self,_tools,_bases,_work_poses,_start_points):
        self.tools = _tools
        self.bases = _bases
        self.work_poses = _work_poses
        self.start_points = _start_points

    def for_ser(self):
        return [self.tools,self.bases,self.work_poses,self.start_points]


class RobPosThread(QtCore.QThread):
    def __init__(self,pulse_arm:RobotPulse, label:QLabel):
        QtCore.QThread.__init__(self)   
        self.pulse_arm = pulse_arm
        self.label = label
        self.timeDelt = 0.05
        self.start()   
    
    def run(self):
        while True:           
            try:               
                self.label.setText("Joint position:\n"+pose_to_str(self.pulse_arm.get_pose())+"\n\n\n"+"Cartesian position:\n"+position_to_str(self.pulse_arm.get_position())+"\n\n\n"+motor_state_to_str(self.pulse_arm.status_motors()))                 
            except BaseException:
                pass
        
            sleep(self.timeDelt)

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
        self.load_settings()
        self.setWindowTitle("Интерфейс Pulse")
        self.resize(1750, 1000)
        self.build()  

        


 
    def build(self):
        self.build_connection()
        self.build_position()
        self.build_coords()
        self.build_settings()
        self.build_config()
        self.build_progr()

    def build_connection(self):
        self.but_connect_robot = QPushButton('Подключиться', self)
        self.but_connect_robot.setGeometry(QtCore.QRect(100, 100, 140, 30))
        self.but_connect_robot.clicked.connect(self.connect_robot)

        self.but_connect_robot = QPushButton('Отключиться', self)
        self.but_connect_robot.setGeometry(QtCore.QRect(100, 140, 140, 30))
        self.but_connect_robot.clicked.connect(self.disconnect_robot)

    def connect_robot(self):
        self.pulse_robot = RobotPulse(host)
        #self.pulse_robot.recover()
        ps = [self.settins_pulse.start_points["calib_1_1"],self.settins_pulse.start_points["calib_1_2"],self.settins_pulse.start_points["calib_1_3"],self.settins_pulse.start_points["calib_1_4"],self.settins_pulse.start_points["calib_1_5"]]
        
        tcp = calibrate_tcp_4p(ps)
        tool = tool_info(position([tcp[0][0],tcp[1][0],tcp[2][0]],[0,0,0]))
        self.pulse_robot.change_tool_info(tool)

        self.coords_thread = RobPosThread(self.pulse_robot,self.lab_coord)


    

    def disconnect_robot(self):
        self.pulse_robot = None

    def add_axis_buttons(self,name:ax, pos:QtCore.QRect,f_press):
        but_ax_positive = QPushButton(str(name)[-1:]+'+', self)
        but_ax_positive.setGeometry(pos)
        but_ax_positive.pressed.connect(f_press)
        but_ax_positive.released.connect(self.stop_robot)
        pos2 = QtCore.QRect(pos.x(),pos.y()+pos.height()+10,pos.width(),pos.height())
        but_ax_negative = QPushButton(str(name)[-1:]+'-', self)
        but_ax_negative.setGeometry(pos2)
        but_ax_negative.pressed.connect(f_press)
        but_ax_negative.released.connect(self.stop_robot)

    def build_position(self):
        self.but_home_position = QPushButton('Изначальное положение', self)
        self.but_home_position.setGeometry(QtCore.QRect(100, 200, 140, 30))
        self.but_home_position.clicked.connect(self.home_position)

        self.but_work_position = QPushButton('Рабочее положение', self)
        self.but_work_position.setGeometry(QtCore.QRect(100, 240, 140, 30))
        self.but_work_position.clicked.connect(self.work_position)

        self.but_relax_robot = QPushButton('Выключить приводы', self)
        self.but_relax_robot.setGeometry(QtCore.QRect(100, 280, 140, 30))
        self.but_relax_robot.clicked.connect(self.relax_robot)

        self.add_axis_buttons(ax.X,QtCore.QRect(100, 340, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.Y,QtCore.QRect(140, 340, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.Z,QtCore.QRect(180, 340, 30, 30),self.axis_jog)

        self.add_axis_buttons(ax.U,QtCore.QRect(220, 340, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.V,QtCore.QRect(260, 340, 30, 30),self.axis_jog)
        self.add_axis_buttons(ax.W,QtCore.QRect(300, 340, 30, 30),self.axis_jog)

        self.add_axis_buttons(ax.X,QtCore.QRect(100, 440, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.Y,QtCore.QRect(140, 440, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.Z,QtCore.QRect(180, 440, 30, 30),self.axis_move)

        self.add_axis_buttons(ax.U,QtCore.QRect(220, 440, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.V,QtCore.QRect(260, 440, 30, 30),self.axis_move)
        self.add_axis_buttons(ax.W,QtCore.QRect(300, 440, 30, 30),self.axis_move)

    def home_position(self):
        home_pose = pose([0, -90, 0, -90, -90, 0])
        SPEED = 20
        self.pulse_robot.set_pose(home_pose, speed=SPEED)

    def work_position(self):
        position_target = position([-0.42, -0.12, 0.35], [math.pi/2, 0, 0])
        SPEED = 10
        print(position_target)
        self.pulse_robot.set_position(position_target, velocity=SPEED, acceleration=10)

    def relax_robot(self):
        self.pulse_robot.relax()

    def axis_jog(self):
        but = self.sender()
        acs = 0.1        
        x,y,z,Rx,Ry,Rz = self.mask_from_button(but.text())
        self.pulse_robot.jogging(jog(x=acs*x,y=acs*y,z=acs*z,rx=acs*Rx,ry=acs*Ry,rz=acs*Rz))   

    def axis_move(self):
        but = self.sender()
        acs = 5   
        vel = 4     
        step = 0.001

        x,y,z,Rx,Ry,Rz = self.mask_from_button(but.text())        
        position_delt = position([step*x, step*y, step*z], [step*Rx, step*Ry, step*Rz])
        pos,rot = position_sum2(self.pulse_robot.get_position(),position_delt)
        pos_rel = position(pos,rot)
        #print(self.pulse_robot.get_position())
        #print(pos_rel)
        self.pulse_robot.set_position(pos_rel , velocity=vel, acceleration=acs)
        #self.pulse_robot.await_stop()

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
        elif name[0]=="U": Rx = sign
        elif name[0]=="V": Ry = sign
        elif name[0]=="W": Rz = sign
        return x,y,z,Rx,Ry,Rz

    def stop_robot(self):

        self.pulse_robot.freeze()
    
    buffer_positions:list[Position] = []
    buffer_poses:list[Pose] = []

    def build_coords(self):
        self.lab_coord = QLabel(self)
        self.lab_coord.setGeometry(QtCore.QRect(400, 10, 100, 800))
        self.lab_coord.setAlignment(Qt.AlignmentFlag.AlignTop|Qt.AlignmentFlag.AlignLeft)
        self.lab_coord.setText('Координаты')
        self.coords_thread = None

        self.but_add_position = QPushButton('Добавить позицию', self)
        self.but_add_position.setGeometry(QtCore.QRect(400, 340, 140, 30))
        self.but_add_position.clicked.connect(self.add_position)

        self.but_clear_buf_positions = QPushButton('Очистить позиции', self)
        self.but_clear_buf_positions.setGeometry(QtCore.QRect(400, 380, 140, 30))
        self.but_clear_buf_positions.clicked.connect(self.clear_buf_positions)

        self.lab_buf_positions = QLabel(self)
        self.lab_buf_positions.setGeometry(QtCore.QRect(400, 420, 800, 400))
        self.lab_buf_positions.setAlignment(Qt.AlignmentFlag.AlignTop|Qt.AlignmentFlag.AlignLeft)
        self.lab_buf_positions.setText('')

        self.but_add_pose= QPushButton('Добавить конфигурацию', self)
        self.but_add_pose.setGeometry(QtCore.QRect(750, 340, 140, 30))
        self.but_add_pose.clicked.connect(self.add_pose)

        self.but_clear_buf_poses = QPushButton('Очистить конфигурации', self)
        self.but_clear_buf_poses.setGeometry(QtCore.QRect(750, 380, 140, 30))
        self.but_clear_buf_poses.clicked.connect(self.clear_buf_poses)

        self.lab_buf_poses = QLabel(self)
        self.lab_buf_poses.setGeometry(QtCore.QRect(750, 420, 800, 400))
        self.lab_buf_poses.setAlignment(Qt.AlignmentFlag.AlignTop|Qt.AlignmentFlag.AlignLeft)
        self.lab_buf_poses.setText('')

    def add_position(self):
        self.buffer_positions.append(self.pulse_robot.get_position())
        self.show_buffers()

    def add_pose(self):
        self.buffer_poses.append(self.pulse_robot.get_pose())
        self.show_buffers()

    def add_position_from_cur(self):
        start_point =  self.get_cur_item_from_combo(self.combo_start_points,self.settins_pulse.start_points)
        self.buffer_positions.append(Position(start_point["point"],start_point["rotation"]))
        self.show_buffers()

    def add_pose_from_cur(self):
        work_pose = self.get_cur_item_from_combo(self.combo_work_poses,self.settins_pulse.work_poses)
        self.buffer_poses.append(Pose(work_pose["angles"]))
        self.show_buffers()

    def clear_buf_positions(self):
        self.buffer_positions = []
        self.show_buffers()

    def clear_buf_poses(self):
        self.buffer_poses = []
        self.show_buffers()

    def show_buffers(self):
        self.lab_buf_positions.setText("")
        self.lab_buf_poses.setText("")
        for i in range(len(self.buffer_positions)):
            self.lab_buf_positions.setText(self.lab_buf_positions.text()+"P"+str(i)+" "+position_to_str(self.buffer_positions[i],separator=" ")+"\n")
        for i in range(len(self.buffer_poses)):
            self.lab_buf_poses.setText(self.lab_buf_poses.text()+"P"+str(i)+" "+pose_to_str(self.buffer_poses[i],separator=" ")+"\n")

    settings_path = "settings_pulse.json"
    settins_pulse = SettingsPulse(dict(),dict(),dict(),dict())

    current_tool = None
    current_base = None


    def build_settings(self):
        self.but_load_settings = QPushButton('Загрузить настройки', self)
        self.but_load_settings.setGeometry(QtCore.QRect(100, 640, 140, 30))
        self.but_load_settings.clicked.connect(self.load_settings)

        self.but_save_settings = QPushButton('Сохранить настройки', self)
        self.but_save_settings.setGeometry(QtCore.QRect(100, 680, 140, 30))
        self.but_save_settings.clicked.connect(self.save_settings)
        #---------------------------------------------------------------------------
        self.but_save_start_point = QPushButton('Сохранить старт. т.', self)
        self.but_save_start_point.setGeometry(QtCore.QRect(250, 640, 140, 30))
        self.but_save_start_point.clicked.connect(self.save_start_point_direct)
       
        self.lin_name_start_point = QLineEdit(self)
        self.lin_name_start_point.setGeometry(QtCore.QRect(400, 640, 140, 30))
        self.lin_name_start_point.setText("new_start_point")
        #---------------------------------------------------------------------------
        self.but_save_work_pose = QPushButton('Сохранить рабоч. конф.', self)
        self.but_save_work_pose.setGeometry(QtCore.QRect(250, 680, 140, 30))
        self.but_save_work_pose.clicked.connect(self.save_work_pose_direct)

        self.lin_name_work_pose = QLineEdit(self)
        self.lin_name_work_pose.setGeometry(QtCore.QRect(400, 680, 140, 30))
        self.lin_name_work_pose.setText("new_work_pose")

        #---------------------------------------------------------------------------
        self.but_save_tool = QPushButton('Сохранить инструмент', self)
        self.but_save_tool.setGeometry(QtCore.QRect(250, 720, 140, 30))
        self.but_save_tool.clicked.connect(self.save_tool)

        self.but_create_tool = QPushButton('Создать инструмент', self)
        self.but_create_tool.setGeometry(QtCore.QRect(250, 720, 140, 30))
        self.but_create_tool.clicked.connect(self.create_tool)

        self.lin_name_tool = QLineEdit(self)
        self.lin_name_tool.setGeometry(QtCore.QRect(400, 720, 140, 30))
        self.lin_name_tool.setText("new_tool")

        self.but_comp_tcp_point = QPushButton('Расч. точку TCP', self)
        self.but_comp_tcp_point.setGeometry(QtCore.QRect(140, 720, 100, 30))
        self.but_comp_tcp_point.clicked.connect(self.comp_tcp_point)

        self.but_comp_tcp_rotate = QPushButton('Расч. ориент TCP', self)
        self.but_comp_tcp_rotate.setGeometry(QtCore.QRect(30, 720, 100, 30))
        self.but_comp_tcp_rotate.clicked.connect(self.comp_tcp_rotate)


        #---------------------------------------------------------------------------
        self.but_save_base = QPushButton('Сохранить базу', self)
        self.but_save_base.setGeometry(QtCore.QRect(250, 760, 140, 30))
        self.but_save_base.clicked.connect(self.save_base)

        self.lin_name_base = QLineEdit(self)
        self.lin_name_base.setGeometry(QtCore.QRect(400, 760, 140, 30))
        self.lin_name_base.setText("new_base")

    def comp_tcp_point(self):
        ps = [self.settins_pulse.start_points["calib_1_1"],self.settins_pulse.start_points["calib_1_2"],self.settins_pulse.start_points["calib_1_3"],self.settins_pulse.start_points["calib_1_4"],self.settins_pulse.start_points["calib_1_5"]]        
        ps = self.buffer_positions
        tcp = calibrate_tcp_4p(ps)
        tool = tool_info(position([tcp[0][0],tcp[1][0],tcp[2][0]],[0,0,0]))
        


    def comp_tcp_rotate(self):

        pass

    def comp_base(self):
        
        pass

    
    def create_tool(self):
        name = self.lin_name_tool.text()
        self.settins_pulse.tools[name] = tool_info(position( [0,0,0],[0,0,0]),name).to_dict()

    def load_settings(self):
        list_settings = []
        with open(self.settings_path) as file:   
            list_settings=json.load(file)
        self.settins_pulse = SettingsPulse(list_settings[0],list_settings[1],list_settings[2],list_settings[3])

    def save_settings(self):
        with open(self.settings_path, "w", encoding="utf-8") as file:
            json.dump(self.settins_pulse.for_ser(), file,indent=5)

    def save_start_point(self):
        if len(self.buffer_positions)>0:
            self.settins_pulse.start_points[self.lin_name_start_point.text()] =  self.buffer_positions[0].to_dict()
        self.save_settings()
        self.set_setting_items()

    def save_work_pose(self):
        if len(self.buffer_poses)>0:
            self.settins_pulse.work_poses[self.lin_name_work_pose.text()] =  self.buffer_poses[0].to_dict()
        self.save_settings()
        self.set_setting_items()

    def save_tool(self):
        if self.current_tool is not None:
            self.settins_pulse.tools[self.lin_name_tool.text()] =  self.current_tool.to_dict()
        self.save_settings()
        self.set_setting_items()

    def save_base(self):
        if self.current_base is not None:
            self.settins_pulse.bases[self.lin_name_base.text()] =  self.current_base.to_dict()
        self.save_settings()
        self.set_setting_items()

    
    def del_position(self):
        del self.settins_pulse.start_points[self.combo_start_points.currentText()]
        self.save_settings()
        self.set_setting_items()
    
    def del_pose(self):
        del self.settins_pulse.work_poses[self.combo_work_poses.currentText()]
        self.save_settings()
        self.set_setting_items()
    

    def save_start_point_direct(self):
        self.settins_pulse.start_points[self.lin_name_start_point.text()] =  (self.pulse_robot.get_position()).to_dict()
        self.save_settings()
        self.set_setting_items()

    def save_work_pose_direct(self):
        self.settins_pulse.work_poses[self.lin_name_work_pose.text()] =  (self.pulse_robot.get_pose()).to_dict()
        self.save_settings()
        self.set_setting_items()

    def build_config(self):

        self.combo_start_points = QComboBox(self)
        self.combo_start_points.setGeometry(QRect(870, 560, 120, 30))        
        self.combo_start_points.setCurrentIndex(0)

        self.combo_work_poses = QComboBox(self)
        self.combo_work_poses.setGeometry(QRect(870, 600, 120, 30))        
        self.combo_work_poses.setCurrentIndex(0)

        self.combo_tools = QComboBox(self)
        self.combo_tools.setGeometry(QRect(870, 640, 120, 30))        
        self.combo_tools.setCurrentIndex(0)

        self.combo_bases = QComboBox(self)
        self.combo_bases.setGeometry(QRect(870, 680, 120, 30))       
        self.combo_bases.setCurrentIndex(0)

        self.but_add_position_from_cur = QPushButton('Добавить в буффер', self)
        self.but_add_position_from_cur.setGeometry(QtCore.QRect(720, 560, 140, 30))
        self.but_add_position_from_cur.clicked.connect(self.add_position_from_cur)

        self.but_add_pose_from_cur = QPushButton('Добавить в буффер', self)
        self.but_add_pose_from_cur.setGeometry(QtCore.QRect(720, 600, 140, 30))
        self.but_add_pose_from_cur.clicked.connect(self.add_pose_from_cur)

        self.but_del_position = QPushButton('Удалить', self)
        self.but_del_position.setGeometry(QtCore.QRect(720, 560, 140, 30))
        self.but_del_position.clicked.connect(self.del_position)

        self.but_del_pose = QPushButton('Удалить', self)
        self.but_del_pose.setGeometry(QtCore.QRect(720, 600, 140, 30))
        self.but_del_pose.clicked.connect(self.del_pose)

        
        self.set_setting_items()

    def set_setting_items(self):
        self.combo_start_points.clear()
        self.combo_work_poses.clear()
        self.combo_tools.clear()
        self.combo_bases.clear()

        self.combo_start_points.addItems(self.settins_pulse.start_points)
        self.combo_work_poses.addItems(self.settins_pulse.work_poses)
        self.combo_tools.addItems(self.settins_pulse.tools)
        self.combo_bases.addItems(self.settins_pulse.bases)

    def get_cur_item_from_combo(self, combo:QComboBox, data:dict):
        try:

            return data[combo.currentText()]
        except KeyError:
            print("key err")
            return None

    cur_start_point = None
    cur_work_pose = None
    cur_tool = None
    cur_base = None

    def apply_settings_to_robot(self):      
        self.cur_tool = self.get_cur_item_from_combo(self.combo_tools,self.settins_pulse.tools)
        self.cur_base = self.get_cur_item_from_combo(self.combo_bases,self.settins_pulse.bases)
        self.cur_start_point = self.get_cur_item_from_combo(self.combo_start_points,self.settins_pulse.start_points)
        if self.cur_tool is not None:
            self.pulse_robot.change_tool_info(self.cur_tool)
        if self.cur_base is not None:
            self.pulse_robot.change_base(self.cur_base)

    
    def build_progr(self):

        self.but_set_cur_start_point = QPushButton('Исполнить стартовую т.', self)
        self.but_set_cur_start_point.setGeometry(QtCore.QRect(1000, 560, 140, 30))
        self.but_set_cur_start_point.clicked.connect(self.set_cur_start_point)

        self.but_set_cur_work_pose = QPushButton('Исполнить рабоч. конф.', self)
        self.but_set_cur_work_pose.setGeometry(QtCore.QRect(1000, 600, 140, 30))
        self.but_set_cur_work_pose.clicked.connect(self.set_cur_work_pose)

        self.but_start_prog = QPushButton('Исполнить программу', self)
        self.but_start_prog.setGeometry(QtCore.QRect(1000, 640, 140, 30))
        self.but_start_prog.clicked.connect(self.exec_prog_arm)

        self.but_stop_robot = QPushButton('Остановить', self)
        self.but_stop_robot.setGeometry(QtCore.QRect(1000, 680, 140, 30))
        self.but_stop_robot.clicked.connect(self.stop_robot)

        self.text_prog_code = QTextEdit(self)
        self.text_prog_code.setGeometry(QtCore.QRect(1150, 560, 500, 400))


    def set_cur_work_pose(self):
        self.cur_work_pose = self.get_cur_item_from_combo(self.combo_work_poses,self.settins_pulse.work_poses)
        if self.cur_work_pose is not None:
            self.pulse_robot.set_pose(Pose(self.cur_work_pose["angles"]),5)
            #self.pulse_robot.await_stop()

    def set_cur_start_point(self):
        self.cur_start_point = self.get_cur_item_from_combo(self.combo_start_points,self.settins_pulse.start_points)
        if self.cur_start_point is not None:
            self.pulse_robot.set_position(Position(self.cur_start_point["point"],self.cur_start_point["rotation"]),velocity=5,acceleration=1)
            #self.pulse_robot.await_stop()

    def exec_prog_arm(self):
        vel = 5
        acs = 0.1
        self.apply_settings_to_robot()
        
        self.pulse_robot.set_position(Position(self.cur_start_point["point"],self.cur_start_point["rotation"]),velocity=vel,acceleration=acs,motion_type=MT_LINEAR)

        positions = self.generate_traj()
        vel = 0.005
        acs = 0.1
        linear_motion_parameters = LinearMotionParameters(interpolation_type=InterpolationType.BLEND,velocity=vel,acceleration=acs)
        self.pulse_robot.run_linear_positions(positions,linear_motion_parameters)


    def stop_robot(self):
        self.pulse_robot.stop()
        self.pulse_robot.recover()

#-----------------------------------------------------------------------------------
    def generate_traj(self):
        
        ps = parse_g_code(self.text_prog_code.toPlainText())
        start_point = self.cur_start_point["point"]
        start_rot =  self.cur_start_point["rotation"]

        points = []
        p = [start_point["x"],start_point["y"],start_point["z"]]
        r = [start_rot["roll"],start_rot["pitch"],start_rot["yaw"]]
        pos = position(p,r)
        points.append(p)
        positions = [pos]
        for i in range(len(ps)):               
            p = [start_point["x"]+ps[i].x,start_point["y"]+ps[i].y,start_point["z"]+ps[i].z]
            r = [start_rot["roll"],start_rot["pitch"],start_rot["yaw"]]
            
            pos = position(p,r,blend=0.0001)
            if self.dist(p,points[-1])>0.00001:
                positions.append(pos)
                points.append(p)

        for i in range(len(positions)):
            print(i," ",positions[i])
        
        return positions


#-----------------------------------------------------------------------------------
    def generate_traj_abc(self):
        
        ps = parse_g_code(self.text_prog_code.toPlainText())
        start_point = self.cur_start_point["point"]
        start_rot =  self.cur_start_point["rotation"]

        points = []
        p = [start_point["x"],start_point["y"],start_point["z"]]
        r = [start_rot["roll"],start_rot["pitch"],start_rot["yaw"]]
        pos = position(p,r)
        points.append(p)
        positions = [pos]
        for i in range(len(ps)):               
            p = [start_point["x"]+ps[i].x,start_point["y"]+ps[i].y,start_point["z"]+ps[i].z]
            r = [start_rot["roll"]+ps[i].r,start_rot["pitch"]+ps[i].g,start_rot["yaw"]+ps[i].b]
            
            pos = position(p,r,blend=0.0001)
            if self.dist(p,points[-1])>0.00001:
                positions.append(pos)
                points.append(p)

        for i in range(len(positions)):
            print(i," ",positions[i])
        
        return positions


    def dist(self,p1,p2):
        return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)

    


        


    


   

if __name__ == '__main__':    
    app = QApplication(sys.argv)
    pulse = PulseApp()
    pulse.show()
    sys.exit(app.exec_())