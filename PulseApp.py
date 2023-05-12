from time import sleep
import sys
from PyQt5 import QtCore,  QtWidgets
from PyQt5.QtWidgets import (QPushButton, QLineEdit, QApplication,QTextEdit,QLabel,QComboBox,QRadioButton)
from PyQt5.QtCore import Qt,QRect,QPointF
from enum import Enum
import json
import math
from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_LINEAR,jog,create_box_obstacle,LinearMotionParameters,InterpolationType,tool_info
from pdhttp import Position,Point,Rotation,Pose,MotorStatus,PoseTimestamp,PositionTimestamp
from Viewer3D_GL import Paint_in_GL,GLWidget
from g_code_parser import *
from PulseUtil import *
from KukaRobot import *
from PulseRobotExt import *
from Plotter import Plotter

def position_sum(p1:Position,p2:Position):
    x = p1.point.x+p2.point.x
    y = p1.point.y+p2.point.y
    z = p1.point.z+p2.point.z

    u = check_angle(p1.rotation.roll  + p2.rotation.roll)
    v = check_angle(p1.rotation.pitch + p2.rotation.pitch)
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

def pose_to_str(p,separator:str ="\n")->str:
    if type(p) == Pose or type(p) == PoseTimestamp:
        p = p.to_dict()
    angles = p["angles"]
    pres = 2
    return "A1: "+str(round(angles[0],pres))+separator+"A2: "+str(round(angles[1],pres))+separator+"A3: "+str(round(angles[2],pres))+separator+"A4: "+str(round(angles[3],pres))+separator+"A5: "+str(round(angles[4],pres))+separator+"A6: "+str(round(angles[5],pres))

def pose_to_list(p)->list: 
    if type(p) == Pose or type(p) == PoseTimestamp:
        p = p.to_dict()
    return p["angles"]

def position_to_str(p,separator:str ="\n",simple:bool = False)->str:
    if type(p) == Position or type(p) == PositionTimestamp:
        p = p.to_dict()
    pos = p["point"]
    rot = p["rotation"]
    pres = 3
    x = str(round(1000*pos["x"],pres))
    y = str(round(1000*pos["y"],pres))
    z = str(round(1000*pos["z"],pres))
    roll = str(round(rot["roll"],pres))
    pitch = str(round(rot["pitch"],pres))
    yaw = str(round(rot["yaw"],pres))
    cs = [x,y,z,roll,pitch,yaw]
    ns = ["X: ","Y: ","Z: ","Rx: ","Ry: ","Rz: "]
    
    ret = ""
    for i in range(6):
        pre = ns[i]
        if simple: pre=""
        sep = separator
        ret+= pre+cs[i]+sep
    
    return ret

def position_to_p3d(p)->Point3D:
    if type(p) == Position or type(p) == PositionTimestamp:
        p = p.to_dict()
    pos = p["point"]
    rot = p["rotation"]
    return Point3D (pos["x"],pos["y"],pos["z"],_roll= rot["roll"],_pitch= rot["pitch"],_yaw= rot["yaw"])

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

#----------------------------------------------------------


def fullsum(l:"list[QPointF]"):
    p = QPointF(0,0)
    for e in l: p+=e
    return p*(1/float(len(l)))

def filtr_gaussQF(ps:"list[QPointF]",wind:int):
    ps_f = []#ps[:wind-1]
    for i in range(wind,len(ps)-wind):
        p_f = fullsum(ps[i-wind:i+wind])
        ps_f.append(p_f)
    #ps_f+=ps[len(ps)-wind-1:]
    return ps_f

def qf_len(q:QPointF):
    return (q.x()**2 + q.y()**2)**0.5

def filtr_medianQF(ps:"list[QPointF]",wind:int,delt:float):
    ps_f = []#ps[:wind-1]
    for i in range(wind,len(ps)-wind):
        p_f = fullsum(ps[i-wind:i+wind])
        if qf_len(p_f - ps[i]) < delt:           
            ps_f.append(p_f)
        else:
            p = QPointF(ps[i].x(),ps[-1].y())
            ps_f.append(p)
        
    #ps_f+=ps[len(ps)-wind-1:]
    return ps_f

def diff_plot(plot:"list[QPointF]"):
    ps = []

    for i in range(len(plot)-1):

        dt = plot[i+1].x()-plot[i].x()
        dy = plot[i+1].y()-plot[i].y()
        if dt>0:
            ps.append(QPointF(plot[i].x(),dy/dt))
    return ps

def traj_to_plots(qs:list[Pose3D]):
    plots = []
    for i in range(6):        
        plot = []        
        for j in range(len(qs)):    
            plot.append(QPointF(qs[j].t,qs[j].angles[i]))           
        plots.append(plot)

    return plots

def traj_to_plots_ps(ps:list[Point3D]):    
    plot_x = [] 
    plot_y = []
    plot_z = []
    plot_xyz = [] 
    plot_a = []
    plot_b = []
    plot_c = []  
    d = 0     
    for i in range(len(ps)):            
        plot_x.append(QPointF(ps[i].t,ps[i].x))  
        plot_y.append(QPointF(ps[i].t,ps[i].y)) 
        plot_z.append(QPointF(ps[i].t,ps[i].z)) 
        
        plot_a.append(QPointF(ps[i].t,ps[i].roll)) 
        plot_b.append(QPointF(ps[i].t,ps[i].pitch)) 
        plot_c.append(QPointF(ps[i].t,ps[i].yaw)) 

        if i!=0:
            d += (ps[i]-ps[i-1]).magnitude()
        plot_xyz.append(QPointF(ps[i].t,d)) 

    plots = [plot_x,plot_y,plot_z,plot_xyz,plot_a,plot_b,plot_c]

    return plots

def plots_qs(plotter:Plotter,plots):
    i=0
    for plot in plots:            
        i+=1
        plotter.addPlot(plot,"q"+str(i),i,1)
        plot_dif = diff_plot(plot)
        plotter.addPlot(plot_dif,"v q"+str(i),i,2)
        plot_dif = diff_plot(plot_dif)
        #plot_dif = filtr_medianQF(plot_dif,10,0.2)
        #plot_dif = filtr_gaussQF(plot_dif,100)
        
        plotter.addPlot(plot_dif,"a q"+str(i),i,3)   

    return plotter


def plots_ps(plotter:Plotter,plots):
    koords = ["x","y","z","xyz","a","b","c"]
    i=0
    for plot in plots:            
        i+=1
        plotter.addPlot(plot,koords[i-1],i,4)
        plot_dif = diff_plot(plot)

        #plot_dif = filtr_medianQF(plot_dif,10,0.2)
        #plot_dif = filtr_gaussQF(plot_dif,100)
        plotter.addPlot(plot_dif,koords[i-1]+ " v",i,5)
        
        plot_dif = diff_plot(plot_dif)
        plotter.addPlot(plot_dif,koords[i-1]+ " a",i,6)   

    return plotter

#-----------------------------------------------------------------
def vel_to_st(vel:float):
    nT = 10000    #timer freq
    p = 4         #step mm
    rev = 200*16*10# - reduct steps per revol
    st = int((nT*p)/(vel*rev))
    vel = (nT*p)/(st*rev)
    return st,vel

def vel_to_st2(vel_nos:float,d_nos:float,d_syr:float):
    vel = (vel_nos*d_nos**2)/(d_syr**2)
    nT = 10000    #timer freq
    p = 4         #step mm
    rev = 200*16*10# - reduct steps per revol
    st = int((nT*p)/(vel*rev))
    vel = (nT*p)/(st*rev)
    return st,vel

#---------------------------json-------------------------

def load_file(file_in):
    list = []
    with open(file_in) as file:   
        list=json.load(file)
    return list

def save_file(list,file_in):
    with open(file_in, "w", encoding="utf-8") as file:
        json.dump(list, file,indent=5)

#----------------------------------------------------------

def timestamp_decod(time_s:str)->float:
    time_s = time_s.split("T")[-1]
    time_s = time_s.replace("Z","")
    date = time_s.split(":")
    secs = float(date[0])*3600+float(date[1])*60+float(date[2])
    return secs

def str_to_dict(s:str):
    json_acceptable_string = s.replace("\n", "").replace(" ", "").replace("'", "\"")
    return json.loads(json_acceptable_string)

def load_feedback(file)->list[Pose3D]:
    ps = []
    traj_l = load_file(file)
    for frame in traj_l:
        traj_d = str_to_dict(frame)
        p = Pose3D(toRad(traj_d['angles']))
        p.t = timestamp_decod(traj_d['timestamp'])
        ps.append(p)
    return ps

#-----------------------------------------------------------------------

def posit_time(ts,ps,t)->Point3D:

    pass

def sync_traj(ts1,ps1,ts2,ps2)->float:
    
    pass

def draw_plots_compare(plotter,qs1,ps1,qs2,ps2):

    plots_qs1 = traj_to_plots(qs1)
    plots_qs2 = traj_to_plots(qs2)

    one = False
    one = True
    two = False
    #two = True


    plots_ps1 = traj_to_plots_ps(ps1)
    plots_ps2 = traj_to_plots_ps(ps2)

    if one:
        plotter = plots_qs(plotter,plots_qs1)
        plotter = plots_ps(plotter,plots_ps1)
    if two:
        plotter = plots_qs(plotter,plots_qs2)
        plotter = plots_ps(plotter,plots_ps2)
    

    return plotter



#----------------------------------------------------------------------



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

class RobAnimThread(QtCore.QThread):
    plotter_signal = QtCore.pyqtSignal(int)

    def __init__(self,pulse_arm:"PulseApp",trajectory:"list[Point3D]",div:float):
        QtCore.QThread.__init__(self)   
        
        self.pulse_arm = pulse_arm
        self.timeDelt = 0.0003        
        self.traj = trajectory
        self.plots = None
        self.div = div
        self.qs = []
        self.start()  

    
    def run(self):
        vel = 0.03                #mm/s
        qs = []
        ts = []
        ps = self.traj
        dt = self.div*1e-3/vel
        t = 0.0
        i = 0
        for p in ps:
            p.t = t
            q = self.pulse_arm.draw_3d_rob_pos(p,self.pulse_arm.q_draw) 
            qs.append(q)
            t += dt
            i+=1
            if i%100==0:
                print(i,"/",len(ps))
            #sleep(self.timeDelt)
        
        #qs = filtr_gauss_list(qs,100)
            
        self.plots = traj_to_plots(qs)
        self.qs = qs
        self.plotter_signal.emit(0)

class RemoteControlThread(QtCore.QThread):
    prog_signal = QtCore.pyqtSignal(str) 
    def __init__(self,pulse_app:"PulseApp"):
        QtCore.QThread.__init__(self)  
        
        self.pulse_arm = pulse_app.pulse_robot
        self.pulse_app = pulse_app
        self.timeDelt = 0.001
        self.workmode = 0
        self.inp_mass = ""
        self.base = 0
        sock = socket.socket()
        sock.bind(('', 30006))
        sock.listen(1)
        self.conn, self.addr = sock.accept()
        print ('connected:', self.addr)
        self.start()   


    def run(self):
        while True:
            data = self.conn.recv(1024)
            #print(len(data))
            if len(data)>1:
                data = str(data.decode())
                data_in =data.split(" ")
                #print(data[0])
                #print(len(data_in))
                if len(data_in)==1:
                    if "a" in data:
                        print("Auto")
                        self.workmode = 1
                    elif "m" in data:
                        print("Manual")
                        self.workmode = 0
                    elif "s" in data:
                        print("Start")
                        #print(self.inp_mass)
                        print(len(self.inp_mass))
                        self.prog_signal.emit(self.inp_mass)
                        #self.pulse_app.text_prog_code.setText(self.inp_mass)
                        self.workmode = 3

                    elif "c" in data:
                        print("Clear")
                        self.inp_mass = ""
                    
                    elif "b" in data:
                        print("Base")
                        self.base = 1
                    elif "f" in data:
                        print("pos")
                        self.conn.send(self.pulse_arm.cur_posit.encode())
                        self.workmode = 0
                if len(data_in)>5:
                    #print("data add")
                    self.inp_mass+=data

                        

            

class RobPosThread(QtCore.QThread):
    def __init__(self,pulse_arm:PulseRobotExt, label:QLabel, slot):
        QtCore.QThread.__init__(self)   
        self.pulse_arm = pulse_arm
        self.label = label
        self.timeDelt = 0.001
        self.start()   
        self.writing = False
        self.feedback = []
        slot.connect(self.set_writing)

    def set_writing(self,val):
        self.writing = val

    def run(self):
        while True:
            
            #cur_posit = self.pulse_arm.get_position()

            #cur_posit = self.pulse_arm.get_position()

            #cur_posit_m = pulse_matrix_p(position_to_p3d(cur_posit))

            #cur_posit_m_comp = pulse_FK(pose_to_list(self.pulse_arm.get_pose()))

            #print(cur_posit_m_comp)

            #print(cur_posit_m)
            

            pose = self.pulse_arm.get_pose()
            
            if self.writing:
                self.feedback.append(str(pose))

            else:
                try:
                    motors = self.pulse_arm.status_motors()
                    position = self.pulse_arm.get_position()
                    self.pulse_arm.cur_posit = position_to_str(position," ",True)
                    self.label.setText("Joint position:\n"+pose_to_str(pose)+"\n\n"+"Cartesian position:\n"+position_to_str(position)+"\n\n"+motor_state_to_str(motors))  
                    pass               
                except BaseException:
                    pass
                #print(str(pose))
            

            #cur_posit_comp = position_from_matrix_pulse(cur_posit_m_comp)

            #print(Point3D.ToStringPulse(self.pulse_arm.tool,delim=", "),"\n",self.pulse_arm.get_tool_info())

            #p3d = p3d_cur_pulse(cur_posit_comp,self.pulse_arm.tool,self.pulse_arm.base)

            #print(pulse_matrix_p(position_to_p3d(cur_posit)))
            
        
            sleep(self.timeDelt)


host = "http://10.10.10.20:8081"


class ax(Enum):
    X = "X"
    Y = "Y"
    Z = "Z"
    U = "U"
    V = "V"
    W = "W"



class PulseApp(QtWidgets.QWidget):
    

    pulse_robot = None
    count = 0
    q_draw = [0,0,0,0,0,0,0]
    plotter:Plotter = None
    writing_signal = QtCore.pyqtSignal(bool)

    
    

    def __init__(self, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.load_settings()
        self.setWindowTitle("Интерфейс Pulse")
        self.resize(1750, 1000)
        self.build()  

        self.plotter = Plotter(self)
        print(vel_to_st2(10,1,20.1))
        
        
    

        
    def draw_3d_rob_pos(self,p3d:Point3D,rob_draw:list):
        q = calc_inverse_kinem_pulse(p3d)[1]  
        q.t = p3d.t
        #print(q)     
        return self.draw_3d_rob(rob_draw,q)

    def draw_rob(self):
        L1 = 0.2311
        L2 = 0.45
        L3 = 0.37
        L4 = 0.1351
        L5 = 0.1825
        L6 = 0.1325
        l = [L1,L2,L3,L4,L5,L6]
        q_draw = []
        q_draw.append(self.viewer3d.addLines_ret([Point3D(0,0,0),Point3D(0,-l[0],0)],1,0,0,1))
        q_draw.append(self.viewer3d.addLines_ret([Point3D(0,0,0),Point3D(l[1],0,0)],1,0,0,1))      
        q_draw.append(self.viewer3d.addLines_ret([Point3D(0,0,0),Point3D(l[2],0,0)],1,0,0,1))      
        q_draw.append(self.viewer3d.addLines_ret([Point3D(0,0,0),Point3D(0,-l[3],0)],1,0,0,1))
        q_draw.append(self.viewer3d.addLines_ret([Point3D(0,0,0),Point3D(0,l[4],0)],1,0,0,1))
        q_draw.append(self.viewer3d.addLines_ret([Point3D(0,0,0),Point3D(0,0,-l[5])],1,0,0,1))
        return q_draw
    
    def draw_rob3d(self):
        L1 = 0.2311
        L2 = 0.45
        L3 = 0.37
        L4 = 0.1351
        L5 = 0.1825
        L6 = 0.1325
        l = [L1,L2,L3,L4,L5,L6]
        base1 = self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\0.STL")
        base2 = self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\1.STL")
        self.q_draw = []
        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\2.STL"))
        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\3.STL"))     
        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\4.STL"))      
        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\5.STL"))
        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\6.STL"))
        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\7.STL"))

        self.viewer3d.setMatr_off(pulse_matrix(0,0,0,np.pi/2,0,0),base1)
        self.viewer3d.setMatr_off(pulse_matrix(0,0,0,np.pi/2,0,0),base2)
        self.viewer3d.setMatr_off(pulse_matrix(0,-L1+0,0,0,np.pi,0),self.q_draw[0])

        self.viewer3d.setMatr_off(np.dot(np.dot(pulse_matrix(0,0,0,0,0,np.pi/2),pulse_matrix(0,-L1-L2+0,0,0,0,0)),pulse_matrix(0,0,0,0,np.pi,0)),self.q_draw[1])
        self.viewer3d.setMatr_off(np.dot(np.dot(pulse_matrix(0,0,0,0,0,np.pi/2),pulse_matrix(0,-L1-L2-L3+0,0,0,0,0)),pulse_matrix(0,0,0,0,np.pi,0)),self.q_draw[2])

        self.viewer3d.setMatr_off(np.dot(np.dot(pulse_matrix(0,0,0,np.pi/2,0,0),pulse_matrix(0,-L1-L2-L3,L4+0,0,0,0)),pulse_matrix(0,0,0,0,0,0)),self.q_draw[3])

        self.viewer3d.setMatr_off(np.dot(np.dot(pulse_matrix(0,0,0,0,np.pi,np.pi),pulse_matrix(0,-L1-L2-L3-L5,L4+0,0,0,0)),pulse_matrix(0,0,0,0,0,0)),self.q_draw[4])

        self.viewer3d.setMatr_off(np.dot(np.dot(pulse_matrix(0,0,0,np.pi,0,0),pulse_matrix(0,-L1-L2-L3-L5,L4+L6,0,0,0)),pulse_matrix(0,0,0,0,0,0)),self.q_draw[5])

        self.q_draw.append(self.viewer3d.addModel_ret(r"C:\Users\1\Desktop\misis\in situ printer\rozum\lowres\t1.STL"))

        self.viewer3d.setMatr_off(pulse_matrix(0,0,0,np.pi/2,-np.pi/4,0),self.q_draw[6])

        self.draw_3d_rob(self.q_draw,[0,0,0,0,0,0])

    def draw_line_rob(self,q_draw:list,q:Pose3D):
        solv_pms = comp_matrs_ps(q)
        for i in range(6):
            self.viewer3d.setMatr(solv_pms[i],q_draw[i])

    def draw_3d_rob(self,q_draw:list,q:Pose3D):
        solv_pms = comp_matrs_ps(q)
        for i in range(len(q_draw)):
            if i < len(q.angles):
                self.viewer3d.setMatr(solv_pms[i],q_draw[i])
            else:
                pass
                self.viewer3d.setMatr(solv_pms[len(q.angles)-1],q_draw[i])
        return q



    def start_anim_robot(self):
        p =  self.settins_pulse.start_points["sp_1303_1"]
        base =  self.settins_pulse.bases["bs0603"]
        base_p =  pos_dict_to_point3d(base)
        p1 =  pos_dict_to_point3d(p)
        base_m = pulse_matrix_p(base_p)
        p1_m = pulse_matrix_p(p1)
        base_m_inv = np.linalg.inv(base_m)
        p_base_m = np.dot(base_m_inv,p1_m)
        p1 = position_from_matrix_pulse(p_base_m)
        #p1 = Point3D(-0.3748,0.358,0.25,_roll=-1.515,_pitch=1.515,_yaw=-0.757)
        
        traj = filtr_dist(parse_g_code_pulse(self.text_prog_code.toPlainText()),0.3)
        #self.viewer3d.addLines_ret(traj,1,1,0,0.4)
        traj_d = 0.01
        traj = blend_lines(traj,1.4,traj_d)
        #self.viewer3d.addLines_ret(traj,1,1,0,0.4)

        traj_1 = comp_traj_to_anim_2( traj,traj_d)
        self.viewer3d.addLines_ret(traj_1,1,1,0,0.2)
        traj_p = Point3D.addList(Point3D.mulList(traj_1,1e-3),p1)
        

        self.thr = RobAnimThread(self,traj_p,traj_d)
        self.thr.plotter_signal.connect(self.test_plot)

    def test_plot(self):
        p1 = Point3D(-0.3748,0.358,0.25,_roll=-1.515,_pitch=1.515,_yaw=-0.757)
        qs = self.thr.qs

        ps = []
        for q in qs:
            ps.append( calc_forward_kinem_pulse(q,True))
        
        ps = Point3D.mulList(Point3D.addList(ps,-p1),1e3)  

        #print(len(ps),ps[0].ToString())
        self.viewer3d.addLines_ret(ps,0,1,1,0.2) 

        self.plotter = plots_qs(self.plotter,self.thr.plots)
        self.plotter.show()

    def test1(self):
        qs = load_feedback("feedback.json")
        print("LEN PLOT: ",len(qs))
        #qs = filtr_gauss_list(qs,100)
        plots = traj_to_plots(qs)
        self.plotter = plots_qs(self.plotter,plots)

        p =  self.settins_pulse.start_points["sp_1303_1"]
        base =  self.settins_pulse.bases["bs0603"]
        base_p =  pos_dict_to_point3d(base)
        p1 =  pos_dict_to_point3d(p)
        base_m = pulse_matrix_p(base_p)
        p1_m = pulse_matrix_p(p1)
        base_m_inv = np.linalg.inv(base_m)
        p_base_m = np.dot(base_m_inv,p1_m)
        p_b = position_from_matrix_pulse(p_base_m)

        ps = []
        for q in qs:
            ps.append(calc_forward_kinem_pulse(q))

        plots = traj_to_plots_ps(ps)
        self.plotter = plots_ps(self.plotter,plots)


        
        ps = Point3D.mulList(Point3D.addList( Point3D.mulPoint(ps,base_p),-p1),1e3)  
        print(len(ps),ps[0].ToString())
        traj = filtr_dist(parse_g_code_pulse(self.text_prog_code.toPlainText()),0.3)
        traj_d = 0.01
        traj = blend_lines(traj,1.4,traj_d)
        traj_1 = comp_traj_to_anim_2( traj,traj_d)

        self.viewer3d.addLines_ret(traj_1,1,1,0,0.2)
        self.viewer3d.addLines_ret(ps,0,1,1,0.2) 
        self.plotter.show()

    def test2(self):
        qs_real = load_feedback("feedback.json")
        prog = parse_g_code_pulse( self.text_prog_code.toPlainText())
        p_st =  pos_dict_to_point3d(self.settins_pulse.start_points["sp_1303_1"])
        base =  pos_dict_to_point3d(self.settins_pulse.bases["bs0603"])

        qs_real,ps_real,qs_model,ps_model = compare_traj_pulse(qs_real,prog,base,p_st,blend=1.2,traj_divide=0.3)
        ps_model = Point3D.addList(Point3D.mulList(Point3D.addList(Point3D.mulPoint(ps_model,base),-p_st),1e3),Point3D(x=0,y=-30))
        ps_real =  Point3D.addList(Point3D.mulList(Point3D.addList(Point3D.mulPoint(ps_real,base),-p_st),1e3),Point3D(x=0,y=-30))

        self.viewer3d.addLines_ret(ps_model,1,1,0,0.2)
        self.viewer3d.addLines_ret(ps_real,0,1,0,1.2)
        draw_plots_compare(self.plotter,qs_real,ps_real,qs_model,ps_model)
        self.plotter.show()
 
    def build(self):
        self.build_connection()
        self.build_position()
        self.build_coords()
        self.build_settings()
        self.build_config()
        self.build_progr()

        self.build_connection_kuka()

    def build_connection(self):

        self.viewer3d = GLWidget(self)
        self.viewer3d.setGeometry(QtCore.QRect(350, 10, 600, 600))
        self.viewer3d.draw_start_frame(10.)
        #self.draw_rob3d()

        self.but_connect_robot = QPushButton('Подключиться', self)
        self.but_connect_robot.setGeometry(QtCore.QRect(100, 100, 140, 30))
        self.but_connect_robot.clicked.connect(self.connect_robot)

        self.but_disconnect_robot = QPushButton('Отключиться', self)
        self.but_disconnect_robot.setGeometry(QtCore.QRect(100, 140, 140, 30))
        self.but_disconnect_robot.clicked.connect(self.disconnect_robot)

        self.but_remote_connect_robot = QPushButton('Удалённое управление', self)
        self.but_remote_connect_robot.setGeometry(QtCore.QRect(250, 140, 140, 30))
        self.but_remote_connect_robot.clicked.connect(self.start_remote_control)

        self.but_start_anim_robot = QPushButton('Анимац', self)
        self.but_start_anim_robot.setGeometry(QtCore.QRect(100, 60, 140, 30))
        self.but_start_anim_robot.clicked.connect(self.start_anim_robot)

        self.but_test_plot = QPushButton('Тест', self)
        self.but_test_plot.setGeometry(QtCore.QRect(100, 20, 140, 30))
        self.but_test_plot.clicked.connect(self.test2)

        self.but_start_writing = QPushButton('Начать запись', self)
        self.but_start_writing.setGeometry(QtCore.QRect(250, 20, 140, 30))
        self.but_start_writing.clicked.connect(self.start_writing)

        self.but_stop_writing = QPushButton('Остановить запись', self)
        self.but_stop_writing.setGeometry(QtCore.QRect(250, 60, 140, 30))
        self.but_stop_writing.clicked.connect(self.stop_writing)

    def start_remote_control(self):
        self.rem_thr = RemoteControlThread(self)
        self.rem_thr.prog_signal.connect(self.set_prog_text, QtCore.Qt.QueuedConnection)


    def set_prog_text(self,val):
        self.text_prog_code.setText(val)

    def start_writing(self):
        self.writing_signal.emit(True)

        
    def stop_writing(self):
        self.writing_signal.emit(False)
        save_file( self.coords_thread.feedback,"feedback.json")

    def connect_robot(self):
        self.pulse_robot = PulseRobotExt(host)
        self.coords_thread = RobPosThread(self.pulse_robot,self.lab_coord,self.writing_signal)

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

        axis = [ax.X,ax.Y,ax.Z,ax.U,ax.V,ax.W]
        i=0
        for axe in axis:           
            self.add_axis_buttons(axe,QtCore.QRect(100+40*i, 340, 30, 30),self.axis_jog)
            self.add_axis_buttons(axe,QtCore.QRect(100+40*i, 440, 30, 30),self.axis_move)
            i+=1

        """self.add_axis_buttons(ax.X,QtCore.QRect(100, 340, 30, 30),self.axis_jog)
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
        self.add_axis_buttons(ax.W,QtCore.QRect(300, 440, 30, 30),self.axis_move)"""

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
        step = 0.1
        x,y,z,Rx,Ry,Rz = self.mask_from_button(but.text())        
        position_delt = position([step*x, step*y, step*z], [step*Rx, step*Ry, step*Rz])
        pos,rot = position_sum2(self.pulse_robot.get_position(),position_delt)
        pos_rel = position(pos,rot)

        self.pulse_robot.robot.set_position(pos_rel , velocity=vel, acceleration=acs)
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
        self.buffer_positions.append(Position(start_point["point"],start_point["rotation"]).to_dict())
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
        self.but_create_tool.setGeometry(QtCore.QRect(550, 720, 140, 30))
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

        self.but_comp_base = QPushButton('Расч базу', self)
        self.but_comp_base.setGeometry(QtCore.QRect(100, 760, 140, 30))
        self.but_comp_base.clicked.connect(self.comp_base)

        self.lin_name_base = QLineEdit(self)
        self.lin_name_base.setGeometry(QtCore.QRect(400, 760, 140, 30))
        self.lin_name_base.setText("new_base")

    def comp_tcp_point(self):
        #ps = [self.settins_pulse.start_points["calib_1_1"],self.settins_pulse.start_points["calib_1_2"],self.settins_pulse.start_points["calib_1_3"],self.settins_pulse.start_points["calib_1_4"],self.settins_pulse.start_points["calib_1_5"]]        
        ps = self.buffer_positions
        tcp = calibrate_tcp_4p(ps)
        rot = self.settins_pulse.tools[self.combo_tools.currentText()]['tcp']['rotation']
        self.current_tool = tool_info(position([tcp[0][0],tcp[1][0],tcp[2][0]],[rot['pitch'],rot['roll'],rot['yaw']]))


    def comp_tcp_rotate(self):
        ps = self.buffer_positions
        tcp = self.settins_pulse.tools[self.combo_tools.currentText()]['tcp']['point']
        p = orient_tool_calibration(ps)
        self.current_tool = tool_info(position([tcp['x'],tcp['y'],tcp['z']],[p.roll,p.pitch,p.yaw]))


    def comp_base(self):
        ps = self.buffer_positions
        p = base_calibration(ps)
        self.current_base = position([p.x,p.y,p.z],[p.roll,p.pitch,p.yaw])
        

    
    def create_tool(self):
        name = self.lin_name_tool.text()
        self.settins_pulse.tools[name] = tool_info(position( [0,0,0],[0,0,0]),name).to_dict()
        self.save_settings()
        self.set_setting_items()


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


    def del_tool(self):
        del self.settins_pulse.tools[self.combo_tools.currentText()]
        self.save_settings()
        self.set_setting_items()

    def del_base(self):
        del self.settins_pulse.bases[self.combo_bases.currentText()]
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
        self.combo_tools.activated[str].connect(self.combo_tool_act)

        self.combo_bases = QComboBox(self)
        self.combo_bases.setGeometry(QRect(870, 680, 120, 30))       
        self.combo_bases.setCurrentIndex(0)
        self.combo_bases.activated[str].connect(self.combo_base_act)

        self.but_add_position_from_cur = QPushButton('Добавить в буффер', self)
        self.but_add_position_from_cur.setGeometry(QtCore.QRect(720, 560, 100, 30))
        self.but_add_position_from_cur.clicked.connect(self.add_position_from_cur)

        self.but_add_pose_from_cur = QPushButton('Добавить в буффер', self)
        self.but_add_pose_from_cur.setGeometry(QtCore.QRect(720, 600, 100, 30))
        self.but_add_pose_from_cur.clicked.connect(self.add_pose_from_cur)

        self.but_del_position = QPushButton('Уд', self)
        self.but_del_position.setGeometry(QtCore.QRect(820, 560, 40, 30))
        self.but_del_position.clicked.connect(self.del_position)

        self.but_del_pose = QPushButton('Уд', self)
        self.but_del_pose.setGeometry(QtCore.QRect(820, 600, 40, 30))
        self.but_del_pose.clicked.connect(self.del_pose)

        self.but_del_tool = QPushButton('У', self)
        self.but_del_tool.setGeometry(QtCore.QRect(820, 640, 40, 30))
        self.but_del_tool.clicked.connect(self.del_tool)

        self.but_del_base = QPushButton('У', self)
        self.but_del_base.setGeometry(QtCore.QRect(820, 680, 40, 30))
        self.but_del_base.clicked.connect(self.del_base)

        
        self.set_setting_items()

    def combo_tool_act(self,text):
        tool = self.get_cur_item_from_combo(self.combo_tools,self.settins_pulse.tools)
        self.cur_tool = tool_info(Position(tool['tcp']["point"],tool['tcp']["rotation"]))

        if self.pulse_robot is not None:  self.pulse_robot.change_tool_info(self.cur_tool)

    def combo_base_act(self,text):
        base = self.get_cur_item_from_combo(self.combo_bases,self.settins_pulse.bases)
        self.cur_base =  Position(base["point"],base["rotation"])


        #print(self.cur_base)
        
        if self.pulse_robot is not None: self.pulse_robot.change_base(self.cur_base)

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

        self.but_start_prog_abs = QPushButton('Исп. прог. абс', self)
        self.but_start_prog_abs.setGeometry(QtCore.QRect(1000, 800, 140, 30))
        self.but_start_prog_abs.clicked.connect(self.exec_prog_arm_abs)

        self.but_start_prog_rel = QPushButton('Исп. прог. относ', self)
        self.but_start_prog_rel.setGeometry(QtCore.QRect(1000, 840, 140, 30))
        self.but_start_prog_rel.clicked.connect(self.exec_prog_arm_rel)

        self.but_start_prog_abs_xyz = QPushButton('Исп. прог. абс xyz', self)
        self.but_start_prog_abs_xyz.setGeometry(QtCore.QRect(1000, 880, 140, 30))
        self.but_start_prog_abs_xyz.clicked.connect(self.exec_prog_arm_abs_xyz)

        self.but_stop_robot = QPushButton('Остановить', self)
        self.but_stop_robot.setGeometry(QtCore.QRect(1000, 940, 140, 30))
        self.but_stop_robot.clicked.connect(self.stop_ex_robot)

        self.text_prog_code = QTextEdit(self)
        self.text_prog_code.setGeometry(QtCore.QRect(1150, 560, 500, 400))

        self.text_prog_code.setText("G1 X1 Y1\nG1 X1 Y30\nG1 X30 Y30")


    def set_cur_work_pose(self):
        self.cur_work_pose = self.get_cur_item_from_combo(self.combo_work_poses,self.settins_pulse.work_poses)
        if self.cur_work_pose is not None:
            self.pulse_robot.set_pose(Pose(self.cur_work_pose["angles"]),5)
            #self.pulse_robot.await_stop()

    def set_cur_start_point(self):
        self.cur_start_point = self.get_cur_item_from_combo(self.combo_start_points,self.settins_pulse.start_points)
        if self.cur_start_point is not None:
            self.pulse_robot.robot.set_position(Position(self.cur_start_point["point"],self.cur_start_point["rotation"]),velocity=5,acceleration=1)
            #self.pulse_robot.await_stop()

    def exec_prog_arm_rel(self):

        positions = self.generate_traj()
        vel1 = 2
        vel2 = 0.02

        acs1 = 50
        acs2 = 0.05
        #print(positions)
        vel = vel1
        acs = acs1
        #self.pulse_robot.robot.set_position(positions[0],velocity=vel,acceleration=acs,motion_type=MT_LINEAR)
        
        vel = vel2
        acs = acs2
        print("len all",len(positions))
        dn = 950
        linear_motion_parameters = LinearMotionParameters(interpolation_type=InterpolationType.BLEND,velocity=vel,acceleration=acs)
        for i in range(int(len(positions)/dn)+1):
            self.pulse_robot.robot.set_position(positions[dn*i],velocity=vel1,acceleration=acs1,motion_type=MT_LINEAR)        
            linear_motion_parameters = LinearMotionParameters(interpolation_type=InterpolationType.BLEND,velocity=vel2,acceleration=acs2)
            print("load",dn)
            if len(positions)>dn*(i+1)+1:
                self.pulse_robot.robot.run_linear_positions(positions[dn*i:dn*(i+1)],linear_motion_parameters)
            else:
                self.pulse_robot.robot.run_linear_positions(positions[dn*i:],linear_motion_parameters)


    def exec_prog_arm_abs_xyz(self):
        
        #self.apply_settings_to_robot()
        
        #self.pulse_robot.set_position(Position(self.cur_start_point["point"],self.cur_start_point["rotation"]),velocity=vel,acceleration=acs,motion_type=MT_LINEAR)

        positions = self.generate_traj_abs_xyz()
        
        print(positions)
        vel = 50
        acs = 0.1
        self.pulse_robot.robot.set_position(positions[0],velocity=vel,acceleration=acs,motion_type=MT_LINEAR)
        
        vel = 0.03
        acs = 0.1
        linear_motion_parameters = LinearMotionParameters(interpolation_type=InterpolationType.BLEND,velocity=vel,acceleration=acs)
        self.pulse_robot.robot.run_linear_positions(positions[:800],linear_motion_parameters)


    def exec_prog_arm_abs(self):
        #self.apply_settings_to_robot()
        
        positions = self.generate_traj_abc()
        vel = 0.01
        acs = 0.1
        linear_motion_parameters = LinearMotionParameters(interpolation_type=InterpolationType.BLEND,velocity=vel,acceleration=acs)
        self.pulse_robot.robot.set_position(positions[0],velocity=20,acceleration=1,motion_type = MT_LINEAR)
        #print(positions)
        self.pulse_robot.robot.run_linear_positions(positions,linear_motion_parameters)

    


    def stop_ex_robot(self):
        self.pulse_robot.stop()
        self.pulse_robot.recover()


    def traj_prep(self,ps:list[Point3D],p_off=Point3D()):
        """ps: mm, p_off: m"""
        points = []
        positions = []
        for i in range(len(ps)): 
            #print(ps[i].ToString())              
            p = [p_off.x+0.001*ps[i].x,p_off.y+0.001*ps[i].y,p_off.z+0.001*ps[i].z]
            r = [p_off.roll+ps[i].roll,p_off.pitch+ps[i].pitch,p_off.yaw+ps[i].yaw]            
            pos:Position = position(p,r,blend=0.0001)  
            if i>2:
                
                if self.dist(p,points[-1])>0.003:
                    p1 = Point3D(p[0],p[1],p[2])
                    p2 = Point3D(points[-1][0],points[-1][1],points[-1][2])
                    p3 = Point3D(points[-2][0],points[-2][1],points[-2][2])
                    v1 = p2-p1
                    v2 = p3-p2
                    alph = Point3D.ang(v1,v2)
                    if abs(abs(alph)%np.pi)>0.03:                                           
                        positions.append(pos)
                        points.append(p)
            else:
                positions.append(pos)
                points.append(p)
        #print(ps)
        print("#######################################")
        return positions

#-----------------------------------------------------------------------------------
    def generate_traj(self):
        self.cur_start_point = self.get_cur_item_from_combo(self.combo_start_points,self.settins_pulse.start_points)

        ps = parse_g_code(self.text_prog_code.toPlainText())
        p_off = pos_dict_to_point3d(self.cur_start_point)
        p_off.z+=5*1e-3
        positions = self.traj_prep(ps,p_off)

        
        return positions

    def generate_traj_abs_xyz(self):
        self.cur_start_point = self.get_cur_item_from_combo(self.combo_start_points,self.settins_pulse.start_points)
        ps = parse_g_code(self.text_prog_code.toPlainText())
        start_point = self.cur_start_point["point"]
        start_rot =  self.cur_start_point["rotation"]

        points = []
        p = [start_point["x"],start_point["y"],start_point["z"]]
        r = [start_rot["roll"],start_rot["pitch"],start_rot["yaw"]]
        pos = position(p,r)
        points.append(p)
        positions = []
        for i in range(len(ps)):               
            p = [0.001*ps[i].x,0.001*ps[i].y,0.001*ps[i].z]
            r = [start_rot["roll"],start_rot["pitch"],start_rot["yaw"]]
            
            pos = position(p,r,blend=0.0001) 
            if self.dist(p,points[-1])>0.003:
                #print(pos)
                positions.append(pos)
                points.append(p)

        #for i in range(len(positions)):
            #print(i," ",positions[i])
        
        return positions
#-----------------------------------------------------------------------------------
    def generate_traj_abc(self):        
        ps = parse_g_code_pulse(self.text_prog_code.toPlainText())
        p_off = Point3D(z=0.5*1e-3)
        #p_off.z+=5*1e-3
        positions = self.traj_prep(ps,p_off)
        
        return positions


    def dist(self,p1,p2):
        if type(p1)==Position:
            return math.sqrt((p1.point.x-p2.point.x)**2+(p1.point.y-p2.point.y)**2+(p1.point.z-p2.point.z)**2)
        else:
            return math.sqrt((p1[0]-p2[0])**2+(p1[1]-p2[1])**2+(p1[2]-p2[2])**2)

#--------------------------------------------------------------------------------
    kuka_robot = None

    def build_connection_kuka(self):
        self.but_connect_kuka = QPushButton('Подключиться', self)
        self.but_connect_kuka.setGeometry(QtCore.QRect(1000, 100, 140, 30))
        self.but_connect_kuka.clicked.connect(self.connect_kuka)

        self.but_disconnect_kuka = QPushButton('Отключиться', self)
        self.but_disconnect_kuka.setGeometry(QtCore.QRect(1000, 140, 140, 30))
        self.but_disconnect_kuka.clicked.connect(self.disconnect_kuka)    

        self.but_resiev_kuka = QPushButton('Принять', self)
        self.but_resiev_kuka.setGeometry(QtCore.QRect(1000, 240, 140, 30))
        self.but_resiev_kuka.clicked.connect(self.resiev_kuka)

        self.but_send_kuka = QPushButton('Отправить', self)
        self.but_send_kuka.setGeometry(QtCore.QRect(1000, 280, 140, 30))
        self.but_send_kuka.clicked.connect(self.send_kuka)

        self.text_mes_kuka = QTextEdit(self)
        self.text_mes_kuka.setGeometry(QtCore.QRect(1150, 240, 500, 30))

    def connect_kuka(self):
        self.kuka_robot = KukaRobot()
        self.kuka_robot.connect()

    def disconnect_kuka(self):
        self.kuka_robot.send('q\n')
        self.kuka_robot.close()

    def resiev_kuka(self):
        self.kuka_robot.send('f\n')
        sleep(0.01)
        mes = self.kuka_robot.resieve()
        print(mes)
        self.text_mes_kuka.setText(mes)

    def send_kuka(self):
        self.text_mes_kuka.update()
        mes = self.text_mes_kuka.toPlainText()+' \n'
        self.kuka_robot.send(mes)




 

if __name__ == '__main__':    
    app = QApplication(sys.argv)
    pulse = PulseApp()
    pulse.show()
    sys.exit(app.exec_())