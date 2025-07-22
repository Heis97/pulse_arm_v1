from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_JOINT, MT_LINEAR,jog,create_box_obstacle,LinearMotionParameters,InterpolationType,tool_info
from pdhttp import Position,Point,Rotation,Pose,MotorStatus,PoseTimestamp,PositionTimestamp,ToolInfo
from PulseUtil import *
from api.robot_api_v2 import *
from api.robot_api import *
from threading import Thread
#from api.robot_api_rc5.API.rc_api import *  # type: ignore
from API38.rc_api import *
from enum import Enum
import datetime


def posit_to_list(posit:Position):
    p3d = position_to_p3d(posit)
    return p3d_to_list(p3d)

def posit_to_list_rc5(posit:Position):
    p3d = position_to_p3d(posit)
    return p3d_to_list_rc5(p3d)

def p3d_to_list(p3d:Point3D):
    return [p3d.x,p3d.y,p3d.z,p3d.roll,p3d.pitch,p3d.yaw]

def p3d_to_list_rc5(p3d:Point3D):
    return [p3d.x,p3d.y,p3d.z,p3d.pitch,p3d.roll,p3d.yaw]


def pos_v3_to_v1(p_l):
    p = Point3D(p_l[0],p_l[1],p_l[2], _roll= p_l[3],_pitch= p_l[4],_yaw= p_l[5])
    m = pulse_matrix_p_v3(p)
    #print("m_v3: ",m)
    p = position_from_matrix_pulse(m)
    return [p.x,p.y,p.z,p.roll,p.pitch,p.yaw]

def pos_v1_to_v3(p_l):
    p = Point3D(p_l[0],p_l[1],p_l[2], _roll= p_l[3],_pitch= p_l[4],_yaw= p_l[5])
    m = pulse_matrix_p(p)
    p = p3d_from_matrix_pulse_v3(m)
    return [p.x,p.y,p.z,p.roll,p.pitch,p.yaw]


host_old = "http://10.10.10.20:8081"
#host_v3 = "192.168.0.50"#cyto 
host_v3 = "192.168.10.71"#misis
#host_v36 = "192.168.0.10"#old rc5
host_v36 = "10.10.10.3"#new rc5


class PulseRobotExt(object):
    zg = False
    robot: RobotPulse
    robot_v3: RobotAPI
    robot_v36: RobotApi
    base: Point3D = Point3D(0,0,0)
    tool: Point3D= Point3D(0,0,0)
    cur_posit :str = ""
    buf_pos_3d : list[Point3D] = []
    buf_len:int = 20
    cur_posit_3d : Point3D = None
    cur_prog_3d : list[Point3D] = None
    cur_i_prog:int = 0
    cur_progr_line:float = 0
    rem_thr = None
    controller_v3:RobotType = None
    pos_v3 = None


    def update_buf(self):
        self.buf_pos_3d.append(self.cur_posit_3d)
        if len(self.buf_pos_3d)> self.buf_len:
            del self.buf_pos_3d[0]

    def __init__(self,controller_v3:RobotType = RobotType.pulse_v3) -> None:
        self.controller_v3 = controller_v3
        if self.controller_v3 is RobotType.pulse_v3:   
            print("connect v3")    
            self.robot_v3 = RobotAPI(host_v3)
            self.robot_v3.init_robot()
        if self.controller_v3 is RobotType.pulse_v36:   
            print("connect v36")    
            self.robot_v36 = RobotApi(host_v36,enable_logger=True,log_std_level=logging.DEBUG,enable_logfile=True, logfile_level=logging.INFO)
            #self.robot_v36.controller_state.set('off')
            self.robot_v36.controller_state.set('run')
            #print(self.robot_v36.controller_state.get())
            #self.robot_v36.
            #print("self.robot_v36.get_robot_info()")
            #print(self.robot_v36.get_robot_info())
            #print("________________________-")
        else:
            self.robot = RobotPulse(host_old)
                


    def get_pose(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            return Pose(self.robot_v3.get_act_pos_q())
        if self.controller_v3 is RobotType.pulse_v36:  
            #print(self.robot_v36.motion.joint.get_actual_position())
            return Pose(self.robot_v36.motion.joint.get_actual_position())
        else:
            #pose_deg = 

            return self.robot.get_pose()
    
    def get_position(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            list_pos = self.robot_v3.get_act_pos_cartesian()

            #angles = self.get_pose().angles
            #pose = Pose3D(angles)
            #position_c:Point3D = q_to_p(pose,False,RobotType.pulse_v3)
            
            #list_pos =  [ position_c.x,position_c.y,position_c.z,position_c.roll,position_c.pitch,position_c.yaw]
            #print("list1: ",list_pos)
            list_pos =  pos_v3_to_v1(list_pos)
            #print("list2: ",list_pos)
            return position(list_pos[0:3],list_pos[3:6])
        if self.controller_v3 is RobotType.pulse_v36:  
            list_pos = self.robot_v36.motion.linear.get_actual_position("rad")
            #print("list1: ",list_pos)
            #list_pos =  pos_v3_to_v1(list_pos)
            #print("list2: ",list_pos)
            return position(list_pos[0:3],list_pos[3:6])
        else:
            return self.robot.get_position()
    
    def stop(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            return self.robot_v3.hold()
            #return self.robot_v3.stby()
        if self.controller_v3 is RobotType.pulse_v36:  
            return self.robot_v36.motion.mode.set('hold')
        else:
            
            
            #return self.robot.zg_on()
            return self.robot.stop()
    def off(self):
        if self.controller_v3 is RobotType.pulse_v36:  
            self.robot_v36.controller_state.set('off')
    def recover(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            pass
            #return self.robot_v3.stby()
        else:
            return self.robot.recover()
    
    def set_position(self,_t_p:Position,
                     _velocity = None,
                     _acceleration = None,
                     _tcp_max_velocity = None, 
                     _motion_type: str = MT_LINEAR):
        if self.controller_v3 is RobotType.pulse_v3:  
            t = Thread(target=self.run_position_v3(_t_p,_velocity,_acceleration,_tcp_max_velocity,_motion_type))
            t.start()
            return 
        if self.controller_v3 is RobotType.pulse_v36:  
            self.run_position_v36(_t_p,_velocity,_acceleration,_tcp_max_velocity,_motion_type)
            return 
        if self.controller_v3 is RobotType.pulse_v1:
            return self.robot.set_position(target_position=_t_p,
                                       velocity=_velocity,
                                       acceleration=_acceleration,
                                       #tcp_max_velocity=_tcp_max_velocity,
                                         motion_type=_motion_type)
    
    def set_pose(self,target_pose:Pose,
                speed= None,
                velocity = None,
                acceleration = None,
                tcp_max_velocity = None,
                motion_type: str = MT_JOINT):
        if self.controller_v3 is RobotType.pulse_v3:  
            """t = Thread(target=self.run_pose_v3(target_pose,speed,
                velocity,
                acceleration,
                tcp_max_velocity,
                motion_type))
            t.start()"""
            self.run_pose_v3(target_pose,speed,
                velocity,
                acceleration,
                tcp_max_velocity,
                motion_type)
            return 
        if self.controller_v3 is RobotType.pulse_v36:  
            """t = Thread(target=self.run_pose_v3(target_pose,speed,
                velocity,
                acceleration,
                tcp_max_velocity,
                motion_type))
            t.start()"""
            self.run_pose_v36(target_pose,speed,
                velocity,
                acceleration,
                tcp_max_velocity,
                motion_type)
            return 
        else:
            return self.robot.set_pose(target_pose,speed,velocity ,acceleration, tcp_max_velocity,motion_type)
        #return self.robot.set_pose(target_pose,speed,velocity ,acceleration, tcp_max_velocity,motion_type)
    
    def run_position_v3(self,_t_p:Position,
                     _velocity = None,
                     _acceleration = None,
                     _tcp_max_velocity = None, 
                     _motion_type: str = MT_LINEAR):
        #pos v1_to_v3
        #print("1",self.robot_v3.get_act_pos_cartesian())
        #print("2",pos_v1_to_v3(posit_to_list(  _t_p)))
        self.robot_v3.move_l(pos_v1_to_v3(posit_to_list(  _t_p)),_velocity,_acceleration)
        self.robot_v3.run_wps()
        #self.robot_v3.await_motion()
        return
    
    def run_position_v36(self,_t_p:Position,
                     _velocity = None,
                     _acceleration = None,
                     _tcp_max_velocity = None, 
                     _motion_type: str = MT_LINEAR):
        self.robot_v36.motion.linear.add_new_waypoint(posit_to_list(_t_p),_velocity,_acceleration,orientation_units="rad")
        self.robot_v36.motion.mode.set('move')
        #self.robot_v36.motion.wait_waypoint_completion(0)
        return
    
    def run_pose_v3(self,target_pose:Pose,
                speed= None,
                velocity = None,
                acceleration = None,
                tcp_max_velocity = None,
                motion_type: str = MT_JOINT):
        if speed is None: speed = 0.1
        if acceleration is None: acceleration = 0.1
        self.robot_v3.move_j(target_pose.angles,speed,acceleration)
        self.robot_v3.run_wps()
        #self.robot_v3.colab_await_buffer(0)
        #self.robot_v3.a
        #self.robot_v3.await_motion()
        return
    def run_pose_v36(self,target_pose:Pose,
                speed= None,
                velocity = None,
                acceleration = None,
                tcp_max_velocity = None,
                motion_type: str = MT_JOINT):
        if speed is None: speed = 1
        if acceleration is None: acceleration = 0.5
        print(target_pose.angles)
        self.robot_v36.motion.joint.add_new_waypoint(target_pose.angles,None,speed,acceleration)
        self.robot_v36.motion.mode.set('move')
        self.robot_v36.motion.wait_waypoint_completion(0)
        return

    def run_positions_v3(self,positions: list[Position],
                                motion_parameters: LinearMotionParameters):
        for pos in positions: self.robot_v3.move_l(pos_v1_to_v3(posit_to_list(pos)),motion_parameters.velocity,motion_parameters.acceleration) 
        self.robot_v3.run_wps()
        #self.robot_v3.await_motion()

    def run_positions_v36(self,positions: list[Position],
                                motion_parameters: LinearMotionParameters):
        for pos in positions: self.robot_v36.motion.linear.add_new_waypoint(posit_to_list(pos),motion_parameters.velocity,motion_parameters.acceleration) 
        self.robot_v36.motion.mode.set('move')


    def run_linear_positions(self,positions: list[Position],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = positions_to_p3ds(positions)
        self.cur_i_prog = 0

        if self.controller_v3 is RobotType.pulse_v3:  
            t = Thread(target=self.run_positions_v3(positions,motion_parameters))
            t.start()
        else:
            return self.robot.run_linear_positions(positions,motion_parameters)
    
    """def run_linear_positions(self,positions: list[Position],ps: list[Point3D],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = ps
        self.cur_i_prog = 0
        if self.controller_v3 is RobotType.pulse_v3:  
            for pos in positions: self.robot_v3.move_l(posit_to_list(pos),motion_parameters.velocity,motion_parameters.acceleration) 
            return
            #return self.robot_v3.move_j(target_pose.angles,speed,acceleration)
        else:
            return self.robot.run_linear_positions(positions,motion_parameters)"""
    
    """def run_poses(self,positions: list[Pose],ps: list[Point3D],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = ps
        self.cur_i_prog = 0
        return self.robot.run_poses(positions,motion_parameters)"""
    
    def change_base(self,base_position):
        self.base = pos_dict_to_point3d(base_position.to_dict())
        if self.controller_v3 is RobotType.pulse_v3:  
            return
        else:
            return self.robot.change_base(base_position)
    
    def get_base(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            return
        else:
            return self.robot.get_base()
    
    def change_tool_info(self,new_tool_info:ToolInfo):
        self.tool = pos_dict_to_point3d(new_tool_info.tcp.to_dict())
        if self.controller_v3 is RobotType.pulse_v3:  
            self.robot_v3.set_tool(pos_v1_to_v3(p3d_to_list( self.tool)))
            self.robot_v3.hold()
            return 
        else:
            return self.robot.change_tool_info(new_tool_info)
        #return self.robot.change_tool_info(new_tool_info)
    
    def get_tool_info(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            return
        else:
            return self.robot.get_tool_info()
        #return self.robot.get_tool_info()
    
    def freeze(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            self.robot_v3.hold()
            return
        else:
            return self.robot.freeze()
    
    def jogging(self,x,y,z,rx,ry,rz):
        if self.controller_v3 is RobotType.pulse_v3:  
            speed = 0.1
            acs = 0.2
            return self.robot_v3.set_jog_param([0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[speed]*6,[acs]*6,[acs]*6)
        else:
            return self.robot.jogging(jog(x,y,z,rx,ry,rz))
    
    def await_stop(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            return
        else:
            return self.robot.await_stop()
    
    def relax(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            if self.zg==False:
                self.robot_v3.zg(True)
                self.zg = True
            else:
                self.robot_v3.zg(False)
                self.zg = False
            return
        else:
            return self.robot.zg_on()
            #return self.robot.relax()
    
    def status_motors(self):
        if self.controller_v3 is RobotType.pulse_v3:  
            return
        else:
            return self.robot.status_motors()

    def current_progress_prog(self):

        dist_err = 0.3
        ang_err = 0.1
        if self.cur_prog_3d is None or self.cur_posit_3d is None or len(self.buf_pos_3d)<2:
            return None
        ds = []
        for i in range(1,len(self.cur_prog_3d)):
            d = self.cur_posit_3d.dist_to_sect(self.cur_prog_3d[i-1], self.cur_prog_3d[i])
            ds.append((d,i))

        ds.sort(reverse=False)
        
        #self.cur_i_prog = self.correct_cur_progress(ds,dist_err,ang_err)
        if ds[0][1]-self.cur_i_prog == 1:
            self.cur_i_prog+=1
        prog_line = self.cur_posit_3d.part_of_sect(self.cur_prog_3d[self.cur_i_prog-1], self.cur_prog_3d[self.cur_i_prog])
        
        self.cur_progr_line = prog_line

    
    def correct_cur_progress(self,sort_ds,dist_err,ang_err):
        ds_win =[]
        angs = []
        for i in range(len(sort_ds)):
            j = sort_ds[i][1]

            v1 = self.cur_prog_3d[j]-self.cur_prog_3d[j-1]
            v2 = self.buf_pos_3d[-1]-self.buf_pos_3d[0]
            alph = Point3D.ang(v1,v2)
            angs.append(alph)
            if alph<ang_err:
                ds_win.append(sort_ds[i])
        if len(ds_win)==0: return 0
        return ds_win[0][1]
    
    

    #def stop(self):
        #return self.robot.stop()
    
