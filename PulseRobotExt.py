from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_JOINT, MT_LINEAR,jog,create_box_obstacle,LinearMotionParameters,InterpolationType,tool_info
from pdhttp import Position,Point,Rotation,Pose,MotorStatus,PoseTimestamp,PositionTimestamp,ToolInfo
from PulseUtil import *
from api.robot_api import *


def posit_to_list(posit:Position):
    p3d = position_to_p3d(posit)
    return p3d_to_list(p3d)

def p3d_to_list(p3d:Point3D):
    return [p3d.x,p3d.y,p3d.z,p3d.pitch,p3d.roll,p3d.yaw]


host_old = "http://10.10.10.20:8081"
host_v3 = "192.168.0.50"


class PulseRobotExt(object):
    robot: RobotPulse
    robot_v3: RobotAPI
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
    controller_v3:bool = None


    def update_buf(self):
        self.buf_pos_3d.append(self.cur_posit_3d)
        if len(self.buf_pos_3d)> self.buf_len:
            del self.buf_pos_3d[0]

    def __init__(self,controller_v3:bool = False) -> None:
        self.controller_v3 = controller_v3
        if self.controller_v3:   
            print("connect v3")    
            self.robot_v3 = RobotAPI(host_v3)
            self.robot_v3.init_robot()
        else:
            self.robot = RobotPulse(host_old)
                


    def get_pose(self):
        if self.controller_v3:
            return Pose(self.robot_v3.get_act_pos_q())
        else:
            return self.robot.get_pose()
    
    def get_position(self):
        if self.controller_v3:
            list_pos = self.robot_v3.get_act_pos_cartesian()
            return position(list_pos[0:3],list_pos[3:6])
        else:
            return self.robot.get_position()
    
    def stop(self):
        if self.controller_v3:
            return self.robot_v3.hold()
            #return self.robot_v3.stby()
        else:
            return self.robot.stop()
    
    def recover(self):
        if self.controller_v3:
            pass
            #return self.robot_v3.stby()
        else:
            return self.robot.recover()
    
    def set_position(self,_t_p:Position,
                     _velocity = None,
                     _acceleration = None,
                     _tcp_max_velocity = None, 
                     _motion_type: str = MT_LINEAR):
        if self.controller_v3:
            #pos = [_t_p.point.x(),_t_p.point.y(),_t_p.point.z(),_t_p.rotation.x(),_t_p.rotation.y(),_t_p.rotation.z()]
            self.robot_v3.move_l(posit_to_list(_t_p),_velocity,_acceleration)
            self.robot_v3.run_wps()
            self.robot_v3.await_motion()
            return 
        else:
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
        if self.controller_v3:
            self.robot_v3.move_j(target_pose.angles,speed,acceleration)
            self.robot_v3.run_wps()
            #self.robot_v3.colab_await_buffer(0)
            return 
        
        else:
            return self.robot.set_pose(target_pose,speed,velocity ,acceleration, tcp_max_velocity,motion_type)
        #return self.robot.set_pose(target_pose,speed,velocity ,acceleration, tcp_max_velocity,motion_type)
    


    def run_linear_positions(self,positions: list[Position],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = positions_to_p3ds(positions)
        self.cur_i_prog = 0

        if self.controller_v3:
            for pos in positions: self.robot_v3.move_l(posit_to_list(pos),motion_parameters.velocity,motion_parameters.acceleration) 
            return
            #return self.robot_v3.move_j(target_pose.angles,speed,acceleration)
        else:
            return self.robot.run_linear_positions(positions,motion_parameters)
    
    """def run_linear_positions(self,positions: list[Position],ps: list[Point3D],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = ps
        self.cur_i_prog = 0
        if self.controller_v3:
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
        if self.controller_v3:
            return
        else:
            return self.robot.change_base(base_position)
    
    def get_base(self):
        if self.controller_v3:
            return
        else:
            return self.robot.get_base()
    
    def change_tool_info(self,new_tool_info:ToolInfo):
        self.tool = pos_dict_to_point3d(new_tool_info.tcp.to_dict())
        if self.controller_v3:
            return self.robot_v3.set_tool(p3d_to_list(self.tool))
        else:
            return self.robot.change_tool_info(new_tool_info)
        return self.robot.change_tool_info(new_tool_info)
    
    def get_tool_info(self):
        if self.controller_v3:
            return
        else:
            return self.robot.get_tool_info()
        #return self.robot.get_tool_info()
    
    def freeze(self):
        if self.controller_v3:
            return
        else:
            return self.robot.freeze()
    
    def jogging(self,x,y,z,rx,ry,rz):
        if self.controller_v3:
            speed = 0.1
            acs = 0.2
            return self.robot_v3.set_jog_param([0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0],[speed]*6,[acs]*6,[acs]*6)
        else:
            return self.robot.jogging(jog(x,y,z,rx,ry,rz))
    
    def await_stop(self):
        if self.controller_v3:
            return
        else:
            return self.robot.await_stop()
    
    def relax(self):
        if self.controller_v3:
            return
        else:
            return self.robot.relax()
    
    def status_motors(self):
        if self.controller_v3:
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
    
