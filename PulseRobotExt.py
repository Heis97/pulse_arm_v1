from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_JOINT, MT_LINEAR,jog,create_box_obstacle,LinearMotionParameters,InterpolationType,tool_info
from pdhttp import Position,Point,Rotation,Pose,MotorStatus,PoseTimestamp,PositionTimestamp,ToolInfo
from PulseUtil import *

class PulseRobotExt(object):
    robot: RobotPulse
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


    def update_buf(self):
        self.buf_pos_3d.append(self.cur_posit_3d)
        if len(self.buf_pos_3d)> self.buf_len:
            del self.buf_pos_3d[0]

    def __init__(self,host) -> None:
        if host is not None:       
            self.robot = RobotPulse(host)


    def get_pose(self):
        return self.robot.get_pose()
    
    def get_position(self):
        return self.robot.get_position()
    
    def stop(self):
        return self.robot.stop()
    
    def recover(self):
        return self.robot.recover()
    
    def set_position(self,_target_position,
                     _velocity = None,
                     _acceleration = None,
                     _tcp_max_velocity = None, 
                     _motion_type: str = MT_LINEAR):
        return self.robot.set_position(target_position=_target_position,
                                       velocity=_velocity,
                                       acceleration=_acceleration,
                                       #tcp_max_velocity=_tcp_max_velocity,
                                         motion_type=_motion_type)
    
    def set_pose(self,target_pose,
                speed= None,
                velocity = None,
                acceleration = None,
                tcp_max_velocity = None,
                motion_type: str = MT_JOINT):
        return self.robot.set_pose(target_pose,speed,velocity ,acceleration, tcp_max_velocity,motion_type)
    
    def run_linear_positions(self,positions: list[Position],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = positions_to_p3ds(positions)
        self.cur_i_prog = 0
        return self.robot.run_linear_positions(positions,motion_parameters)
    
    def run_linear_positions(self,positions: list[Position],ps: list[Point3D],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = ps
        self.cur_i_prog = 0
        return self.robot.run_linear_positions(positions,motion_parameters)
    
    """def run_poses(self,positions: list[Pose],ps: list[Point3D],
                                motion_parameters: LinearMotionParameters):
        self.cur_prog_3d = ps
        self.cur_i_prog = 0
        return self.robot.run_poses(positions,motion_parameters)"""
    
    def change_base(self,base_position):
        self.base = pos_dict_to_point3d(base_position.to_dict())
        return self.robot.change_base(base_position)
    
    def get_base(self):

        return self.robot.get_base()
    
    def change_tool_info(self,new_tool_info:ToolInfo):
        self.tool = pos_dict_to_point3d(new_tool_info.tcp.to_dict())
        return self.robot.change_tool_info(new_tool_info)
    
    def get_tool_info(self):
        return self.robot.get_tool_info()
    
    def freeze(self):
        return self.robot.freeze()
    
    def jogging(self,jog_value):
        return self.robot.jogging(jog_value)
    
    def await_stop(self):
        return self.robot.await_stop()
    
    def relax(self):
        return self.robot.relax()
    
    def status_motors(self):
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
    
