from pulseapi import  RobotPulse, pose, position, PulseApiException, MT_JOINT, MT_LINEAR,jog,create_box_obstacle,LinearMotionParameters,InterpolationType,tool_info
from pdhttp import Position,Point,Rotation,Pose,MotorStatus,PoseTimestamp,PositionTimestamp,ToolInfo
from PulseUtil import *

class PulseRobotExt(object):
    robot: RobotPulse
    base: Point3D = Point3D(0,0,0)
    tool: Point3D= Point3D(0,0,0)



    def __init__(self,host) -> None:
        self.robot = RobotPulse(host)


    def get_pose(self):
        return self.robot.get_pose()
    
    def get_position(self):
        return self.robot.get_position()
    
    def stop(self):
        return self.robot.stop()
    
    def recover(self):
        return self.robot.recover()
    
    def set_position(self,target_position,
                     velocity = None,
                     acceleration = None,
                     tcp_max_velocity = None, 
                     motion_type: str = MT_JOINT):
        return self.robot.set_position(target_position,velocity,acceleration,tcp_max_velocity, motion_type)
    
    def set_pose(self,target_pose,
                speed= None,
                velocity = None,
                acceleration = None,
                tcp_max_velocity = None,
                motion_type: str = MT_JOINT):
        return self.robot.set_pose(target_pose,speed,velocity ,acceleration, tcp_max_velocity,motion_type)
    
    def run_linear_positions(self,positions: list[Position],
                                motion_parameters: LinearMotionParameters):
        return self.robot.run_linear_positions(positions,motion_parameters)
    
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
    
    #def stop(self):
        #return self.robot.stop()
    
