import math
import numpy as np
import cv2 as cv
from polygon import *

def find_center_sphere_4p(ps:list[Point3D]):
    x1 = ps[0].x; y1 = ps[0].y; z1 = ps[0].z
    x2 = ps[1].x; y2 = ps[1].y; z2 = ps[1].z
    x3 = ps[2].x; y3 = ps[2].y; z3 = ps[2].z
    x4 = ps[3].x; y4 = ps[3].y; z4 = ps[3].z
    U = (z1 - z2) * (x3 * y4 - x4 * y3) - (z2 - z3) * (x4 * y1 - x1 * y4)
    V = (z3 - z4) * (x1 * y2 - x2 * y1) - (z4 - z1) * (x2 * y3 - x3 * y2)
    W = (z1 - z3) * (x4 * y2 - x2 * y4) - (z2 - z4) * (x1 * y3 - x3 * y1)

    Ax = (x1 * x1 + y1 * y1 + z1 * z1) * (y2 * (z3 - z4) + y3 * (z4 - z2) + y4 * (z2 - z3))
    Bx = (x2 * x2 + y2 * y2 + z2 * z2) * (y3 * (z4 - z1) + y4 * (z1 - z3) + y1 * (z3 - z4))
    Cx = (x3 * x3 + y3 * y3 + z3 * z3) * (y4 * (z1 - z2) + y1 * (z2 - z4) + y2 * (z4 - z1))
    Dx = (x4 * x4 + y4 * y4 + z4 * z4) * (y1 * (z2 - z3) + y2 * (z3 - z1) + y3 * (z1 - z2))

    Ay = (x1 * x1 + y1 * y1 + z1 * z1) * (x2 * (z3 - z4) + x3 * (z4 - z2) + x4 * (z2 - z3))
    By = (x2 * x2 + y2 * y2 + z2 * z2) * (x3 * (z4 - z1) + x4 * (z1 - z3) + x1 * (z3 - z4))
    Cy = (x3 * x3 + y3 * y3 + z3 * z3) * (x4 * (z1 - z2) + x1 * (z2 - z4) + x2 * (z4 - z1))
    Dy = (x4 * x4 + y4 * y4 + z4 * z4) * (x1 * (z2 - z3) + x2 * (z3 - z1) + x3 * (z1 - z2))

    Az = (x1 * x1 + y1 * y1 + z1 * z1) * (x2 * (y3 - y4) + x3 * (y4 - y2) + x4 * (y2 - y3))
    Bz = (x2 * x2 + y2 * y2 + z2 * z2) * (x3 * (y4 - y1) + x4 * (y1 - y3) + x1 * (y3 - y4))
    Cz = (x3 * x3 + y3 * y3 + z3 * z3) * (x4 * (y1 - y2) + x1 * (y2 - y4) + x2 * (y4 - y1))
    Dz = (x4 * x4 + y4 * y4 + z4 * z4) * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))

    x0 = 0.5 * (Ax - Bx + Cx - Dx) / (U + V + W)
    y0 = -0.5 * (Ay - By + Cy - Dy) / (U + V + W)
    z0 = 0.5 * (Az - Bz + Cz - Dz) / (U + V + W)
    R = math.sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0))
    return [Point3D(x0, y0, z0),Point3D(R,R,R)]


def base_calibration(points):
    ps = poses_dict_to_point3d(points)
    vx = compute_vector(ps[0],ps[1])
    vz = Flat3D.compFlat(ps[0],ps[1],ps[2]).abc
    vy = (vz*vx).normalyse()
    m = matr_from_vecs(vx,vy,vz,ps[0])
    m_inv =  np.linalg.inv(m)
    print(m)
    print(m_inv)
    return position_from_matrix_pulse(m_inv)

def orient_tool_calibration(points):
    ps = poses_dict_to_point3d(points)
    vz = compute_vector(ps[0],ps[1])
    vx = compute_vector(ps[1],ps[2])
    vy = (vz*vx).normalyse()
    m = matr_from_vecs(vx,vy,vz,Point3D(0,0,0))

    p_fl = pulse_matrix_p(ps[3])
    p_fl_inv = np.linalg.inv(p_fl)
    p_t1 = np.dot(p_fl_inv,m)

    return position_from_matrix_pulse(np.linalg.inv(p_t1))

def orient_tool_calibration_1p(p_flange,p_tool):
    p_fl = pulse_matrix_p(pos_dict_to_point3d(p_flange))
    p_t = pulse_matrix_p(pos_dict_to_point3d(p_tool))

    p_fl_inv = np.linalg.inv(p_fl)

    p_t1 = np.dot(p_fl_inv,p_t)

    return position_from_matrix_pulse(p_t1)

def matr_from_vecs(rx:Point3D,ry:Point3D,rz:Point3D,pos:Point3D):
    return np.array([[rx.x,rx.y,rx.z,pos.x],[ry.x,ry.y,ry.z,pos.y],[rz.x,rz.y,rz.z,pos.z],[0.,0.,0.,1.]])
    
def compute_vector(p1:Point3D,p2:Point3D):
    pd = p2-p1
    pd = pd.normalyse()
    return pd


def pos_dict_to_point3d(pos_dict:dict):
        return Point3D(pos_dict["point"]["x"],pos_dict["point"]["y"],pos_dict["point"]["z"],_pitch = pos_dict["rotation"]["roll"],_roll = pos_dict["rotation"]["pitch"],_yaw = pos_dict["rotation"]["yaw"])

def poses_dict_to_point3d(pos_dict:list[dict])->list[Point3D]:
    ps = []
    for p in pos_dict:
        ps.append(pos_dict_to_point3d(p))
    return ps
    
#def 

def calibrate_tcp_4p(points:list[Point3D]):

        ps = poses_dict_to_point3d(points)
        pc1 = find_center_sphere_4p([ps[0],ps[1],ps[2],ps[3]])
        #pc2 = find_center_sphere_4p([ps[0],ps[1],ps[2],ps[4]])

        pc = pc1[0]

        tcp_aver = np.array([[0.],[0.],[0.],[0.]])
        #print(type(tcp_aver))
        for p in ps:
            m = pulse_matrix(p.x,p.y,p.z,p.pitch,p.roll,p.yaw)
            m_inv = np.linalg.inv(m)
            tcp = np.dot( m_inv ,np.array([[pc.x],[pc.y],[pc.z],[1.]]))
            tcp_aver+=tcp
        return tcp_aver/len(ps)


def create_dhmatr(params:list):
    theta = params[0]
    alpha = params[1]
    a = params[2]
    d = params[3]
    cosQ = np.cos(theta)
    sinQ = np.sin(theta)
    cosA = np.cos(alpha)
    sinA = np.sin(alpha)
    return np.array(
			[[cosQ,-sinQ*cosA,sinQ*sinA,a*cosQ],
			[sinQ, cosQ*cosA,-cosQ*sinA,a*sinQ],
			[0   , sinA     , cosA    ,  d],
			[0   , 0        , 0       ,  1]])


def calc_pos(dh_params:list[list]):
    matrs = []
    for dh_param in dh_params:
        matrs.append(create_dhmatr(dh_param))
    matr_res = np.eye(4)
    for matr in matrs:
        matr_res = np.dot(matr_res,matr)
    return matr_res


def calc_forward_kinem(q:list):
    q = toRad(q)
    dh_params = [
        [q[0], np.pi / 2, 0, 0.2311],
        [ q[1],  0, -0.450, 0],
        [ q[2],  0, -0.370, 0],
        [ q[3], np.pi / 2, 0, 0.1351],
        [ q[4], -np.pi / 2, 0, 0.1825],
        [ q[5],  0, 0, 0.1325]
    ]
    return calc_pos(dh_params)

def toRad(q:list):
    qs = []
    for qi in q:
        qc = qi*np.pi/180
        qs.append(qc)
    return qs


















