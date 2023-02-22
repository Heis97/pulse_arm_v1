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

    return position_from_matrix_pulse(p_t1)

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
        return Point3D(pos_dict["point"]["x"],pos_dict["point"]["y"],pos_dict["point"]["z"],_roll = pos_dict["rotation"]["roll"],_pitch = pos_dict["rotation"]["pitch"],_yaw = pos_dict["rotation"]["yaw"])

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


def calc_forward_kinem_pulse(q:list):
    #print(q)
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

def pulse_FK(pose:list):
    return calc_forward_kinem_pulse(pose)

def toRad(q:list):
    qs = []
    for qi in q:
        qc = qi*np.pi/180
        qs.append(qc)
    return qs

def to_degree(q):
    return q*180/np.pi


def p3d_cur_pulse(flange:Point3D,tool:Point3D,base:Point3D):
    cur_flange_m = pulse_matrix_p(flange)

    cur_base_m = pulse_matrix_p(base)
    cur_tool_m = pulse_matrix_p(tool)

    end_p = np.dot(cur_flange_m,cur_tool_m) 
    end_p = np.dot(cur_base_m,end_p) 
    #print(cur_tool_m)

    return position_from_matrix_pulse(end_p)

def debug_inv_kin(pose,posit):
    q = toRad(pose)
    print("orig:_________________")
    dh_params = [
        [q[0], np.pi / 2, 0, 0.2311],
        [ q[1],  0, -0.450, 0],
        [ q[2],  0, -0.370, 0],
        [ q[3], np.pi / 2, 0, 0.1351],
        [ q[4], -np.pi / 2, 0, 0.1825],
        [ q[5],  0, 0, 0.1325]
    ]
    pm = calc_pos(dh_params)
    p = Point3D(pm[0][3],pm[1][3],pm[2][3])
    #print("p_or",p.ToString())

    dh_params = [
        [q[0], np.pi / 2, 0, 0.2311],
        [ q[1],  0, -0.450, 0],
        [ q[2],  0, -0.370, 0],
        [ q[3], np.pi / 2, 0, 0.1351],
        [ q[4], -np.pi / 2, 0, 0.1825],
        #[ q[5],  0, 0, 0.1325]
    ]
    pm = calc_pos(dh_params)
    p0 = Point3D(pm[0][3],pm[1][3],pm[2][3])
    print("p0_or",p0.ToString(),"\n",pm)

    

    dh_params = [
        [q[0], np.pi / 2, 0, 0.2311],
        [ q[1],  0, -0.450, 0],
        [ q[2],  0, -0.370, 0],
        [ q[3], np.pi / 2, 0, 0.1351],
        #[ q[4], -np.pi / 2, 0, 0.1825],
        #[ q[5],  0, 0, 0.1325]
    ]
    pm = calc_pos(dh_params)
    p1 = Point3D(pm[0][3],pm[1][3],pm[2][3])
    print("p1_or",p1.ToString(),"\n",pm)

    dh_params = [
        [q[0], np.pi / 2, 0, 0.2311],
        [ q[1],  0, -0.450, 0],
        [ q[2],  0, -0.370, 0],
        #[ q[3], np.pi / 2, 0, 0.1351],
        #[ q[4], -np.pi / 2, 0, 0.1825],
        #[ q[5],  0, 0, 0.1325]
    ]
    pm = calc_pos(dh_params)
    p1 = Point3D(pm[0][3],pm[1][3],pm[2][3])
    print("p2_or",p1.ToString(),"\n",pm)

    #print("d01_or ",(p1-p0).magnitude())

    print("comp:_________________")
    calc_inverse_kinem_pulse(posit)


def calc_inverse_kinem_pulse(position:Point3D)->list:
    pm = pulse_matrix_p(position)
    p = Point3D(pm[0][3],pm[1][3],pm[2][3])
    L4 = 0.1351
    L5 = 0.1825
    L6 = 0.1325

    dz = np.array([
        [1.,0.,0.,0.],
    [0.,1.,0.,0.],
    [0.,0.,1.,-L6],
    [0.,0.,0.,1.]])
    
    #---------------p0 ---------------
    p0 = np.dot(pm,dz)
    #print("p0\n",p0)
    p0p = Point3D(p0[0][3],p0[1][3],p0[2][3])
    #print("p0 ",p0p.ToString())
    #---------------vz---------------
    vz = Point3D(pm[0][2],pm[1][2],pm[2][2])
    #---------------v f---------------
    xy_d = (p0p.x**2+p0p.y**2)**0.5
    aa_d = (xy_d**2-L4**2)**0.5

    alpha = np.arctan(p0p.y/p0p.x)
    gamma = np.arctan(L4/aa_d)
    beta = alpha - gamma
    x2 = aa_d * np.cos(beta)
    y2 = aa_d * np.sin(beta)

    vf = Point3D(x2 - p0p.x, y2 - p0p.y,0)

    vf = vf.normalyse()

    #---------------p1------------------

    dza5 = np.array([
        [1.,0.,0.,0.],
    [0.,1.,0.,0.],
    [0.,0.,1.,-L5],
    [0.,0.,0.,1.]])

    va5_1,va5_2 = Point3D.vec_perpend_2_vecs(vz,vf)

    #print("vf ",vf.ToString())

    va5_1v = va5_1*L5
    va5_2v = va5_2*L5

    va5_1va = np.array([
        [1.,0.,0.,va5_1v.x],
    [0.,1.,0.,va5_1v.y],
    [0.,0.,1.,va5_1v.z],
    [0.,0.,0.,1.]])

    va5_2va = np.array([
        [1.,0.,0.,va5_2v.x],
    [0.,1.,0.,va5_2v.y],
    [0.,0.,1.,va5_2v.z],
    [0.,0.,0.,1.]])

    p1 =  p0+va5_1va
    p1a = p0+va5_2va
    #print("p1\n",p1)
    p1p  = Point3D(p1[0][3],p1[1][3],p1[2][3])
    print("p1 ",p1p.ToString())

    #print("d01 ",(p1p-p0p).magnitude())
    #print("p1a ",Point3D(p1a[0][3],p1a[1][3],p1a[2][3]).ToString())

    #print("p1\n",p1)
    #print("p1a\n",p1a)



    #print(va5_1.ToString(),va5_2.ToString())



    #print(pm)





def pulse_RK(position:Point3D):
    pass























