import math
import numpy as np
import cv2 as cv
from polygon import Point3D

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


def rotatedX(alpha):
    c_A = np.cos(alpha)
    s_A = np.sin(alpha)
    return np.array([
        [1.,0.,0.,0.],
    [0.,c_A,-s_A,0.],
    [0.,s_A,c_A,0.],
    [0.,0.,0.,1.]])

def rotatedY(alpha):
    c_A = np.cos(alpha)
    s_A = np.sin(alpha)
    return np.array([
        [c_A,0.,s_A,0.],
        [0.,1.,0.,0.],
        [-s_A,0.,c_A,0.],
        [0.,0.,0.,1.]])

def rotatedZ(alpha):
    c_A = np.cos(alpha)
    s_A = np.sin(alpha)
    return np.array([
        [c_A,-s_A,0.,0.],
        [s_A,c_A,0.,0.],
        [0.,0.,1.,0.],
        [0.,0.,0.,1.]])

def pulse_rot_matrix(Rx,Ry,Rz):
    mx = rotatedX(Rx)
    my = rotatedY(Ry)
    mz = rotatedZ(Rz)
    mxy = np.dot( mx, my)
    return np.dot(mxy, mz)

def pulse_matrix(x,y,z,Rx,Ry,Rz):
    rot = pulse_rot_matrix(Rx,Ry,Rz)
    rot[0][3] = x
    rot[1][3] = y
    rot[2][3] = z

    #rot = np.linalg.inv(rot)
    return rot

def position_from_matrix(m):
    x = m[0][3]
    y = m[1][3]
    z = m[2][3]

    b =np.arcsin(-m[0][2])

    if np.cos(b) != 0:   
        a = np.arcsin(m[1][2] / np.cos(b))
        c = np.arcsin(m[0][1] / np.cos(b))

    return x,y,z,a,b,c

def base_calibration(points):
    ps = poses_dict_to_point3d(points)
    vx = compute_vector(ps[0],ps[1])
    vy_1 = compute_vector(ps[0],ps[2])
    vz = (vx*vy_1).normalyse()
    vy = (vz*vx).normalyse()
    m = matr_from_vecs(vx,vy,vz,ps[0])
    return position_from_matrix(m)

def orient_tool_calibration(points):
    ps = poses_dict_to_point3d(points)
    vx = compute_vector(ps[0],ps[1])
    vy_1 = compute_vector(ps[0],ps[2])
    vz = (vx*vy_1).normalyse()
    vy = (vz*vx).normalyse()
    m = matr_from_vecs(vx,vy,vz,ps[0])
    return position_from_matrix(m)

def matr_from_vecs(rx:Point3D,ry:Point3D,rz:Point3D,pos:Point3D):
    return np.array([[rx.x,rx.y,rx.z,pos.x],[ry.x,ry.y,ry.z,pos.y],[rz.x,rz.y,rz.z,pos.z],[0.,0.,0.,1.]])
    
def compute_vector(p1:Point3D,p2:Point3D):
    pd = p2-p1
    pd = pd.normalyse()
    return pd


def pos_dict_to_point3d(pos_dict:dict):
        return Point3D(pos_dict["point"]["x"],pos_dict["point"]["y"],pos_dict["point"]["z"],_r = pos_dict["rotation"]["roll"],_g = pos_dict["rotation"]["pitch"],_b = pos_dict["rotation"]["yaw"])

def poses_dict_to_point3d(pos_dict:list[dict]):
    ps = []
    for p in pos_dict:
        ps.append(pos_dict_to_point3d(p))
    return ps
    

def calibrate_tcp_4p(points:list):

        ps = poses_dict_to_point3d(points)
        pc1 = find_center_sphere_4p([ps[0],ps[1],ps[2],ps[3]])
        #pc2 = find_center_sphere_4p([ps[0],ps[1],ps[2],ps[4]])

        pc = pc1[0]

        tcp_aver = np.array([[0.],[0.],[0.],[0.]])

        for p in ps:
            m = pulse_matrix(p.x,p.y,p.z,p.r,p.g,p.b)
            m_inv = np.linalg.inv(m)
            tcp = np.dot( m_inv ,np.array([[pc.x],[pc.y],[pc.z],[1.]]))
            tcp_aver+=tcp
        return tcp_aver/len(ps)

















