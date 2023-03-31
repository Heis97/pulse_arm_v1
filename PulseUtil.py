import math
import numpy as np
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

def comp_axes_ps(qs:list,rad:bool = True)->list[Point3D]:
    ps = [Point3D(0,0,0)]
    for i in range(len(qs)):
        pm = calc_forward_kinem_pulse(qs,rad,i+1)
        p  = Point3D(pm[0][3],pm[1][3],pm[2][3])*100
        ps.append(p)
    return ps

def comp_matrs_ps(q:Pose3D,rad:bool = True)->list[Point3D]:
    pms = []
    for i in range(len(q.angles)):
        pm = pulse_matrix_p( calc_forward_kinem_pulse(q,rad,i+1))
        pms.append(pm)
    return pms


def calc_forward_kinem_pulse(q:Pose3D,rad:bool = False,n = 6):
    L1 = 0.2311
    L2 = 0.45
    L3 = 0.37
    L4 = 0.1351
    L5 = 0.1825
    L6 = 0.1325
    #print(q)
    if not rad:
        q.angles = toRad(q.angles)
    
    dh_params = [
        [ q.angles[0], np.pi / 2, 0, L1],
        [ q.angles[1],  0, -L2, 0],
        [ q.angles[2],  0, -L3, 0],
        [ q.angles[3], np.pi / 2, 0, L4],
        [ q.angles[4], -np.pi / 2, 0, L5],
        [ q.angles[5],  0, 0, L6]
    ]

    p = position_from_matrix_pulse(calc_pos(dh_params[:n]))
    p.t = q.t
    return p

def pulse_FK(pose:Pose3D):
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

    return position_from_matrix_pulse(end_p)

def debug_inv_kin(pose,posit):
    q = toRad(pose)
    print("orig:_________________")
    L1 = 0.2311
    L2 = 0.45
    L3 = 0.37
    L4 = 0.1351
    L5 = 0.1825
    L6 = 0.1325

    dh_params = [
        [q[0], np.pi / 2, 0, L1],
        [ q[1],  0, -L2, 0],
        [ q[2],  0, -L3, 0],
        [ q[3], np.pi / 2, 0, L4],
        [ q[4], -np.pi / 2, 0, L5],
        [ q[5],  0, 0, L6]
    ]
    pm = calc_pos(dh_params)
    p = Point3D(pm[0][3],pm[1][3],pm[2][3])

    pos = position_from_matrix_pulse(pm)
    #print("p_or",p.ToString(),"\n",pm)
    print(q)
    print("comp:_________________")
    return calc_inverse_kinem_pulse(pos)



def calc_inverse_kinem_pulse(position:Point3D)->list[Pose3D]:
    vars3 = [[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]]
    solvs = []
    for var in vars3:
        solvs.append(calc_inverse_kinem_pulse_priv(position,var[0],var[1],var[2]))
    return solvs

def p_to_q(position:Point3D,t:int = 1)->Pose3D:
    var = [[-1,-1,-1],[-1,-1,1],[-1,1,-1],[-1,1,1],[1,-1,-1],[1,-1,1],[1,1,-1],[1,1,1]]
    return calc_inverse_kinem_pulse_priv(position,var[t][0],var[t][1],var[t][2])

def q_to_p(pose:Pose3D)->Point3D:
    return calc_forward_kinem_pulse(pose,True)


def calc_inters_2circ(x1,y1,x2,y2,R1,R2,sign):
    x2-=x1
    y2-=y1
    sD=sign*((R1**2+2*R1*R2+R2**2-x2**2-y2**2)*(2*R1*R2-R1**2-R2**2+x2**2+y2**2))**0.5
    x=(R1**2-R2**2+x2**2+y2**2-((y2*(R1**2*y2-R2**2*y2+x2**2*y2+y2**3+x2*sD))/(x2**2+y2**2)))/(2*x2)
    y=(R1**2*y2-R2**2*y2+x2**2*y2+y2**3+x2*sD)/(2*(x2**2+y2**2))
    x+=x1
    y+=y1
    return (x,y)

def arccos(cos):
    if cos>=1: cos = 1
    elif cos <=-1: cos = -1
    return np.arccos(cos)

def calc_inverse_kinem_pulse_priv(position:Point3D,t1=1,t2=1,t3=1) -> Point3D:
    pm = pulse_matrix_p(position)
    p = Point3D(pm[0][3],pm[1][3],pm[2][3])
    L1 = 0.2311
    L2 = 0.45
    L3 = 0.37
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

    """p0_xy = p0p.magnitude_xy()
    alpha = np.arctan(p0p.y/p0p.x)
    gamma = np.arctan(L4/aa_d)
    beta = alpha - t1*gamma
    x2 = aa_d * np.cos(beta)
    y2 = aa_d * np.sin(beta)"""
    #t1 = 1
    #t2 = 1
    #t3 = 1
    x2,y2 = calc_inters_2circ(0,0,p0p.x,p0p.y,aa_d,L4,t1)

    vf = Point3D(x2 - p0p.x, y2 - p0p.y,0)

    vf = vf.normalyse()

    #---------------p1------------------
    ax5y = Point3D.vec_perpend_2_vecs(vz,vf)

    #print(ax5y**vf,ax5y**vz)

    ax5y*= t2


    #vf == ax4y

    #print("vf ",vf.ToString())
    #print("ax5v ",ax5y.ToString())

    p1 =  p0p + ax5y*L5
    #print("p1 ",p1.ToString())
    p2 = p1 + vf*L4
    #print("p2 ",p2.ToString())

    scara = p2 - Point3D(0,0,L1)

    #q1 = np.arctan(scara.y/scara.x) - np.pi

    sq1 = -scara.y/scara.magnitude_xy()
    cq1 = -scara.x/scara.magnitude_xy()

    q1 = np.sign(sq1)* np.arccos(cq1)
    
    Ls = scara.magnitude_xy()
    Lt = scara.magnitude()
    #omega = np.arcsin(scara.z/Lt)

    omega = np.arctan(scara.z/Ls)
    theta = arccos((L2**2+L3**2-Lt**2)/(2*L2*L3))
    omega_ext = arccos((L2**2+Lt**2-L3**2)/(2*L2*Lt))
    omega += t3*omega_ext

    q2 = -omega
    q3 = np.pi-theta

    if t1>0:
        q1+=np.pi
        
        q2*=-1
        q2-=np.pi
        q3*=-1
        pass

    if t3<0:
        q3*=t3        
        pass

    ax4z = -ax5y 
    ax4y = -vf
    ax4x = ax4y*ax4z
    dh_params = [
        [q1, np.pi / 2, 0, L1],
        [ q2,  0, -L2, 0],
        [ q3,  0, -L3, 0],
    ]
    pm3 = calc_pos(dh_params)

    ax3x = Point3D(pm3[0][0],pm3[1][0],pm3[2][0])
    s4 = Point3D.sign_r_v(ax3x,ax4x,ax4y)
    q4 =  s4*np.arccos((ax3x**ax4x)/(ax4x.magnitude()*ax3x.magnitude())) #check sign from ax3z


    ax6y = Point3D(pm[0][1],pm[1][1],pm[2][1])
    ax6z = Point3D(pm[0][2],pm[1][2],pm[2][2])

    s6 = Point3D.sign_r_v(ax5y,ax6y,ax6z)
    q6 = s6*np.arccos((ax5y**ax6y)/(ax5y.magnitude()*ax6y.magnitude()))#check sign from ax6z

    
    s5 = Point3D.sign_r_v(ax4y,ax6z,ax4z)
    q5 = s5*np.arccos((ax4y**ax6z)/(ax4y.magnitude()*ax6z.magnitude()))#check sign from ax4z

    q6+=np.pi

    q = [q1,q2,q3,q4,q5,q6]
    for i in range(len(q)):
        qi = q[i]
        if qi>np.pi: qi-=2*np.pi
        if qi<-np.pi: qi+=2*np.pi
        q[i]= qi
    k = 100

    q_p = Pose3D(q)
    q_p.t = position.t
    return q_p#,[p*k,p0p*k,p1*k,p2*k,Point3D(0,0,L1)*k],t1,t2,t3]


def calc_inverse_kinem_pulse_old(position:Point3D)->list:
    pm = pulse_matrix_p(position)
    p = Point3D(pm[0][3],pm[1][3],pm[2][3])
    L1 = 0.2311
    L2 = 0.45
    L3 = 0.37
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

    vf = -Point3D(x2 - p0p.x, y2 - p0p.y,0)

    vf = vf.normalyse()



    #---------------p1------------------
    ax5y_1,ax5y_2 = Point3D.vec_perpend_2_vecs(vz,vf)

    #vf == ax4y

    print("vf ",vf.ToString())
    print("ax5v ",ax5y_1.ToString())

    p1 =  p0p + ax5y_1*L5
    #print("p1 ",p1.ToString())
    p2 = p1 - vf*L4
    #print("p2 ",p2.ToString())

    scara = p2 - Point3D(0,0,L1)

    q1 = np.arctan(scara.y/scara.x) - np.pi
    
    Ls = scara.magnitude_xy()
    Lt = scara.magnitude()
    omega = np.arctan(scara.z/Ls)

    #Lt**2 = L2**2 + L3**2 - 2*L2*L3*cos(theta)

    theta = np.arccos((L2**2+L3**2-Lt**2)/(2*L2*L3))
    omega_ext = np.arccos((L2**2+Lt**2-L3**2)/(2*L2*Lt))
    omega+=omega_ext

    q2 = -omega
    q3 = np.pi-theta

    
    #print("scara ",scara.ToString())
    print("q1",q1,"q2",q2,"q3",q3)

    ax6y = Point3D(pm[0][1],pm[1][1],pm[2][1])
    ax5y = ax5y_1
    q6 = -np.arccos((ax5y**ax6y)/(ax5y.magnitude()*ax6y.magnitude()))#check sign from ax6z

    ax6z = Point3D(pm[0][2],pm[1][2],pm[2][2])
    ax4y = vf
    q5 = np.arccos((ax4y**ax6z)/(ax4y.magnitude()*ax6z.magnitude()))#check sign from ax4z

    ax4z = -ax5y 

    ax4x = ax4y*ax4z


    dh_params = [
        [q1, np.pi / 2, 0, L1],
        [ q2,  0, -L2, 0],
        [ q3,  0, -L3, 0],
    ]
    pm3 = calc_pos(dh_params)

    ax3x = Point3D(pm3[0][0],pm3[1][0],pm3[2][0])

    q4 = np.arccos((ax3x**ax4x)/(ax4x.magnitude()*ax3x.magnitude()))#check sign from ax3z

    #print("pm3",pm3)


    print("q4",q4,"q5",q5,"q6",q6)


#----------------------------------------------------------
def div_traj(p1:"Point3D",p2:"Point3D",dist:float)->list[Point3D]:
    d = (p2-p1).magnitude()
    div_count = int(d/dist)
    if div_count==0:
        return []
    dp = (p2-p1)*float(1/div_count)
    ps =[]
    for i in range(0,div_count):
        ps.append(p1+dp*i)
    return ps

def comp_traj_to_anim(trajectory:"list[Point3D]",dist:float):
    ps = []

    for i in range(len(trajectory)-1):
        ps_d = div_traj(trajectory[i],trajectory[i+1],dist)
        ps+=ps_d
    return ps


def div_traj_2(p1:"Point3D",p2:"Point3D",dist:float,cd:float)->tuple["list[Point3D]",float]:
    d = (p2-p1).magnitude()
    div_count = int(d/dist)
    if div_count==0:
        return [],cd-d
    ort_v = (p2-p1).normalyse()
    dp = ort_v*dist
    ps =[]
    p1s = p1+ort_v*(dist-cd)
    for i in range(div_count):
        ps.append(p1s+dp*i)
    return ps,d-div_count*dist+cd

def comp_traj_to_anim_2(trajectory:"list[Point3D]",dist:float)->"list[Point3D]":
    ps = []
    cd = 0
    for i in range(len(trajectory)-1):
        ps_d,cd = div_traj_2(trajectory[i],trajectory[i+1],dist,cd)
        #print("cd",cd)
        ps+=ps_d
    return ps

def comp_traj_to_anim_3(trajectory:"list[Point3D]",dist:float)->"list[Point3D]":
    ps = []
    cd = 0
    for i in range(len(trajectory)-1):
        if (trajectory[i]-trajectory[i+1]).magnitude()>2*dist:
            ps_d,cd = div_traj_2(trajectory[i],trajectory[i+1],dist,cd)
            ps+=ps_d
        else:
            ps.append(trajectory[i+1])
        #print("cd",cd)
        
    return ps

#------------------------------------------------    
def comp_blend_lines(p1:"Point3D",p2:"Point3D",p3:"Point3D",r:float,d:float):
    v1 = p1-p2
    v2 = p3-p2
    alph = Point3D.ang(v1,v2)
    d_alph = d/2*np.pi*r
    #print(alph,d_alph)
    if alph<d_alph or np.pi-alph<d_alph:
        return [p2]
    else:
        r_1:float = r/np.sin(alph/2)
        vr = v1.normalyse()+v2.normalyse()
        vr = vr.normalyse()*r_1
        rc = p2+vr
        dr:float = (r_1**2-r**2)**0.5
        vr1 = v1*dr - vr
        vr2 = v2*dr - vr
        alph_r = Point3D.ang(vr1,vr2)
        dvr = vr1-vr2
        #n = int(dvr.magnitude()/d)
        #print(r,alph,d)
        n = int((alph/(2*np.pi))*2*np.pi*r/d)
        vp2s = []
        vp2s.append(p2+vr+vr1.normalyse()*r)
        for i in range(1,n):
            ang = 0.5*np.pi*i/n
            p =  p2+vr+(vr1.normalyse()*np.cos(ang)+vr2.normalyse()*np.sin(ang)).normalyse()*r
            vp2s.append(p)
        vp2s.append(p2+vr+vr2.normalyse()*r)
        #vp2s = [p2+vr]
        return vp2s
    
def blend_lines(ps:"list[Point3D]",r:float,d:float):
    ps_b = [ps[0]]
    for i in range(1,len(ps)-1):
        ps_b+=comp_blend_lines(ps[i-1],ps[i],ps[i+1],r,d)

        if i%100==0:
            print(i,"/",len(ps))
    ps_b.append(ps[-1])
    return ps_b

def filtr_dist(ps:"list[Point3D]",d):
    ps_f = [ps[0]]
    for i in range(1,len(ps)):
        if (ps[i]-ps_f[-1]).magnitude()>d:
            ps_f.append(ps[i])

    return ps_f


def fullsum_list(l:"list[Pose3D]")->Pose3D:
    #print(type([[]]))
    p = [0]*len(l[0].angles)
    for e in l:
        for i in range(len(l[0].angles)):
            #print(p,e)
            p[i]+=e.angles[i]

    for i in range(len(l[0].angles)):
        p[i]/=float(len(l))
    return Pose3D(p)

def filtr_gauss_list(ps:"list[Pose3D]",wind:int)->list[Pose3D]:
    ps_f = []
    for i in range(wind,len(ps)-wind):
        i_b = i-wind
        i_end = i+wind
        if i_b<0: i_b = 0
        if i_end>len(ps): i_end=len(ps)
        p_f = fullsum_list(ps[i_b:i_end])
        p_f.t = ps[i].t
        ps_f.append(p_f)

    return ps_f

def filtr_gauss(ps:"list[Point3D]",wind:int):
    ps_f = ps[:wind]
    ps_f = []
    for i in range(wind,len(ps)-wind):
        p_f = Point3D.fullsum(ps[i-wind:i+wind])
        #ps_f.append(p_f)
        ps_f.append(ps[i])
    return ps_f
#---------------------------------------------------------------------------

def g_code_to_ps_rel_xyz(prog:list[Point3D],base:Point3D,st_p:Point3D,
                                filtr_dist_g_code:float,
                                traj_divide:float,
                                blend:float,vel:float):
    base_m = pulse_matrix_p(base)
    p1_m = pulse_matrix_p(st_p)
    base_m_inv = np.linalg.inv(base_m)
    p_base_m = np.dot(base_m_inv,p1_m)

    p1 = position_from_matrix_pulse(p_base_m)
    base_p_inv = position_from_matrix_pulse(base_m_inv)
    traj = filtr_dist(prog,filtr_dist_g_code)
    traj = blend_lines(traj,blend,traj_divide)

    #d_t = Point3D.dists_between_ps(traj)
    #for d in d_t: print(d)
    ps = comp_traj_to_anim_2(traj,traj_divide)
    print("______________")
    #d_t = Point3D.dists_between_ps(ps)
    #for d in d_t: print(d)
    #ps = traj
    #return ps
    dt = traj_divide/vel
    t = 0
    for i in range(len(ps)):
        ps[i].t = t
        t+=dt

    ps = Point3D.mulList(ps,1e-3)
    ps = Point3D.addList( ps,st_p)
    ps = Point3D.mulPoint(ps,base_p_inv)
    #ps = Point3D.mulList(Point3D.addList( Point3D.mulPoint(ps,base_p),-p1),1e3) 
    return ps

def qs_to_ps(traj:list[Pose3D]):
    ps = []
    for q in traj: ps.append(q_to_p(q))
    return ps

def ps_to_qs(traj:list[Point3D]):
    qs = []
    for p in traj: qs.append(p_to_q(p))
    return qs

def compare_traj_pulse(qs_real:list[Pose3D],prog:list[Point3D],base:Point3D,st_p:Point3D,filtr_dist_g_code:float = 0.3,traj_divide:float = 0.01,blend:float = 1,vel = 20):
    ps_model = g_code_to_ps_rel_xyz(prog,base,st_p,filtr_dist_g_code,traj_divide,blend,vel)
    qs_model = ps_to_qs(ps_model)
    #qs_model = Pose3D.run_aver(qs_model,0.01)
    ps_model = qs_to_ps(qs_model)

    #q_vel = 2*np.pi*40/60
    qs_real = Pose3D.median(qs_real,50)
    qs_real = Pose3D.gauss(qs_real,50)
    ps_real = qs_to_ps(qs_real)

    return qs_real,ps_real,qs_model,ps_model

