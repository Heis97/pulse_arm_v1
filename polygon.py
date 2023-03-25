import numpy as np
import math 
import enum
from random import triangular
import struct
import io

class Pose3D(object):
    angles:list[float] = []
    t:float = 0
    def __init__(self,angles:list):
        self.angles = angles.copy()


class Point3D(object):
    x:float = 0
    y:float = 0
    z:float = 0
    extrude:bool
    r:float = 0
    g:float = 0
    b:float = 0

    roll:float = 0
    pitch:float = 0    
    yaw:float = 0

    t:float = 0
    
    def __init__(self,_x:float= 0,_y:float= 0,_z:float = 0,_extrude:bool = True,_r:float = 0.0,_g:float = 1.,_b:float =0.,_pitch:float = 0.0,_roll:float = 0.,_yaw:float =0.,t=0):
        self.x = _x
        self.y = _y
        self.z = _z
        self.r = _r
        self.g = _g
        self.b = _b
        self.pitch = _pitch
        self.roll = _roll
        self.yaw = _yaw
        self.extrude = _extrude
        self.t = t

    def normalyse(self):
        norm = math.sqrt(self.x**2+self.y**2+self.z**2)
        if norm!=0:
            self.x /=norm
            self.y /=norm
            self.z /=norm
        return self

    def ToString(self)->str:
        return str(self.x)+" "+str(self.y)+" "+str(self.z)+";"

    def ToStringPulse(self,pres = 3,delim = "\n")->str:
        return str(round(self.x,pres))+delim+str(round(self.y,pres))+delim+str(round(self.z,pres))+delim+str(round(self.roll,pres))+delim+str(round(self.pitch,pres))+delim+str(round(self.yaw,pres))+";"

    def ToStringArr(arr:"list[Point3D]")->str:
        ret = ""
        for i in range(len(arr)):
            ret+=arr[i].ToString()+"\n"
        return ret

    def magnitude(self):
        return (self.x**2 + self.y**2 + self.z**2)**0.5

    def magnitude_xy(self):
        return (self.x**2 + self.y**2)**0.5

    def __add__(self, other):
        if(type(other)==Point3D):
            self.x += other.x
            self.y += other.y
            self.z += other.z
            return self.Clone()
        else:
            return self

    def __sub__(self, other):

        if(type(other)==Point3D):
            self.x -= other.x
            self.y -= other.y
            self.z -= other.z
            return self.Clone()
        else:
            return self

    def __neg__(self):
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self

    def Clone(self):
        return Point3D(self.x,self.y,self.z,self.extrude,self.r,self.g,self.b,self.pitch,self.roll,self.yaw,self.t)

    def __mul__(self, other):
        if(type(other)==Point3D):
            return Point3D(self.y*other.z-self.z*other.y, self.z*other.x-self.x*other.z,self.x*other.y-self.y*other.x,self.extrude)
        elif(type(other)==np.ndarray):
            x,y,z,Rx,Ry,Rz = position_from_matrix(np.dot(pulse_matrix_p(self),other))
            return Point3D(x,y,z,_pitch= Rx,_roll=Ry,_yaw=Rz)
        elif(type(other)==float):
            self.x*=other
            self.y*=other
            self.z*=other
            return self.Clone()
        elif(type(other)==int):
            return Point3D(other*self.x,other*self.y,other*self.z)
        else:
            return Point3D(self.x*other,self.y*other,self.z*other,self.extrude)

    def __pow__(self, other):
        return self.x*other.x +self.y*other.y+self.z*other.z

    def dot(self,other:"Point3D"):
        self_m = pulse_matrix_p(self)
        other_m = pulse_matrix_p(other)
        m = np.dot(self_m,other_m) 
        p = position_from_matrix_pulse(m)
        p.t = self.t
        return p


    def dists_between_ps(ps:"list[Point3D]")->"list[float]":
        dists= []
        for i in range(len(ps)-1):
            dists.append((ps[i]-ps[i+1]).magnitude())
        return dists
        
    def matrMul(self,matr:"list[list[float]]"):

        x = matr[0][0]*self.x + matr[0][1]*self.y + matr[0][2]*self.z+matr[0][3]
        y = matr[1][0]*self.x + matr[1][1]*self.y + matr[1][2]*self.z+matr[1][3]
        z = matr[2][0]*self.x + matr[2][1]*self.y + matr[2][2]*self.z+matr[2][3]
        return Point3D(x,y,z,self.extrude)

    def vec_perpend_2_vecs(v1:"Point3D",v2:"Point3D"):
        d = (v1.x**2)*(v2.y**2)+(v1.x**2)*(v2.z**2)-2*v1.x*v1.y*v2.x*v2.y-2*v1.x*v2.x*v1.z*v2.z+(v1.y**2)*(v2.x**2)+(v1.y**2)*(v2.z**2)-2*v1.y*v1.z*v2.y*v2.z+(v2.x**2)*(v1.z**2)+(v1.z**2)*(v2.y**2)
        vx =  (v1.y*v2.z-v1.z*v2.y)*((1/d)**0.5)
        vy = -(v1.x*v2.z-v2.x*v1.z)*((1/d)**0.5)
        vz =  (v1.x*v2.y-v1.y*v2.x)*((1/d)**0.5)

        return Point3D(vx,vy,vz)
    def ang(v1:"Point3D",v2:"Point3D"):
        cos = v1**v2/(v1.magnitude()*v2.magnitude())
        if cos>=1: cos = 1
        elif cos <=-1: cos = -1
        return np.arccos(cos)
        
    
    def one_dir(v1:"Point3D",v2:"Point3D"):
        if Point3D.ang(v1,v2)<np.pi/2: return True
        else: return False

    def sign_r_v(v1:"Point3D",v2:"Point3D",v3:"Point3D"):
        sign =1
        t_v3 = v1*v2
        if not Point3D.one_dir(v3,t_v3):
            sign = -1

        return sign
    
    def mulList(l:"list[Point3D]",k:float):
        ps = []
        for e in l: ps.append(e*k)
        return ps
    
    def addList(l:"list[Point3D]",p_off:"Point3D"):
        ps = []
        for e in l: ps.append(p_off+e)
        return ps
    
    def mulPoint(l:"list[Point3D]",p_off:"Point3D"):
        ps = []
        for e in l: ps.append(p_off.dot(e))
        return ps
    
    
    
    def mulPoint_2(l:"list[Point3D]",p_off:"Point3D"):
        ps = []
        for e in l: ps.append(e.dot(p_off))
        return ps
    
    


class Flat3D(object):
    abc:Point3D
    d:float
    def __init__(self,_abc:Point3D,_d:float) -> None:
        self.abc = _abc
        self.d = _d

    def compFlat(p1:Point3D,p2:Point3D,p3:Point3D):
        v1 = p2 - p1
        v2 = p3 - p1
        abc = (v1*v2).normalyse()
        d = -abc**p1
        return Flat3D(abc,d)

    def compFlatPV(abc:Point3D,p:Point3D):
        d = -abc**p
        return Flat3D(abc,d)   
        
class PrimitiveType(enum.Enum):
    points = 1
    lines = 2
    triangles = 3 
    lines_def = 4 

class Polygon3D(object):
    vert_arr:"list[Point3D] "
    n:Point3D
    def __init__(self,_vert_arr:"list[Point3D]"=None):
        if(_vert_arr!=None):
            self.n = None
            self.vert_arr = _vert_arr
            if (len(_vert_arr) > 2):
                self.n = self.compNorm(_vert_arr[0],_vert_arr[1],_vert_arr[2])

    def compNorm(self, p3:Point3D,p2:Point3D,p1:Point3D):
        v = p3-p1
        u = p2-p1
        v = v.normalyse()
        u = u.normalyse()
        '''Norm = Point3D(
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x)'''
        Norm = u*v
        Norm.normalyse()
        return Point3D(-Norm.x,-Norm.y,-Norm.z)
    
    def matrMul(self,matr:"list[list[float]]"):
        for i in range(len(self.vert_arr)):
            self.vert_arr[i] = self.vert_arr[i].matrMul(matr)
        return self
    
    def affilationPoint(self, p: Point3D):
        if (len(self.vert_arr)<3):
            return False
        a = self.vert_arr [0].Clone()
        b = self.vert_arr [1].Clone()
        c = self.vert_arr [2].Clone()
        p = p.Clone()
        p = p - a
        b = b - a
        c = c - a

        m =  (p.x*b.y - b.x*p.y)/(c.x*b.y - b.x*c.y)
        if(m >=0 and m <=1):
            l = (p.x - m*c.x)/b.x
            if (l >=0 and m+l <=1):
                return True
        return False

    def project_point(self,p: Point3D):
        p1 = self.vert_arr [0]
        d = -(self.n.x*p1.x + self.n.y*p1.y + self.n.z*p1.z)
        z = (-d - self.n.x*p.x- self.n.y*p.y)/self.n.z
        return Point3D(p.x, p.y, z)

    def __mul__(self, other):
        if(type(other)==float):
            for i in range(len(self.vert_arr)):
                self.vert_arr[i]*=other
            return self

    def Clone(self):
        vert = []
        norm = self.n.Clone()
        for i in range(len(self.vert_arr)):
            vert.append(self.vert_arr[i].Clone())
        copy = Polygon3D()
        copy.vert_arr = vert
        copy.n = norm
        return copy

    def matrMul(self,matr:"list[list[float]]"):
        copy = self.Clone()
        for i in range(len(self.vert_arr)):
             copy.vert_arr[i] = self.vert_arr[i].matrMul(matr)
        return  copy


    def crossFlat(self,flat:Flat3D):
        ps = []
        if len(self.vert_arr)>2:
            for i in range(len(self.vert_arr)):
                p_c = self.cross_affil(self.vert_arr[i-2],self.vert_arr[i-1],flat)
                if p_c!=None:
                    ps.append(p_c)
        return ps
        
    def cross_affil(self,p1:Point3D, p2:Point3D,flat:Flat3D ):
        v = p2 - p1
        p = p1.Clone()
        #print(flat.d)
        if v**flat.abc==0:
            return None
        t = (-flat.d-p**flat.abc)/(v**flat.abc)

        p_c = p + v * t
        if self.affil_segment(p1,p2,p_c)==True:
            return p_c
        else:
            return None   

    def affil_segment(self,p_st:Point3D,p_end:Point3D,p_ch:Point3D)->bool:
        dist_0 = (p_end-p_st).magnitude()
        dist_1 = (p_ch-p_st).magnitude()
        dist_2 = (p_ch-p_end).magnitude()
        if dist_1< dist_0 and dist_2<dist_0:
            return True
        return False

    def cross(self,p1:Point3D, p2:Point3D,flat:Flat3D ):
        v = p2 - p1
        p = p1.Clone()
        t = (-flat.d-p**flat.abc)/(v**flat.abc)
        return p + v * t
        
class Mesh3D(object):
    polygons:"list[Polygon3D]" = []
    primTp:PrimitiveType
    def __init__(self,_points:"list[Point3D]"=None, prim_type: PrimitiveType=None):
        self.polygons = []
        self.primTp = prim_type
        if _points!=None:
            if (prim_type == PrimitiveType.points ):
                
                for i in range (len(_points)):
                    vert_array = []
                    vert_array.append(_points[i])
                    self.polygons.append(Polygon3D(vert_array))

            elif (prim_type == PrimitiveType.lines):
                for i in range (len(_points)-1):
                    vert_array = []
                    vert_array.append(_points[i])
                    vert_array.append(_points[i+1])
                    self.polygons.append(Polygon3D(vert_array)) 

            elif (prim_type == PrimitiveType.lines_def):
                for i in range (0,len(_points)-1,2):
                    vert_array = []
                    vert_array.append(_points[i])
                    vert_array.append(_points[i+1])
                    self.polygons.append(Polygon3D(vert_array)) 

            elif (prim_type == PrimitiveType.triangles):
                for i in range (int(len(_points)/3)):
                    vert_array = []
                    vert_array.append(_points[3*i])
                    vert_array.append(_points[3*i+1])
                    vert_array.append(_points[3*i+2])
                    self.polygons.append(Polygon3D(vert_array))


    def scaleMesh(self,sc:float):
        for i in range(len(self.polygons)):
            self.polygons[i]*=sc
        return self

    
    def invertNormals(self):
        for i in range(len(self.polygons)):
            self.polygons[i].n*=-1
        return self
    def Clone(self):
        polygons = []
        for i in range(len(self.polygons)):
            polygons.append(self.polygons[i])
        copy = Mesh3D()
        copy.polygons = polygons
        return copy
    def setTransform(self,matr:"list[list[float]]"):
        copy = self.Clone()
        for i in range(len(copy.polygons)):
            copy.polygons[i] = self.polygons[i].matrMul(matr)
        return copy

    def save(self,name:str):
        if len(self.polygons)<=0 or self.primTp != PrimitiveType.triangles:
            
            print("len: "+str(len(self.polygons))+ " must be > 0; "+str(self.primTp)+" must be PrimitiveType.triangles")
            print(name+ " not save stl")
            return
        text = "solid\n"
        n_i = 0
        print(len(self.polygons))       
        for i in range(int (len(self.polygons)/3)):
            #print(i)
            text+="facet normal "+str(self.polygons[i].n.x)+" "+str(self.polygons[i].n.y)+" "+str(self.polygons[i].n.z)+"\n "
            text+="outer loop\n"
            text+="vertex "+str(self.polygons[i].vert_arr[0].x)+" "+str(self.polygons[i].vert_arr[0].y)+" "+str(self.polygons[i].vert_arr[0].z)+"\n "
            text+="vertex "+str(self.polygons[i].vert_arr[1].x)+" "+str(self.polygons[i].vert_arr[1].y)+" "+str(self.polygons[i].vert_arr[1].z)+"\n "
            text+="vertex "+str(self.polygons[i].vert_arr[2].x)+" "+str(self.polygons[i].vert_arr[2].y)+" "+str(self.polygons[i].vert_arr[2].z)+"\n "
            text+="endloop\n"
            text+="endfacet \n"
            
            n_i+=1
        text += "endsolid\n"
        f = open(name+'.stl', 'w')
        f.write(text)
        print(name+" saved stl")
        f.close()

    def find_intersect_triangles(self, flat:Flat3D)->"list[Point3D]":
        ps = []
        for i in range(len(self.polygons)):
            ps_pol = self.polygons[i].crossFlat(flat)
            if len(ps_pol)>0:
                ps+=ps_pol
        return self.find_contour(ps) 

    def find_contour(self,ps:"list[Point3D]")->"list[Point3D]":
        if len(ps)==0:
            return []
        cont= [ps[0]]
        ps_cut = self.remove_element(ps,0)
        while(len(ps_cut)>0):
            min_d = 1000000000000
            min_ind = 0
            for j in range(len(ps_cut)):
                dist = (cont[-1]-ps_cut[j]).magnitude()
                if dist < min_d:
                    min_d = dist
                    min_ind = j
            #print(min_ind)
            cont.append(ps_cut[min_ind])
            ps_cut =  self.remove_element(ps_cut,min_ind)
        return cont

    def remove_element(self,list_in:list,ind:int):
        list_r = list_in.copy()
        del list_r[ind]
        return list_r

    def reverse_line_direct(layer:"list[Point3D]"):
        
        i = 0
        stop = 0
        if len(layer) % 2 !=0:
            stop = 1
            #print("len(layer) % 2 !=0")
        while i < len(layer)-stop:
            lam = layer[i+1].Clone()
            layer[i+1] = layer[i].Clone()
            layer[i] = lam.Clone()
            i+=2        
        return layer

    def reverse_line_direct_2(self,layer:"list[Point3D]"):       
        i = 0
        stop = 0
        if len(layer) % 2 !=0:
            stop = 1
            #print("len(layer) % 2 !=0")
        half = int(len(layer)/2)
        while i < half:
            lam = layer[i+half].Clone()
            layer[i+half] = layer[i].Clone()
            layer[i] = lam.Clone()
            i+=1      
        return layer

    def blend_list(self,list_in:list):
        list_bl = list_in.copy()
        

def rotatedX(alpha)->np.ndarray:
    c_A = np.cos(alpha)
    s_A = np.sin(alpha)
    return np.array([
        [1.,0.,0.,0.],
    [0.,c_A,-s_A,0.],
    [0.,s_A,c_A,0.],
    [0.,0.,0.,1.]])

def rotatedY(alpha)->np.ndarray:
    c_A = np.cos(alpha)
    s_A = np.sin(alpha)
    return np.array([
        [c_A,0.,s_A,0.],
        [0.,1.,0.,0.],
        [-s_A,0.,c_A,0.],
        [0.,0.,0.,1.]])

def rotatedZ(alpha)->np.ndarray:
    c_A = np.cos(alpha)
    s_A = np.sin(alpha)
    return np.array([
        [c_A,-s_A,0.,0.],
        [s_A,c_A,0.,0.],
        [0.,0.,1.,0.],
        [0.,0.,0.,1.]])

def pulse_rot_matrix(Rx,Ry,Rz)->np.ndarray:
    mx = rotatedX(Rx)
    my = rotatedY(Ry)
    mz = rotatedZ(Rz)
    mxy = np.dot( mx, my)
    return np.dot(mxy, mz)

def pulse_matrix(x,y,z,Rx,Ry,Rz)->np.ndarray:
    rot = pulse_rot_matrix(Rx,Ry,Rz)
    rot[0][3] = x
    rot[1][3] = y
    rot[2][3] = z

    #rot = np.linalg.inv(rot)
    return rot

def pulse_matrix_p(p:Point3D)->np.ndarray:
    #print(p.roll,p.pitch,p.yaw)
    rot = pulse_rot_matrix(p.roll,p.pitch,p.yaw)
    rot[0][3] = p.x
    rot[1][3] = p.y
    rot[2][3] = p.z

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

def position_from_matrix_kuka(m):
    x = m[0][3]
    y = m[1][3]
    z = m[2][3]

    b =np.arcsin(-m[2][0])

    if np.cos(b) != 0:   
        a = np.arcsin(m[2][1] / np.cos(b))
        c = np.arcsin(m[1][0] / np.cos(b))

    return x,y,z,a,b,c

def comb_angle(angle:float,case:int):
    if case ==0: return angle
    elif case ==1: return angle-np.pi
    elif case ==2: return -angle
    

    elif case ==3: return angle+np.pi
    elif case ==4: return -angle+np.pi
    
    elif case ==5: return -angle-np.pi
    elif case ==6: return angle+np.pi/2
    elif case ==7: return -angle+np.pi/2
    elif case ==8: return angle-np.pi/2
    elif case ==9: return -angle-np.pi/2


def position_from_matrix_pulse(m:np.ndarray,p_ref:Point3D = Point3D(0,0,0)):
    x = m[0][3]
    y = m[1][3]
    z = m[2][3]

    sRy = m[0][2]
    cRy = (1-sRy**2)**0.5

    sRz = -m[0][1]/cRy
    cRz = m[0][0]/cRy

    sRx = -m[1][2]/cRy
    cRx = m[2][2]/cRy
    
    Rx = np.sign(sRx)* np.arccos(cRx)
    Ry =  np.arcsin(m[0][2])
    Rz = np.sign(sRz)*np.arccos(cRz)

    return Point3D(x,y,z,_roll = Rx,_pitch = Ry, _yaw = Rz)#-np.pi -


def extract_coords_from_stl(stl_file):
    ascii = False
    for l in open(stl_file):
        if 'facet' in l:
            ascii = True
    if ascii:
        return extract_coords_from_stl_ascii(stl_file)
    else:
        return extract_coords_from_stl_bin(stl_file)

#---------------stl--ascii---------------
def extract_coords_from_stl_ascii(stl_file):
    result = []
    coords = []
    
    for l in open(stl_file):
        l = l.split()
        if l[0] == 'facet':
            result.append(list(map(float, l[-3:])))
        elif l[0] == 'vertex':
            vert = list(map(float, l[-3:]))
            result[-1] += vert
            coords.append(Point3D(vert[0],vert[1],vert[2]))
    return coords
#---------------stl---bin---------------
def unpack (f, sig, l):
    s = f.read(l)
    #fb.append(s)
    return struct.unpack(sig, s)

def read_triangle(f):
    n = unpack(f,"<3f", 12)
    p1 = unpack(f,"<3f", 12)
    p2 = unpack(f,"<3f", 12)
    p3 = unpack(f,"<3f", 12)
    b = unpack(f,"<h", 2)

    #l = len(points)
    return [Point3D(p1[0], p1[1], p1[2]),Point3D(p2[0], p2[1], p2[2]),Point3D(p3[0], p3[1], p3[2])]

def read_length(f):
    length = struct.unpack("@i", f.read(4))
    return length[0]

def read_header(f):
    f.seek(f.tell()+80)

def extract_coords_from_stl_bin(stl_file):
    result = []
    coords = []
    op = open(stl_file,"rb")
    read_header(op)
    l = read_length(op)
    for i in range(l):
        coords+= read_triangle(op)
    return coords

#--------------------------------







    

        

        
       