import numpy as np
import math 
import enum
from random import triangular



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
    
    def __init__(self,_x:float,_y:float,_z:float,_extrude:bool = True,_r:float = 0.0,_g:float = 1.,_b:float =1.,_pitch:float = 0.0,_roll:float = 1.,_yaw:float =1.):
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

    def normalyse(self):
        mod = abs(self.x*self.x)+abs(self.y*self.y)+abs(self.z*self.z)
        norm = math.sqrt(mod)
        if norm!=0:
            self.x /=norm
            self.y /=norm
            self.z /=norm
        return Point3D(self.x,self.y,self.z,self.extrude)

    def ToString(self)->str:
        return str(self.x)+" "+str(self.y)+" "+str(self.z)+";"

    def ToStringPulse(self)->str:
        return str(self.x)+" "+str(self.y)+" "+str(self.z)+" "+str(self.pitch)+" "+str(self.roll)+" "+str(self.yaw)+";"

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

        xa = self.x + other.x
        ya = self.y + other.y
        za = self.z + other.z
        return Point3D(xa,ya,za,self.extrude)

    def __sub__(self, other):

        xa =  self.x- other.x
        ya = self.y - other.y
        za = self.z - other.z
        #self.x-=other.x
        #self.y-=other.y
        #self.z-=other.z
        return Point3D(xa,ya,za,self.extrude)

    def __neg__(self):
        self.x = -self.x
        self.y = -self.y
        self.z = -self.z
        return self

    def Clone(self):
        return Point3D(self.x,self.y,self.z,self.extrude,self.r,self.g,self.b)

    def __mul__(self, other):
        if(type(other)==Point3D):
            return Point3D(self.y*other.z-self.z*other.y, self.z*other.x-self.x*other.z,self.x*other.y-self.y*other.x,self.extrude)
        elif(type(other)==np.ndarray):
            x,y,z,Rx,Ry,Rz = position_from_matrix(np.dot(pulse_matrix_p(self),other))
            return Point3D(x,y,z,_pitch= Rx,_roll=Ry,_yaw=Rz)
        else:
            return Point3D(self.x*other,self.y*other,self.z*other,self.extrude)

    def __pow__(self, other):
        return self.x*other.x +self.y*other.y+self.z*other.z

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
    rot = pulse_rot_matrix(p.pitch,p.roll,p.yaw)
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

def position_from_matrix_pulse(m):
    x = m[0][3]
    y = m[1][3]
    z = m[2][3]

    Ry =np.arcsin(m[0][2])

    sRy = m[0][2]
    cRy = (1-sRy**2)**0.5

    sRz = -m[0][1]/cRy
    cRz = m[0][0]/cRy

    sRx = -m[1][2]/cRy
    cRx = m[2][2]/cRy

    Rx = np.arcsin(sRx)
    Ry = np.arcsin(sRy)
    Rz = np.arcsin(sRz)

    print("Rx,Ry,Rz1")
    print(Rx,Ry,Rz)
    
    Rx = np.arccos(cRx)
    Ry = np.arccos(cRy)
    Rz = np.arccos(cRz)

    #print("Rx,Ry,Rz")
    print(Rx,Ry,Rz)



    #if np.cos(Ry) != 0:   
        #Rz = np.arcsin(-m[0][1] / np.cos(Ry))
        #Rx = np.arccos(-m[2][2] / np.cos(Ry))

    return Point3D(x,y,z,_pitch =  Rx,_roll= Ry, _yaw =Rz)#-np.pi -











    

        

        
       