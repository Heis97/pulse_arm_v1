

from PyQt5.QtWidgets import QOpenGLWidget
from PyQt5.QtCore import Qt
from PyQt5.QtGui import  QColor, QMouseEvent
from PyQt5.QtCore import ( QPoint,QPointF, QSize, Qt)
import OpenGL.GL as gl
from polygon import Mesh3D, Point3D,PrimitiveType,extract_coords_from_stl
import numpy as np

class Paint_in_GL(object):
    visible = True
    glList = None
    matrs:"list[list[list[float]]]" = None
    red = 0.
    green = 0.
    blue = 0.
    size = 1.
    alpha = 1
    norm:"list[Point3D]" = None
    obj_type:PrimitiveType
    points:"list[Point3D]"
    p2 = []
    p3 = []
    matr_off = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])
    matr = np.array([[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]])

    mesh_obj: Mesh3D = None
    def __init__(self, _red, _green, _blue, _size, _type:PrimitiveType, _mesh_obj:Mesh3D):
        self.red = _red
        self.green = _green
        self.blue = _blue
        self.size = _size
        self.obj_type = _type
        self.mesh_obj = _mesh_obj

    def setTrasform(self,matr:"list[list[float]]"):
        trans_mesh = self.mesh_obj.setTransform(matr)
        return trans_mesh

    #сохранить stl
    def save(self,name:str):
        if(len(self.points)==0 or len(self.norm)==0 or self.obj_type != PrimitiveType.triangles):
            return
        text = "solid\n"
        n_i = 0
        print(len(self.points))       
        for i in range(int (len(self.points)/3)):
            #print(i)
            text+="facet normal "+str(self.norm[n_i].x)+" "+str(self.norm[n_i].y)+" "+str(self.norm[n_i].z)+"\n "
            text+="outer loop\n"
            text+="vertex "+str(self.points[i*3].x)+" "+str(self.points[i*3].y)+" "+str(self.points[i*3].z)+"\n "
            text+="vertex "+str(self.points[i*3+1].x)+" "+str(self.points[i*3+1].y)+" "+str(self.points[i*3+1].z)+"\n "
            text+="vertex "+str(self.points[i*3+2].x)+" "+str(self.points[i*3+2].y)+" "+str(self.points[i*3+2].z)+"\n "
            text+="endloop\n"
            text+="endfacet \n"            
            n_i+=1
        text += "endsolid\n"
        f = open(name+'.stl', 'w')
        f.write(text)
        f.close()
        
class GLWidget(QOpenGLWidget):
    traj_objs:"list[Paint_in_GL]"  = []
    paint_objs:"list[Paint_in_GL]"  = []
    cont_select:bool = False
    rot:bool = True
    trans:bool = True
    cont:"list[Point3D]" = None
    render_count = 0
    lightPower = 20.
    lightZeroPosition = [400.,400.,400.,1.]
    lightZeroColor = [lightPower,lightPower,lightPower,1.0]
    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)
        format = self.format()
        format.setSamples(8)
        
        self.setFormat(format)
        self.object = 0
        self.xRot = 2200
        self.yRot = 0
        self.zRot = 1200
        self.off_x=0.0
        self.off_y=0.0
        self.lastPos = QPoint()
        self.zoom=70
        self.trolltechGreen = QColor.fromCmykF(1.0, 0.5, 0.5, 0.0)
        self.trolltechGreen1 = QColor.fromCmykF(1.0, 0.7, 0.7, 0.0)
        self.trolltechRed = QColor.fromCmykF(0.0, 1.0, 1.0, 0.0)
        self.trolltechPurple = QColor.fromCmykF(0.0, 0.0, 0.0, 0.0)
        self.l2=[]
        self.w = 1000
        self.h = 1000

    def setXY(self):
        self.xRot = 0#2200
        self.yRot = 0
        self.zRot = 0#1200
        self.off_x=0.0
        self.off_y=0.0
        self.zoom=100

#--------------
    def initializeGL(self):
        self.setClearColor(self.trolltechPurple)
        gl.glShadeModel(gl.GL_FLAT)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_MULTISAMPLE)    
        

        #gl.glEnable(gl.GL_CULL_FACE) 
        gl.glEnable(gl.GL_LIGHTING)
        gl.glEnable(gl.GL_LIGHT0)
        gl.glEnable(gl.GL_COLOR_MATERIAL)
         
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, self.lightZeroPosition)
        gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE, self.lightZeroColor)
        
        gl.glLightf(gl.GL_LIGHT0, gl.GL_CONSTANT_ATTENUATION, 0.1)
        gl.glLightf(gl.GL_LIGHT0, gl.GL_LINEAR_ATTENUATION, 0.05)
        
        #gl.glDisable(gl.GL_LIGHTING)
        #gl.glDisable(gl.GL_LIGHT0)
        self.resizeGL(self.w,self.h)
        #self.getOpenglInfo()

    
    def initPaint_in_GL(self,  paint_gls: Paint_in_GL): 
        mesh_obj = paint_gls.mesh_obj
        if(paint_gls.matrs!=None):
            ind_m = self.render_count % len(paint_gls.matrs)
            mesh_obj = paint_gls.setTrasform(paint_gls.matrs[ind_m])
        genList = [gl.glGenLists(1)]
        gl.glNewList(genList[-1], gl.GL_COMPILE)  
        v = paint_gls
        gl.glLineWidth(v.size)
        gl.glPointSize(10*v.size)
        color = QColor.fromCmykF(v.red, v.green, v.blue, 0.0)
        gl.glMaterialfv(gl.GL_FRONT_AND_BACK, gl.GL_DIFFUSE, (v.red, v.green, v.blue))
        self.setColor(color)
        if v.obj_type == PrimitiveType.points:
            gl.glBegin(gl.GL_POINTS)
            len_points = len(v.mesh_obj.polygons)
            for j in range(len_points):  
                p1 = v.mesh_obj.polygons[j].vert_arr[0] 
                gl.glVertex3d(p1.x,p1.y,p1.z)
                gl.glNormal3d(0.5, 0.5, 0.5)
            gl.glEnd()

        elif v.obj_type == PrimitiveType.lines:
            gl.glEnable(gl.GL_LINE_SMOOTH)
            gl.glLineStipple(2,58360)
            gl.glLineWidth(3.*v.size)
            gl.glBegin(gl.GL_LINES)
            len_points = len(v.mesh_obj.polygons)

            for j in range(len_points):                
                p1 = v.mesh_obj.polygons[j].vert_arr[0]  
                p2 = v.mesh_obj.polygons[j].vert_arr[1] 
                color1 = QColor.fromCmykF(p2.r, p2.g, p2.b, 0.0)
                #color1 = QColor.fromCmykF(v.red, v.green, v.blue, 0.0)
                #self.setColor(color1)
                if p2.extrude == True:
                    gl.glVertex3d(p1.x,p1.y,p1.z)
                    gl.glVertex3d(p2.x,p2.y,p2.z)
                    gl.glNormal3d(0.5, 0.5, 0.5)
            gl.glEnd()
            gl.glEndList()

            genList.append(gl.glGenLists(1))
            gl.glNewList(genList[-1], gl.GL_COMPILE)  
            gl.glLineWidth(v.size)
            gl.glEnable(gl.GL_LINE_SMOOTH)
            gl.glLineStipple(2,58360)
            gl.glEnable(gl.GL_LINE_STIPPLE)  
            gl.glBegin(gl.GL_LINES)
            len_points = len(v.mesh_obj.polygons)
            for j in range(len_points):
                
                p1 = v.mesh_obj.polygons[j].vert_arr[0]  
                p2 = v.mesh_obj.polygons[j].vert_arr[1] 
                #color1 = QColor.fromCmykF(p2.r, p2.g, p2.b, 0.0)
                color1 = QColor.fromCmykF(v.red, v.green, v.blue, 0.0)
                self.setColor(color1)
                if p2.extrude == False:
                    color1 = QColor.fromCmykF(0., 1., 1., 0.0)
                    self.setColor(color1)                         
                    gl.glVertex3d(p1.x,p1.y,p1.z)
                    gl.glVertex3d(p2.x,p2.y,p2.z)
                    gl.glNormal3d(0.5, 0.5, 0.5)
            gl.glEnd()
            gl.glDisable(gl.GL_LINE_STIPPLE)  
        
        elif v.obj_type == PrimitiveType.triangles:

            gl.glBegin(gl.GL_TRIANGLES)
            
            len_points = len(mesh_obj.polygons)
            for j in range(len_points):   
                p1 = mesh_obj.polygons[j].vert_arr[0]  
                p2 = mesh_obj.polygons[j].vert_arr[1] 
                p3 = mesh_obj.polygons[j].vert_arr[2]   
                n = mesh_obj.polygons[j].n
                if(n !=None):        
                    gl.glNormal3d(n.x, n.y, n.z)
                gl.glVertex3d(p1.x,p1.y,p1.z)
                gl.glVertex3d(p2.x,p2.y,p2.z)
                gl.glVertex3d(p3.x,p3.y,p3.z)
            
            gl.glEnd()

        gl.glEndList()

        return genList
            
    def GL_paint(self,  paint_gls: "list[Paint_in_GL]"):
        for i in range(len(paint_gls)):
            gl.glMatrixMode(gl.GL_MODELVIEW)
            gl.glLoadIdentity()
            gl.glLoadMatrixd(np.dot(paint_gls[i].matr_off,paint_gls[i].matr))     

            if paint_gls[i].glList==None:
                paint_gls[i].glList = self.initPaint_in_GL(paint_gls[i])
            else:
                for gl_list in paint_gls[i].glList:                  
                    
                    if paint_gls[i].obj_type == PrimitiveType.triangles:
                        gl.glEnable(gl.GL_LIGHTING)
                        gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, self.lightZeroPosition)
                        gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE, [self.lightPower,self.lightPower,self.lightPower,1.0])
                    if paint_gls[i].visible==True:
                        gl.glCallList(gl_list)
                    gl.glDisable(gl.GL_LIGHTING)


    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        #gl.glScalef(self.zoom,self.zoom,self.zoom)
        self.render_count+=1
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(-self.w/ self.zoom ,self.w/  self.zoom , -self.h/  self.zoom , self.h/  self.zoom , -20000., 50000.0)
        gl.glTranslated(self.off_x, self.off_y,-10.)
        gl.glRotated(self.xRot, 1.0, 0.0, 0.0)
        gl.glRotated(self.yRot, 0.0, 1.0, 0.0)
        gl.glRotated(self.zRot, 0.0, 0.0, 1.0)
        

        self.GL_paint(self.paint_objs)
        self.GL_paint(self.traj_objs)
        self.update()


        
    def getOpenglInfo(self):
        
        #print() 
        print(gl.glGetString(gl.GL_RENDERER))


    def minimumSizeHint(self):
        return QSize(50, 50)

    def sizeHint(self):
        return QSize(1200, 800)

    def compNorm(self, p1:Point3D,p2:Point3D,p3:Point3D):
        u = Point3D(p3.x-p1.x,p3.y-p1.y,p3.z-p1.z)
        v = Point3D(p2.x-p1.x,p2.y-p1.y,p2.z-p1.z)
        #print("u: "+str(u.x)+" "+str(u.y)+" "+str(u.z)+" ")
        #print("v: "+str(v.x)+" "+str(v.y)+" "+str(v.z)+" ")
        Norm = Point3D(
            u.y * v.z - u.z * v.y,
            u.z * v.x - u.x * v.z,
            u.x * v.y - u.y * v.x)
        #print("Norm.x: "+str(Norm.x))
        Norm.normalyse()
        #print("norm: "+str(Norm.x)+" "+str(Norm.y)+" "+str(Norm.z)+" ")
        return Norm

    def gridToTriangleMesh(self,points2d:"list[list[Point3D]]"):
        points1d:list[Point3D] = []
        
        for i in range(len(points2d)-1):
            for j in range(len(points2d[0])-1):

                points1d.append( points2d[i][j+1])
                points1d.append(points2d[i][j])
                points1d.append(points2d[i+1][j])

                points1d.append(points2d[i+1][j])
                points1d.append(points2d[i+1][j+1])
                points1d.append(points2d[i][j+1])

        return points1d

    def resizeGL(self, width, height):
        side = min(width, height)
        if side < 0:
            return

        gl.glViewport((width - side) // 2, (height - side) // 2, side,
                           side)
        scale = 1.
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        gl.glOrtho(-width/ self.zoom , width/  self.zoom , -height/  self.zoom , height/  self.zoom , -20000., 50000.0)
        
        gl.glMatrixMode(gl.GL_MODELVIEW)


    def wheelEvent(self, event):
        wheelcounter = event.angleDelta()
        if wheelcounter.y() < 0 :
            if self.zoom<0.02:
                pass
            else:
                self.zoom*=0.7
        elif wheelcounter.y() > 0:
            self.zoom/=0.7

        #print(self.zoom)
        self.update()
    
    def mousePressEvent(self, event:QMouseEvent):
        self.lastPos = event.pos()
        if self.cont_select:
            pf = self.toSurfCoord(self.lastPos)
            self.cont.append(Point3D(pf.x(),pf.y(),0))

    def mouseMoveEvent(self, event:QMouseEvent):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if (event.buttons() & Qt.LeftButton) and self.rot:
            self.xRot += dy
            self.zRot += dx
        if (event.buttons() & Qt.RightButton) and self.trans:
            self.off_x+=2*dx
            self.off_y-=2*dy

        self.lastPos = event.pos()
        self.update()

    def toSurfCoord(self,p_widg:QPoint):
        scale = 2/(self.zoom)
        x = (p_widg.x()-self.w/2)*scale
        y = -(p_widg.y()-self.h/2)*scale
        
        return QPointF(x,y)

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

    def setClearColor(self, c):
        gl.glClearColor(c.redF(), c.greenF(), c.blueF(), c.alphaF())

    def setColor(self, c):
        gl.glColor4f(c.redF(), c.greenF(), c.blueF(), c.alphaF())  

    def addLines(self, traj:"list[Point3D]",r:float,g:float,b:float,size:float):
        mesh3d_traj = Mesh3D(traj,PrimitiveType.lines)
        self.traj_objs.append(Paint_in_GL(r,g,b,size,PrimitiveType.lines,mesh3d_traj))

    def addPoimts(self, traj:"list[Point3D]",r:float,g:float,b:float,size:float):
        mesh3d_traj = Mesh3D(traj,PrimitiveType.points)
        self.traj_objs.append(Paint_in_GL(r,g,b,size,PrimitiveType.points,mesh3d_traj))

    def addLinesDef(self, traj:"list[Point3D]",r:float,g:float,b:float,size:float):
        mesh3d_traj = Mesh3D(traj,PrimitiveType.lines_def)
        self.traj_objs.append(Paint_in_GL(r,g,b,size,PrimitiveType.lines,mesh3d_traj))

    def addLines_ret(self, traj:"list[Point3D]",r:float,g:float,b:float,size:float)->int:
        mesh3d_traj = Mesh3D(traj,PrimitiveType.lines)
        self.paint_objs.append(Paint_in_GL(r,g,b,size,PrimitiveType.lines,mesh3d_traj))
        return len(self.paint_objs)-1
    
    def addTriangles_ret(self, traj:"list[Point3D]",r:float,g:float,b:float,size:float)->int:
        mesh3d_traj = Mesh3D(traj,PrimitiveType.triangles)
        self.traj_objs.append(Paint_in_GL(r,g,b,size,PrimitiveType.triangles,mesh3d_traj))
        return len(self.traj_objs)-1
    
    def addModel_ret(self, stl_file:str,r:float = 0.5,g:float= 0.5,b:float= 0.5,size:float= 0.5)->int:
        model = extract_coords_from_stl(stl_file)
        mesh3d_model = Mesh3D(model,PrimitiveType.triangles)
        self.paint_objs.append(Paint_in_GL(r,g,b,size,PrimitiveType.triangles,mesh3d_model))
        return len(self.paint_objs)-1
    

    def setMatr(self,matr,ind):
        self.paint_objs[ind].matr = np.transpose(matr)

    def setMatr_off(self,matr,ind):
        self.paint_objs[ind].matr_off = np.transpose(matr)

    

    


    def clear(self):
        self.paint_objs= []
    def clear_traj(self):
        self.traj_objs= []    

    def draw_start_frame(self,size:float):
        frame1 = Mesh3D( [Point3D(0, 0, 0,True,0,1,1),Point3D(size, 0, 0,True,0,1,1)] ,PrimitiveType.lines)
        frame2 = Mesh3D( [Point3D(0, 0, 0,True,1,0,1),Point3D( 0,size, 0,True,1,0,1)],PrimitiveType.lines)
        frame3 = Mesh3D( [Point3D(0, 0, 0,True,1,1,0),Point3D( 0, 0,size,True,1,1,0)] ,PrimitiveType.lines)

        self.paint_objs.append(Paint_in_GL(0,1,1.0,0.3,PrimitiveType.lines,frame1))
        self.paint_objs.append(Paint_in_GL(1.0,0,1.0,0.3,PrimitiveType.lines,frame2))
        self.paint_objs.append(Paint_in_GL(1.0,1,0,0.3,PrimitiveType.lines,frame3))

    def draw_frame(self,matr: "list[Point3D]"):
        points = self.createFrame(matr, 1)
        frame1 = Mesh3D( points[0:2] ,PrimitiveType.lines)
        frame2 = Mesh3D( points[2:4],PrimitiveType.lines)
        frame3 = Mesh3D( points[4:6] ,PrimitiveType.lines)

        self.paint_objs.append(Paint_in_GL(0,1,1.0,4,PrimitiveType.lines,frame1))
        self.paint_objs.append(Paint_in_GL(1.0,0,1.0,4,PrimitiveType.lines,frame2))
        self.paint_objs.append(Paint_in_GL(1.0,1,0,4,PrimitiveType.lines,frame3))

    def createFrame(self,matrix,dim:float):
        p1 = Point3D(matrix[0][3],matrix[1][3],matrix[2][3])
        p2 = Point3D(dim*matrix[0][0],dim*matrix[0][1],dim*matrix[0][2])
        p3 = Point3D(dim*matrix[1][0],dim*matrix[1][1],dim*matrix[1][2])
        p4 = Point3D(dim*matrix[2][0],dim*matrix[2][1],dim*matrix[2][2])
        ps = []
        ps.append(p1) 
        ps.append(p1+p2)
        ps.append(p1) 
        ps.append(p1+p3)
        ps.append(p1) 
        ps.append(p1+p4) 
        return ps

    def setLight(self,var:int,val:float):
        if var == 0:
            self.lightZeroPosition[0] = 5*val
        elif var == 1:
            self.lightZeroPosition[1] = 5*val
        elif var == 2:
            self.lightZeroPosition[2] = 5*val
        elif var == 3:
            self.lightPower =5* val

        #gl.glEnable(gl.GL_LIGHTING)
        #gl.glLightfv(gl.GL_LIGHT0, gl.GL_POSITION, self.lightZeroPosition)
        #gl.glLightfv(gl.GL_LIGHT0, gl.GL_DIFFUSE, [self.lightPower,self.lightPower,self.lightPower,1.0])


