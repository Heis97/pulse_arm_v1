from PyQt5 import QtCore,  QtWidgets,QtGui
from PyQt5.QtWidgets import (QPushButton, QLineEdit, QApplication,QTextEdit,QLabel,QComboBox,QRadioButton)
from PyQt5.QtCore import Qt,QRect,QPointF,QSize
from PyQt5.QtGui import QPainter,QPen,QBrush, QColor,QPolygonF



class Plot(object):
    koords:"list[QPointF]"= None
    koords_n:"list[QPointF]"= None
    pen:QPen = None
    name:str = ""
    visible = False
    ymm:QPointF = None

    loc = None
    def __init__(self,koords:"list[QPointF]",color, width,name):
        self.koords = koords        
        self.pen = QPen(color,width,Qt.SolidLine)
        self.name = name

    def setLoc(self,loc:QRect,sim:QPointF = None):
        if sim is not None:
            self.koords_n = self.normPoints_sim(self.koords,loc,sim)
        else:
            self.koords_n = self.normPoints_y(self.koords,loc)        
        self.visible = True
        self.loc = loc

    def normPoints_sim(self,koords:"list[QPointF]",loc:QRect,maxy:QPointF):

        if len(koords)==0:
            return
        xmm:QPointF = None
        kx=abs(loc.width())/abs(maxy.x()) 
        ky=abs(loc.height()/2)/abs(maxy.y())  
        koords_n = [0]*len(koords)

        for i in range(len(koords)):   
            x1=koords[i].x()
            y1=koords[i].y()
            koords_n[i] = QPointF(loc.x()+x1*kx,loc.y()-y1*ky+loc.height()/2)

        return koords_n

    def normPoints(self,koords:"list[QPointF]",loc:QRect):
        Xmin=10e20
        Ymin=10e20
        Xmax=-10e20
        Ymax=-10e20
        Xq1= loc.x()
        Xq2= Xq1+loc.width()
        Yq1= loc.y()
        Yq2= Yq1+loc.height()
        if len(koords)==0:
            return
        for i in range(len(koords)-1):            
            if koords[i].x()>Xmax:
                Xmax=koords[i].x()
            if koords[i].x()<Xmin:
                Xmin=koords[i].x()
            if koords[i].y()>Ymax:
                Ymax=koords[i].y()
            if koords[i].y()<Ymin:
                Ymin=koords[i].y()
        k=abs(Xq1-Xq2)/abs(Xmax-Xmin)  

        k2=abs(Yq1-Yq2)/abs(Ymax-Ymin)

        if k2 > k:
            k = k2

        offX=Xmin*k-Xq1
        offY=Ymin*k-Yq1
        koords_n = [0]*len(koords)

        for i in range(len(koords)):   
            x1=koords[i].x()
            y1=koords[i].y()
            koords_n[i] = QPointF(x1*k-offX,-y1*k-offY)

        return koords_n
    
    def normPoints_y(self,koords:"list[QPointF]",loc:QRect,):
        Xmin=10e20
        Ymin=10e20
        Xmax=-10e20
        Ymax=-10e20
        Xq1= loc.x()
        Xq2= Xq1+loc.width()
        Yq1= loc.y()
        Yq2= Yq1+loc.height()
        if len(koords)==0:
            return
        for i in range(len(koords)-1):            
            if koords[i].x()>Xmax:
                Xmax=koords[i].x()
            if koords[i].x()<Xmin:
                Xmin=koords[i].x()
            if koords[i].y()>Ymax:
                Ymax=koords[i].y()
            if koords[i].y()<Ymin:
                Ymin=koords[i].y()
        
        kx = 1
        if Xmax!=Xmin: 
            kx=abs(Xq1-Xq2)/abs(Xmax-Xmin) 
        ky = 1
        if Ymax!=Ymin: 
            ky=abs(Yq1-Yq2)/abs(Ymax-Ymin)

        offX=Xmin*kx-Xq1
        offY=Ymin*ky-Yq1

        self.ymm = QPointF(Ymin,Ymax)
        koords_n = [0]*len(koords)

        for i in range(len(koords)):   
            x1=koords[i].x()
            y1=koords[i].y()
            koords_n[i] = QPointF(x1*kx-offX,y1*ky-offY)

        return koords_n
    



class Plotter(QtWidgets.QWidget):
    plots:"list[Plot]"= []
    colors = [Qt.yellow,Qt.green,Qt.red]*100
    size_gr:QSize = None
    board = 10
    col = 1
    row = 1
    def __init__(self, parent=None, size:QSize = QSize(400,200)):
        super().__init__(parent, QtCore.Qt.Window)
        self.setWindowTitle("Интерфейс Pulse")
        self.size_gr = size
        self.resize(size.width()+2*self.board,1)
        self.setStyleSheet("background-color:  black;")


    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)
        qp.setRenderHint(QPainter.Antialiasing)
        for plot in self.plots:
            if plot.visible and len(plot.koords)>0:
                self.drawPlot(qp,plot)
        #self.update()

    def drawPlot(self, qp:QPainter,plot:Plot):
        qp.setPen(plot.pen)
        polig = QPolygonF(plot.koords_n)
        qp.drawPolyline(polig)
        qp.setPen(QPen(Qt.gray,1,Qt.SolidLine))
        qp.drawRect(plot.loc)
        qp.drawLine(plot.loc.x(),int(plot.loc.y()+plot.loc.height()/2),plot.loc.x()+plot.loc.width(),int(plot.loc.y()+plot.loc.height()/2))
        qp.setFont(QtGui.QFont("Times", 12, QtGui.QFont.Bold))
        qp.drawText(plot.loc.x()+13,plot.loc.y()+25,plot.name)
        if plot.ymm is not None:
            qp.drawText(plot.loc.x()+43,plot.loc.y()+20,str(round(plot.ymm.y(),4)))
            qp.drawText(plot.loc.x()+43,plot.loc.y()+plot.loc.height()-20,str(round(plot.ymm.x(),4)))

    def addPlot(self,koords:"list[QPointF]",name:str = "",row: int = 1,col:int = 1,maxy = None):
        self.col = max(self.col,col)
        self.row = max(self.row,row)      
        self.resize(2*self.board+self.col *(self.board+self.size_gr.width()),
                    2*self.board+self.row *(self.board+self.size_gr.height()))
        plot = Plot(koords,self.colors[len(self.plots)],1,name)
        loc = QRect(self.board+(col-1)*(self.board+self.size_gr.width()),
                    self.board+(row-1)*(self.board+self.size_gr.height()),
                    self.size_gr.width(),
                    self.size_gr.height())
        plot.setLoc(loc,maxy)
        
        self.plots.append(plot)

    def clearPlots(self):
        self.plots = []

    

    

