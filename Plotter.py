from PyQt5 import QtCore,  QtWidgets,QtGui
from PyQt5.QtWidgets import (QPushButton, QLineEdit, QApplication,QTextEdit,QLabel,QComboBox,QRadioButton)
from PyQt5.QtCore import Qt,QRect,QPointF,QSize
from PyQt5.QtGui import QPainter,QPen,QBrush, QColor,QPolygonF



class Plot(object):
    koords:"list[QPointF]"= None
    koords_n:"list[QPointF]"= None
    pen:QPen = None
    visible = False
    loc = None
    def __init__(self,_koords:"list[QPointF]",_color, _width):
        self.koords = _koords        
        self.pen = QPen(_color,_width,Qt.SolidLine)

    def setLoc(self,loc:QRect,sim:QPointF = None):
        if sim is not None:
            self.koords_n = self.normPoints_sim(self.koords,loc,sim)
        else:
            self.koords_n = self.normPoints(self.koords,loc)        
        self.visible = True
        self.loc = loc

    def normPoints_sim(self,koords:"list[QPointF]",loc:QRect,maxy:QPointF):

        if len(koords)==0:
            return
        
        kx=abs(loc.width())/abs(maxy.x()) 
        ky=abs(loc.height()/2)/abs(maxy.y())  
        koords_n = [0]*len(koords)

        for i in range(len(koords)):   
            x1=koords[i].x()
            y1=koords[i].y()
            koords_n[i] = QPointF(loc.x()+x1*kx,loc.y()+y1*ky+loc.height()/2)

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
            koords_n[i] = QPointF(x1*k-offX,y1*k-offY)

        return koords_n
    



class Plotter(QtWidgets.QWidget):
    plots:"list[Plot]"= []
    colors = [Qt.blue,Qt.green,Qt.red]
    size_gr:QSize = None
    board = 10
    def __init__(self, parent=None, size:QSize = QSize(700,200)):
        super().__init__(parent, QtCore.Qt.Window)
        self.setWindowTitle("Интерфейс Pulse")
        self.size_gr = size
        self.resize(size.width(),1)
        self.setStyleSheet("background-color: white;")


    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)
        qp.setRenderHint(QPainter.Antialiasing)
        for plot in self.plots:
            if plot.visible:
                self.drawPlot(qp,plot)
        self.update()

    def drawPlot(self, qp:QPainter,plot:Plot):
        qp.setPen(plot.pen)
        polig = QPolygonF(plot.koords_n)
        qp.drawPolyline(polig)
        qp.setPen(QPen(Qt.black,1,Qt.SolidLine))
        qp.drawRect(plot.loc)

    def addPlot(self,_koords:"list[QPointF]",maxy = None):
        size = self.size()
        self.resize(size.width(),size.height()+200)
        plot = Plot(_koords,self.colors[len(self.plots)],1)
        plot.setLoc(QRect(self.board,self.board+size.height(),self.size_gr.width(),self.size_gr.height()),maxy)
        self.plots.append(plot)

    def clearPlots(self):
        self.plots = []

    

    

