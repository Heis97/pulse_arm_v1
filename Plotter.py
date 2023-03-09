from PyQt5 import QtCore,  QtWidgets,QtGui
from PyQt5.QtWidgets import (QPushButton, QLineEdit, QApplication,QTextEdit,QLabel,QComboBox,QRadioButton)
from PyQt5.QtCore import Qt,QRect
from PyQt5.QtGui import QPainter,QPen,QBrush, QColor
from polygon import Point3D



class Plot(object):
    koords:"list[Point3D]"= None
    pen:QPen = None
    def __init__(self,_koords:"list[Point3D]",_color, _width):
        self.koords = _koords
        self.pen = QPen(_color,_width,Qt.SolidLine)




class Plotter(QtWidgets.QWidget):
    plots:"list[Plot]"= []
    colors = [Qt.blue,Qt.green,Qt.red]
    def __init__(self, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.setWindowTitle("Интерфейс Pulse")
        self.resize(650, 400)


    def paintEvent(self, e):
        qp = QPainter()
        qp.begin(self)
        qp.setRenderHint(QPainter.Antialiasing)
        for plot in self.plots:
            self.drawLines(qp,plot.pen,plot.koords)
        self.update()

    def drawLines(self, qp:QPainter,pen:QPen,koords:"list[Point3D]"):
        qp.setPen(pen)
        #print(koords)
        for i in range(len(koords)-1):   
            qp.drawLine(int(koords[i].x),int(koords[i].y),int(koords[i+1].x),int(koords[i+1].y))

    def addPlot(self,_koords:"list[Point3D]"):
        loc = QRect(10,10,600,300)
        self.plots.append(Plot(self.normPoints(_koords,loc),self.colors[len(self.plots)],1))

    def normPoints(self,koords:"list[Point3D]",loc:QRect):
        Xmin=100000
        Ymin=100000
        Xmax=-100000
        Ymax=-100000
        Xq1= loc.x()
        Xq2= Xq1+loc.width()
        Yq1=loc.y()
        Yq2=Yq1+loc.height()
        if len(koords)==0:
            return
        for i in range(len(koords)-1):            
            if koords[i].x>Xmax:
                Xmax=koords[i].x
            if koords[i].x<Xmin:
                Xmin=koords[i].x
            if koords[i].y>Ymax:
                Ymax=koords[i].y
            if koords[i].y<Ymin:
                Ymin=koords[i].y
        k=abs(Xq1-Xq2)/abs(Xmax-Xmin)  

        offX=Xmin*k-Xq1
        offY=Ymin*k-Yq1
        koords_n = [0]*len(koords)

        for i in range(len(koords)):   
            x1=koords[i].x
            y1=koords[i].y
            koords_n[i] =Point3D( x1*k-offX,y1*k-offY)

        return koords_n

    

