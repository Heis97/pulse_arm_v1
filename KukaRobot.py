import time,socket
from polygon import *


class KukaRobot(object):
    host = '172.31.1.147'
    port = 30005
    s = None
    def __init__(self) -> None:
        pass
    def connect(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        addr = (self.host,self.port)
        try:
            self.s.connect(addr)
        except BaseException:
            self.s = None
            print('not connect')
        

    def resieve(self)->str:
        if self.s is not None:
            data = self.s.recv(1000)
            res=str(data.decode('utf-8'))
            
            return res

    def send(self,mes:str):
        if self.s is not None:
            print(mes)
            self.s.send(mes.encode('utf-8'))

    def close(self):
        if self.s is not None:
            self.s.close()

    def send_pos(self, x,y,z,a,b,c):
        mes = x+" "+y+" "+z+" "+a+" "+b+" "+c+" 10\n"
        self.send(mes)

    def res_pos(self)-> tuple:
        self.send("f\n")
        time.sleep(0.05)
        res = self.resieve().strip().split()
        return float(res[0]),float(res[1]),float(res[2]),float(res[3]),float(res[4]),float(res[5])


    