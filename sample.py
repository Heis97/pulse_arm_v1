import sys
from api.robot_api import RobotAPI
import time
pi = 3.1415926535
addr = "192.168.0.50"  # ip localhost?+,192.168.0.50
rr = RobotAPI(ip=addr)

def connect():
    rr.init_robot()

def set_parameters():
    rr.set_speed_scaling(1)
    rr.set_accel_scaling(1)
    rr.set_payload(0, [0.0, 0.0, 0.0])
    rr.set_tool([0, 0, 0, 0, 0, 0])
    #rr.off()

def check_io():
    rr.wait_input(0, True)
    return True

def off():
    rr.off()


def qs():
    print("rr.run()")
    #rr.run()
    #time.sleep(10)
    #rr.off()
    #if rr.hold_happened:
        #rr.hold()
    #while True:
    print("rr.move")
    rr.move_j([0, -pi/2, 0, -pi/2, 0, 0], 0.5, 0.5, blend=0)
    rr.move_j([0, -pi/2, 0, -pi/2,pi/4, 0], 0.5, 0.5, blend=0)
    rr.move_j([0, -pi/2, 0, -pi/2, 0, 0], 0.5, 0.5, blend=0)
    rr.move_j([0, -pi/2, 0, -pi/2,pi/4, 0], 0.5, 0.5, blend=0)
    rr.move_j([0, -pi/2, 0, -pi/2, 0, 0], 0.5, 0.5, blend=0)
    print("rr.run_wps")
    rr.run_wps()
    print("rr.await")
    rr.colab_await_buffer(0)
    #rr.await_motion()

def cart_1():
    rr.run()
    rr.move_l([-0.3240, -0.22, 0.22, 3.122, 0.03, 1.8], 0.1, 0.1, blend=0)
    rr.run_wps()
    rr.colab_await_buffer(0)

def cart_2():
    rr.run()
    blend = 0.003
    rr.move_l([-0.3240, -0.22, 0.22, 3.122, 0.03, 1.8], 0.1, 0.1, blend=blend)
    rr.move_l([-0.4240, -0.22, 0.22, 3.122, 0.03, 1.8], 0.1, 0.1, blend=blend)
    rr.move_l([-0.4240, -0.32, 0.22, 3.122, 0.03, 1.8], 0.1, 0.1, blend=blend)
    rr.move_l([-0.3240, -0.32, 0.22, 3.122, 0.03, 1.8], 0.1, 0.1, blend=blend)
    rr.move_l([-0.3240, -0.22, 0.22, 3.122, 0.03, 1.8], 0.1, 0.1, blend=blend)
    rr.run_wps()
    rr.colab_await_buffer(0)

def home():
    print("rr.run()")
    rr.init_robot()
    #rr.run()
    #time.sleep(10)
    #rr.off()
    #if rr.hold_happened:
        #rr.hold()
    #while True:
    print("rr.move")
    rr.move_j([0, -pi/2, 0, -pi/2, 0, 0], 0.1, 0.1, blend=0)
    print("rr.run_wps")
    rr.run_wps()
    rr.await_motion()
    #rr.run_wps()
    #print("rr.await")
    #rr.colab_await_buffer(0)
    #rr.await_motion()
    


if __name__ == '__main__':
    print("rr.connect")
    connect()
    #set_parameters()
    #rr.run()

    #time.sleep(30)
    #rr.off()
    #rr.stby()
    #time.sleep(30)
    #rr.run()
    #cart_2()
    print(rr.ctrl.data["act_x"])
    print(rr.fkine(rr.ctrl.data["act_q"]))
    print(rr.ctrl.data["act_q"])
    print(rr.ikine(rr.ctrl.data["act_x"]))
    #print(rr.get_dh_model())
    #qs()
    #off()
    #home()
    #time.sleep(20)

    
    #rr.colab_await_buffer(0)
    #sys.exit()