#!/usr/bin/env python3
import copy
from typing import Callable, Any
COMS_PORT = 29001
RTD_PORT = 29000

from .ctrl import Control
import logging
import sys
import socket
import struct
import math
import time

logging.basicConfig(level=logging.DEBUG)

def _to_rad(x):
    return x * math.pi / 180.0

CTRLR_COMS_STOP = 0
CTRLR_COMS_MOVE_ADD_WP = 1
CTRLR_COMS_MOVE_RUN = 2
CTRLR_COMS_MOVE_SCALE = 3
CTRLR_COMS_RESERVED0 = 4
CTRLR_COMS_POWER = 5
CTRLR_COMS_STOP_SMOOTH = 6
CTRLR_COMS_RESERVED2 = 7
CTRLR_COMS_JOG = 8
CTRLR_COMS_NOP = 9
CTRLR_COMS_RESERVED3 = 10
CTRLR_COMS_RESERVED4 = 11
CTRLR_COMS_RESERVED5 = 12
CTRLR_COMS_RESERVED6 = 13
CTRLR_COMS_SET_OUTPUTS = 14
CTRLR_COMS_ZG = 15
#control unlocking
CTRLR_COMS_UNLOCK = 100
#settings functions
CTRLR_COMS_SET_GRAVITY = 1016
CTRLR_COMS_GET_GRAVITY = 1116
CTRLR_COMS_SET_ZG_FSCALE = 1017
CTRLR_COMS_GET_ZG_FSCALE = 1117
CTRLR_COMS_SET_TRQ_WIN = 1018
CTRLR_COMS_GET_TRQ_WIN = 1118
CTRLR_COMS_SET_FLW_ERR = 1019
CTRLR_COMS_GET_FLW_ERR = 1119
CTRLR_COMS_SET_MAX_VEL = 1020
CTRLR_COMS_GET_MAX_VEL = 1120
CTRLR_COMS_SET_PAYLOAD = 1021
CTRLR_COMS_GET_PAYLOAD = 1121
CTRLR_COMS_SET_TOOL = 1022
CTRLR_COMS_GET_TOOL = 1122
CTRLR_COMS_SET_JOG_PARAM = 1023
CTRLR_COMS_GET_JOG_PARAM = 1123
CTRLR_COMS_SET_FORCE_PARAM = 1024
CTRLR_COMS_GET_FORCE_PARAM = 1124
CTRLR_COMS_SET_IO_FUNC = 1025
CTRLR_COMS_GET_IO_FUNC = 1125
CTRLR_COMS_SET_DH_MODEL = 1026
CTRLR_COMS_GET_DH_MODEL = 1126
CTRLR_COMS_SET_TOOL_CAPSULE_COUNT = 1027
CTRLR_COMS_GET_TOOL_CAPSULE_COUNT = 1127
CTRLR_COMS_SET_TOOL_CAPSULE = 1028
CTRLR_COMS_GET_TOOL_CAPSULE = 1128
CTRLR_COMS_GET_LINK_CAPSULE_COUNT = 1129
CTRLR_COMS_GET_LINK_CAPSULE = 1130
CTRLR_COMS_GET_RUN_TIME = 3005

#service functions
CTRLR_COMS_FKINE = 2000
CTRLR_COMS_IKINE = 2001

CTRLR_COMS_POWER_CMD_OFF = 0
CTRLR_COMS_POWER_CMD_STBY = 1
CTRLR_COMS_POWER_CMD_ON = 2
CTRLR_COMS_POWER_CMD_RUN = 3

#motion types for add_wp command
MOVE_WP_TYPE_JOINT = 0
MOVE_WP_TYPE_LINEAR_CART = 1
MOVE_WP_TYPE_LINEAR_POSE = 2

CTRLR_PROTO_VERSION = 0x02000100

ponts_size = 0

MOVE_L = 'cartesian'
MOVE_J = 'degrees'


class Variables:

    def __init__(self):
        self.t = 0
        self.des_q = [0, 0, 0, 0, 0, 0]
        self.des_x = [0, 0, 0, 0, 0, 0]
        self.force = [0, 0, 0, 0, 0, 0]
        self.force_en = [0, 0, 0, 0, 0, 0]
        self.in_tcp = 0
        self.vmax_t = 0
        self.vmax_r = 0
        self.amax_t = 0
        self.amax_r = 0
        self.vmax_j = 0
        self.amax_j = 0
        self.rblend = 0


class RobotAPI:
    def __init__(self, ip):
        self.coms_port = COMS_PORT
        self.rtd_port = RTD_PORT
        self.ctrl = None
        self.ip = ip
        self.logger = logging.getLogger(__name__)
        self.socket = None
        self.vars = Variables()
        self._cmd_cntr = None
        self.scale_v = 0
        self.scale_a = 0
        self.brake_deceleration = 0

        self.waypoints_list = []

    def _init_control(self):
        self.ctrl = Control(ip=self.ip)
        if not self.ctrl.start_thread():
            self.ctrl = None
        return True


    def _connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ip, self.coms_port))
            self.socket.settimeout(3)
            self.socket.setblocking(True)
            logging.debug(f"Socket connect [{self.ip}:{self.coms_port}] --> Ok")
            self._cmd(CTRLR_COMS_UNLOCK, struct.pack("I", CTRLR_PROTO_VERSION));
            return True
        except Exception as error:
            logging.error(
                f"Socket connect [{self.ip}:{self.coms_port}] --> False\n{error}"
            )
            self._stop()
            return False

    def _recv(self, l):
        try:
            d = self.socket.recv(l)
            if d == b'':
                print("CTRL connection lost")
                self._stop()
        except Exception as error:
            self._stop()
            return False

        return d

    def _send(self, d):
        try:
            s = self.socket.send(d)
            if s == 0:
                print("CTRL connection lost")
                self._stop()
        except Exception as error:
            self._stop()
            return False

    def _cmd(self, cmd_type, data = []):
        self._send(struct.pack("i", len(data) + 4))
        self._send(struct.pack("i", cmd_type))
        if len(data) > 0:
            self._send(data)

    def _resp(self, cmd_type):
        d = self.socket.recv(4);
        sz = struct.unpack("i", d);

        if sz[0] < 4:
            return []

        d = self._recv(4);
        t = struct.unpack("i", d);

        d = self._recv(sz[0] - 4)

        if t[0] != cmd_type:
            return []

        return d

    def wait_input(self, n, state):
        """
        Ожидание цифрового входа

        Waiting for digital input

        :param n: input number
        :param state: waiting state True or False
        """
        while self.read_dig_input(n) != state:
            time.sleep(0.05)

    def move_j(self, position, speed, acceleration, blend = 0):
        """
        Функция создает целевую точку перемещения типа joint.
        Данное перемещение является перемещением по осям.
        Использование этого перемещение увеличивает скорость достижение точки по сравнению с другими типами перемещений, робот перемещается в заданную точку по кратчайшему пути.
        Траектория будет заведомо неизвестной кривой, используйте данный вид перемещения в случае если траектория имеет второстепенное значение.

        The function creates a target motion point of joint type.
        This motion is an axis motion.
        Using this motion increases the speed of reaching a point compared to other types of motions, the robot moves to a given point along the shortest path.
        The trajectory will be an unknown curve, use this type of motion if the trajectory is of secondary importance.

        :param position: target position of 6 axes in degree, format list [0, 0, 0, 0, 0, 0], degree
        :param speed: target speed deg/s
        :param acceleration: target acceleration deg/s2
        :param blend: blending radius
        """
        self.waypoints_list.append({'type': MOVE_J, 'data': (position, speed, acceleration, blend)})
        self.add_wp_deg(t=0, des_q=position, vmax_j=speed, amax_j=acceleration, rblend=blend)
       
       
    def move_l(self, position, speed, acceleration, blend = 0.001):
        """
        Функция создает целевую точку перемещения типа line.
        Данное перемещение является перемещением по траектории, линейным.Центральная точка инструмента пермещается с постоянной скоростью.
        При данном типе перемещения возможна сигнулярность при которой невозможна однозначная обратная трансформация
        (пересчет заданных декартовых координат в осевые значеиня манипулятора).


        The function creates a target motion point of type line.
        This motion is a path motion, linear. The center point of the tool moves at a constant speed.
        With this type of motion, a singularity is possible in which an unambiguous reverse transformation is impossible
        (recalculation of the given Cartesian coordinates into the axial values of the manipulator).


        :param position: target position in cartesian space, format list [X, Y, Z, rx, ry, rz] sm, degree
        :param speed: target speed m/s
        :param acceleration: target acceleration m/s2
        :param blend: blending radius
        """
        self.waypoints_list.append({'type': MOVE_L, 'data': (position, speed, acceleration, blend)})
        self.add_wp(t=1, des_x=position, vmax_t=speed, amax_t=acceleration, amax_r=acceleration, vmax_r=speed,  rblend=blend)

    def add_wp_deg(
        self,
        t = 0,
        des_q = [0, 0, 0, 0, 0, 0],
        des_x = [0, 0, 0, 0, 0, 0],
        force = [0, 0, 0, 0, 0, 0],
        force_en = [0, 0, 0, 0, 0, 0],
        in_tcp = 0,
        vmax_t = 0,
        vmax_r = 0,
        amax_t = 0,
        amax_r = 0,
        vmax_j = 0,
        amax_j = 0,
        rblend = 0,
    ):
        """
                This function creates the motion points.

                J motion:
                    add_wp_deg(t=0, des_q=[0, -90, 0, -90, 0, 0], vmax_j=2.0, amax_j=2.0, rblend=0)
                    des_q: set pos in degree for 6 axes
                L motion:
                    add_wp_deg(t=1, des_x=[-39.00, -13.50, 42.33, -179.99, 0, 90.00], vmax_t=1, amax_t=1, amax_r=1, vmax_r=1, rblend=0)
                    des_x:
                    - first 3 value is TCP pos in cartesian space (sm)
                    - value 4-6 rotation around axes of TCP X, Y, Z, (degree)
                :param t:
                :param des_q:
                :param des_x:
                :param force:
                :param force_en:
                :param in_tcp:
                :param vmax_t:
                :param vmax_r:
                :param amax_t:
                :param amax_r:
                :param vmax_j: speed for j motion deg/s
                :param amax_j:
                :param rblend:
                :return:
                """

        des_q_rad = des_q.copy()
        des_x_rad = des_x.copy()

        for i in range(0,6):
            des_q_rad[i] = _to_rad(des_q[i])

        """for i in range(0,3):
            des_x_rad[i] = des_x[i]/100"""

        for i in range(3,6):
            des_x_rad[i] = _to_rad(des_x[i])

        self.add_wp(
            t,
            des_q_rad,
            des_x_rad,
            force,
            force_en,
            in_tcp,
            vmax_t,
            vmax_r,
            amax_t,
            amax_r,
            vmax_j,
            amax_j,
            rblend,
        )

    def add_wp(
        self,
        t = None,
        des_q = None,
        des_x = None,
        force = None,
        force_en = None,
        in_tcp = None,
        vmax_t = None,
        vmax_r = None,
        amax_t = None,
        amax_r = None,
        vmax_j = None,
        amax_j = None,
        rblend = None,
    ):

        if not t:
            t = self.vars.t
        if not des_q:
            des_q = self.vars.des_q
        if not des_x:
            des_x = self.vars.des_x
        if not force:
            force = self.vars.force
        if not force_en:
            force_en = self.vars.force_en
        if not in_tcp:
            in_tcp = self.vars.in_tcp
        if not vmax_t:
            vmax_t = self.vars.vmax_t
        if not vmax_r:
            vmax_r = self.vars.vmax_r
        if not amax_t:
            amax_t = self.vars.amax_t
        if not amax_r:
            amax_r = self.vars.amax_r
        if not vmax_j:
            vmax_j = self.vars.vmax_j
        if not amax_j:
            amax_j = self.vars.amax_j
        if not rblend:
            rblend = self.vars.rblend
            
        self._cmd(CTRLR_COMS_MOVE_ADD_WP,
            struct.pack(
                "i6d6d6d6BB7di0q",
                t,
                *des_q,
                *des_x,
                *force,
                *force_en,
                in_tcp,
                vmax_t,
                vmax_r,
                amax_t,
                amax_r,
                vmax_j,
                amax_j,
                rblend,
                0,
            )
        )
        self._cmd_cntr = (self._cmd_cntr + 1) & 65535

    def is_motion_stopped(self):
        return self.ctrl.data["buff_fill"] == 0

    def check_wp_buffer_and_collisions(self, value: int) -> bool:
        while int(self.ctrl.data["buff_fill"]) > value:
            time.sleep(0.01)
            if (len(self.waypoints_list) - int(self.ctrl.data["buff_fill"])) == 1 and not self.hold_happened():
                logging.info(f"Waypoint {self.waypoints_list[0]['data'][0]} complete.")
                #logging.info(f"Pos {self.get_act_pos_cartesian()} .")
                self.waypoints_list.pop(0)
                logging.info(f'Local WP: {len(self.waypoints_list)}. Core WP: {int(self.ctrl.data["buff_fill"])}')

        if not self.hold_happened():
            return True
        else:
            logging.error('Collision happened! Switched to HOLD mode.')
            return False

    def colab_await_buffer(self, buff_fill: int, func: Callable[[Any], bool] = None, *f_args, **f_kwargs) -> None:
        """
        The function of collaborative waiting for the execution of points
        with the ability to declare user's logic for collisions cases.

        :param buff_fill: Shows at what number of waypoints in the core buff_fill the script execution can be continued
        :param func: Replaces build-in 'input()' boolean-waiting-function with user-defined waiting boolean function.
        :param f_args: Tuple-like arguments for user-defined function.
        :param f_kwargs: Dict-like arguments for user-defined function.
        """
        logging.info(f'Local WP: {len(self.waypoints_list)}. Core WP: {int(self.ctrl.data["buff_fill"])}')
        while int(self.ctrl.data["buff_fill"]) > buff_fill:

            if not self.check_wp_buffer_and_collisions(buff_fill):
                if func is None:
                    logging.info('To continue, press any [key] and than press [Return]. To stop, press single [Return].')
                    should_continue = bool(input())
                else:
                    logging.info(f'The function [{func.__name__}] defined by the user is working.')
                    # p = self.get_act_pos_cartesian()
                    # for i in range(0, 3):
                    #     logging.info(f'Actual position{p}')
                    #     time.sleep(2)

                    should_continue = func(*f_args, **f_kwargs)

                if not should_continue:
                    self._cmd(CTRLR_COMS_STOP)
                    self._stop()
                elif should_continue:
                    self._cmd(CTRLR_COMS_STOP)
                    waypoints_list_dump = copy.deepcopy(self.waypoints_list)
                    self.waypoints_list.clear()

                    for point in waypoints_list_dump:
                        if point['type'] == MOVE_J:
                            self.move_j(*point['data'])
                        elif point['type'] == MOVE_L:
                            self.move_l(*point['data'])
                        logging.info(f"Restored {point['data'][0]} waypoint.")

                    self.run_wps()

            time.sleep(0.01)

    def colab_thread(self, buff_fill: int, func: Callable[[Any], bool] = None, func_colab_status: Callable[[Any], bool] = None, *f_args, **f_kwargs) -> None:
    
        """
        The function of collaborative waiting for the execution of points
        with the ability to declare user's logic for collisions cases.

        :param buff_fill: Shows at what number of waypoints in the core buff_fill the script execution can be continued
        :param func: Replaces build-in 'input()' boolean-waiting-function with user-defined waiting boolean function.
        :param f_args: Tuple-like arguments for user-defined function.
        :param f_kwargs: Dict-like arguments for user-defined function.
        :func_colab_status: Switch off func
        """

        logging.info(f'Local WP: {len(self.waypoints_list)}. Core WP: {int(self.ctrl.data["buff_fill"])}')
        while int(self.ctrl.data["buff_fill"]) > buff_fill:
            if not self.check_wp_buffer_and_collisions(buff_fill):
                thread_flag = func_colab_status()
                if thread_flag == False:
                    should_continue = False
                    logging.info('Function to restore points switch off')
                    self.waypoints_list.clear()
                elif func is None and thread_flag == True:
                    logging.info('To continue, press any [key] and than press [Return]. To stop, press single [Return].')
                    should_continue = bool(input())
                else:
                    logging.info(f'The function [{func.__name__}] defined by the user is working.')
                    should_continue = func(*f_args, **f_kwargs)

                if not should_continue and thread_flag == False:
                    #self._cmd(CTRLR_COMS_STOP)
                    return
                    #self.smooth_max_hold(1)

                elif not should_continue:
                    self._cmd(CTRLR_COMS_STOP)
                    self._stop()
                elif should_continue:
                    self._cmd(CTRLR_COMS_STOP)
                    waypoints_list_dump = copy.deepcopy(self.waypoints_list)
                    self.waypoints_list.clear()

                    for point in waypoints_list_dump:
                        if point['type'] == MOVE_J:
                            self.move_j(*point['data'])
                        elif point['type'] == MOVE_L:
                            self.move_l(*point['data'])
                        logging.info(f"Restored {point['data'][0]} waypoint.")

                    self.run_wps()

            time.sleep(0.01)

    def await_buffer(self, value):
        waypoint_amount = 0
        while int(self.ctrl.data["buff_fill"]) > value:
            if waypoint_amount != int(self.ctrl.data["buff_fill"]):
                waypoint_amount = int(self.ctrl.data["buff_fill"])
                print(f'Waypoint in queue: {waypoint_amount}')
            time.sleep(0.001)
        print('Waypoint queue is empty\n')

    def await_hold(self):
        while int(self.ctrl.data["motion_mode"]) != 0:
            time.sleep(0.001)
            if not self.ctrl.run:
                self._stop()

    def await_physical_stop(self, threshold):
        while True:
            qdm = 0
            for qd in self.ctrl.data["act_qd"]:
                qdm = qdm + qd**2
            if qdm**0.5 < threshold:
                break
            time.sleep(0.001)
            if not self.ctrl.run:
                self._stop()

    def await_accepted(self):
        while int(self.ctrl.data["cmd_cntr"]) != self._cmd_cntr:
            time.sleep(0.001)
            if not self.ctrl.run:
                self._stop()

    def await_motion(self):
        self.await_accepted()
        self.await_buffer(0)
    
    def hold(self):
        self._cmd(CTRLR_COMS_STOP)

    def smooth_max_hold(self, brake_deceleration: float) -> None:
        """
        Deceleration from maximum available speed (3.14 rad)
        :param brake_deceleration: 0.5 - 20 rad/s (All values above 20 rad/s cause instant stop)
        """

        self.brake_deceleration = brake_deceleration
        self._cmd(CTRLR_COMS_STOP_SMOOTH, struct.pack("d", self.brake_deceleration))

    def get_total_run_time(self):
        """
        Get total robot run_time in 'hh:mm:ss' format.
        """
        self._cmd(CTRLR_COMS_GET_RUN_TIME)
        response = self._resp(CTRLR_COMS_GET_RUN_TIME)
        if len(response) == 0:
            return None
        else:
            result = int(struct.unpack("Q", response)[0])
            return time.strftime('%H:%M:%S', time.gmtime(result))
        
    def hold_happened(self) -> bool:
        return self.ctrl.data["motion_mode"] == 0

    def zg(self, en):
        self._cmd(CTRLR_COMS_ZG, struct.pack("B", en))

    def _stop(self):
        print("Exiting...")
        sys.exit()

    def set_speed_scaling(self, scale):
        """
        Set max speed in %
        range: 0.00 to 1.00
        """
        self.scale_v = scale
        self._cmd(CTRLR_COMS_MOVE_SCALE, struct.pack("dd", self.scale_v, self.scale_a))

    def set_accel_scaling(self, scale):
        """
        Set max accel in %
        range: 0.00 to 1.00
        """
        self.scale_a = scale
        self._cmd(CTRLR_COMS_MOVE_SCALE, struct.pack("dd", self.scale_v, self.scale_a))

    def run_wps(self):
        self.await_accepted()
        self._cmd(CTRLR_COMS_MOVE_RUN)

    def run(self):
        for i in range(0, 10):
            time.sleep(0.1)
            try:
                if self.ctrl.data["state"] <= 1:

                    self._cmd(CTRLR_COMS_POWER, struct.pack("i", CTRLR_COMS_POWER_CMD_OFF))
                break
            except:
                pass

        self._cmd(CTRLR_COMS_POWER, struct.pack("i", CTRLR_COMS_POWER_CMD_RUN))

        for i in range(0, 200):
            time.sleep(0.1)
            if "state" in self.ctrl.data:
                if self.ctrl.data["state"] == 4:
                    self._cmd_cntr = int(self.ctrl.data["cmd_cntr"])
                    self.logger.debug('Robot is running')
                    return True

        return False


    def off(self):
        """
        Switch off power from manipulator
        Servo hold mode is deactivated
        """
        self._cmd(CTRLR_COMS_POWER, struct.pack("i", CTRLR_COMS_POWER_CMD_OFF))

    def stby(self):
        for i in range(0, 10):
            time.sleep(0.1)
            try:
                if self.ctrl.data["state"] <= 1:
                    self._cmd(CTRLR_COMS_POWER, struct.pack("i", CTRLR_COMS_POWER_CMD_OFF))
                break
            except:
                pass

        self._cmd(CTRLR_COMS_POWER, struct.pack("i", CTRLR_COMS_POWER_CMD_STBY))

        for i in range(0, 200):
            time.sleep(0.1)
            if "state" in self.ctrl.data:
                if self.ctrl.data["state"] == 2:
                    self._cmd_cntr  = int(self.ctrl.data["cmd_cntr"])
                    self.logger.debug('Robot is STBY')
                    return True

        return False

    def write_dig_output(self, n, v):
        dm = [0, 0, 0, 0, 0, 0, 0, 0]
        dv = [0, 0, 0, 0, 0, 0, 0, 0]
        am = [0, 0, 0, 0]
        ac = [0, 0, 0, 0]
        av = [0, 0, 0, 0]

        if n >= self.ctrl.data["dig_out_count"] | n < 0:
            self.logger.debug('Wrong digital output number')
            return

        dm[int(n / 8)] = 1 << (n % 8)
        dv[int(n / 8)] = (1 if v else 0) << (n % 8)

        c = struct.pack("24B4d", *dm, *dv, *am, *ac, *av)
        self._cmd(CTRLR_COMS_SET_OUTPUTS, c)

    def write_an_output(self, n, v, cm):
        dm = [0, 0, 0, 0, 0, 0, 0, 0]
        dv = [0, 0, 0, 0, 0, 0, 0, 0]
        am = [0, 0, 0, 0]
        ac = [0, 0, 0, 0]
        av = [0, 0, 0, 0]

        if n >= self.ctrl.data["an_out_count"] | n < 0:
            self.logger.debug('Wrong analog output number')
            return

        am[n] = 1
        av[n] = v * 1e-3 if cm else v
        ac[n] = 1 if cm else 0

        c = struct.pack("24B4d", *dm, *dv, *am, *ac, *av)
        self._cmd(CTRLR_COMS_SET_OUTPUTS, c)

    def write_an_output_volt(self, n, v):
        self.write_an_output(n, v, False)

    def write_an_output_curr(self, n, v):
        self.write_an_output(n, v, True)

    def read_dig_input(self, n):
        if n >= self.ctrl.data["dig_in_count"] | n < 0:
            self.logger.debug('Wrong digital input number')
            return
        byte = int(n / 8);
        mask = 1 << (n % 8);
        return 1 if self.ctrl.data["dig_in"][byte] & mask != 0 else 0

    def read_an_input(self, n):
        if n >= self.ctrl.data["an_in_count"] | n < 0:
            self.logger.debug('Wrong analog input number')
            return
        v = self.ctrl.data["an_in_value"][n]
        c = self.ctrl.data["an_in_curr_mode"][n]
        return v * 1e3 if c else v

    def init_robot(self):
        """
        Use this command at the beginning of the programme.
        This function start self._connect(),self._init_control(), self.run().
        It is possible to use it separately in case it is needed.
        1. connect user host socket to controller socket - _connect()
        2. Start getting data from controller - _init_control() (you are able to get data by self.ctrl.data[])
        3. start power and data connection between controller and manipulator - run()

        """
        if not self._connect():
            self._stop()
        print('connected')
        if not self._init_control():
            self._stop()
        print('init complete')
        if not self.run():
            self._stop()
        print('run')

    def set_gravity(self, v):
        self._cmd(CTRLR_COMS_SET_GRAVITY, struct.pack("3d", *v))

    def set_zg_fscale(self, fs):
        self._cmd(CTRLR_COMS_SET_ZG_FSCALE, struct.pack("6d", *fs))

    def set_trq_win(self, w):
        self._cmd(CTRLR_COMS_SET_TRQ_WIN, struct.pack("6d", *w))
        
    def get_trq_win(self):
        """
        Get torque window.
        """
        self._cmd(CTRLR_COMS_GET_TRQ_WIN)
        response = self._resp(CTRLR_COMS_GET_TRQ_WIN)
        if len(response) == 0:
            return None
        else:
            return list(struct.unpack("6d", response))

    def set_flw_err(self, j, ct, cr):
        self._cmd(CTRLR_COMS_SET_FLW_ERR, struct.pack("3d", j, ct, cr))

    def set_max_vel(self, j, ct):
        self._cmd(CTRLR_COMS_SET_MAX_VEL, struct.pack("7d", *j, ct))

    def set_payload(self, m, com):
        self._cmd(CTRLR_COMS_SET_PAYLOAD, struct.pack("4d", m, *com))

    def set_tool(self, t):
        self._cmd(CTRLR_COMS_SET_TOOL, struct.pack("6d", *t))

    def set_jog_param(self, in_tcp, force_en, force, spd_max, accel, decel):
        self._cmd(CTRLR_COMS_SET_JOG_PARAM, struct.pack("B6B6d6d6d6d", in_tcp,
                    *force_en, *force, *spd_max, *accel, *decel))

    def set_force_param(self, vi, damp, vmax):
        self._cmd(CTRLR_COMS_SET_FORCE_PARAM, struct.pack("18d", *damp, *vi, *vmax))

    def set_io_func(self, hold_in, zg_in):

        if hold_in[1]:
            hold_in[0] |= 0x80000000
        if zg_in[1]:
            zg_in[0] |= 0x80000000

        self._cmd(CTRLR_COMS_SET_IO_FUNC, struct.pack("2I", hold_in[0], zg_in[0]))

    def set_dh_model(self, dh):
        self._cmd(CTRLR_COMS_SET_DH_MODEL, struct.pack("6d6d6d6d6d", *dh["alpha"], *dh["A"],
                *dh["D"], *dh["theta"], *dh["offset"]))

    def get_dh_model(self):
        self._cmd(CTRLR_COMS_GET_DH_MODEL)
        d = self._resp(CTRLR_COMS_GET_DH_MODEL)

        if len(d) == 0:
            return []

        res = list(struct.unpack("6d6d6d6d6d", d))

        return {"alpha" : res[0:6], "A" : res[6:12],
                "D" : res[12:18],  "theta" : res[18:24],
                "offset" : res[24:30]}

    def fkine(self, q):
        self.socket.settimeout(1)
        self._cmd(CTRLR_COMS_FKINE, struct.pack("6d", *q))
        d = self._resp(CTRLR_COMS_FKINE)

        if len(d) == 0:
            return []

        res = list(struct.unpack("i6d", d))

        if res[0] == 0:
            return res[1:7]
        else:
            return []

    def ikine(self, x):
        self.socket.settimeout(1)
        self._cmd(CTRLR_COMS_IKINE, struct.pack("6d", *x))
        d = self._resp(CTRLR_COMS_IKINE)

        if len(d) == 0:
            return []

        res = list(struct.unpack("i48d", d))
        if res[0] == 0:
            return [res[i*6+1:i*6+7] for i in range(8)]
        else:
            return []

    def set_tool_capsule(self, n, c):
        self._cmd(CTRLR_COMS_SET_TOOL_CAPSULE, struct.pack("2i8d",
            0,  n, *c["start"], c["length"], *c["rot"], c["R"]))

    def set_tool_capsule_count(self, ncap):
        self._cmd(CTRLR_COMS_SET_TOOL_CAPSULE_COUNT, struct.pack("2i", 0, ncap))

    def get_act_pos_q(rr):
        """
        get radian axes values and convert them to degrees
        return list of 6 axes degrees
        """
        act_q = list(rr.ctrl.data["act_q"])
        #print("actq1: ", act_q)
        for i in range(6):
            act_q[i] = math.degrees(act_q[i])

        #print("actq2: ", act_q)
        return  act_q

    def get_act_pos_cartesian(rr):
        """
        convert cartesian pos to cm and degree
        """
        """act_x = list(rr.ctrl.data["act_x"])
        for i in range(0, 3):
            act_x[i] = act_x[i] * 100
        for i in range(3, 6):
            act_x[i] = math.degrees(act_x[i])"""

        return list(rr.ctrl.data["act_x"])




