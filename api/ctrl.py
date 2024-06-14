#!/usr/bin/env python

import socket
import struct
import logging
import logging
from threading import Thread
import threading
import sys

logging.basicConfig(level=logging.DEBUG)
PORT_INPUT = 29000 


# 'Cycle time [s]': cycle_time
# 'cycle_duty': cycle_duty
# 'CB state': state
# "Joint's servomotors mode": servo_mode
# 'motion_mode': motion_mode
# 'jcond': jcond 50 max, value more 50 close to singularity
# 'buff_sz': buff_sz
# 'buff_fill': buff_fill
# 'cmd_cntr':  cmd_cntr
# 'res': wpi.res
# 'Target position [deg] j{index}': move_des_q
# 'Target velocity [deg/s] j{index}': move_des_qd
# 'Target TCP pose [m] {["x", "y", "z"][index]}': move_des_x[:3]
# 'Target TCP pose [deg] {["rx", "ry", "rz"][index]}': move_des_x[3:]
# 'Target TCP velocity [m/s] {["x", "y", "z"][index]}': move_des_xd[:3]
# 'Target TCP velocity [deg/s] {["rx", "ry", "rz"][index]}':   move_des_xd[3:]
# 'Actual position [deg] j{index}': act_q
# 'Actual velocity [deg/s] j{index}': act_qd
# 'Actual TCP pose [m] {["x", "y", "z"][index]}': act_x[:3]
# 'Actual TCP pose [deg] {["rx", "ry", "rz"][index]}': act_x[3:]
# 'Actual TCP velocity [m/s] {["x", "y", "z"][index]}': act_xd[:3]
# 'Actual TCP velocity [deg/s] {["rx", "ry", "rz"][index]}': act_xd[3:]
# 'Actual joints torque [Nm] j{index}': act_tq
# 'frict_tq [Nm] {index}': frict_tq
# 'ne_tq [Nm] {index}': ne_tq
# 'TCP force [N] {["x", "y", "z"][index]}': act_force_e[:3]
# 'TCP force [Nm] {["rx", "ry", "rz"][index]}': act_force_e[3:]
# 'act_force_0 [N] {["x", "y", "z"][index]}': act_force_0[:3]
# 'act_force_0 [Nm] {["rx", "ry", "rz"][index]}': act_force_0[3:]
# 'Target joints torque [Nm] j{index}': des_trq
# 'des_qd {index}': des_qd
# 'Motor stator temperature [C] j{index}': temp_m
# 'Motor electronics temperature [C] j{index}': temp_e
# 'Arm current [A]': arm_current
# 'Arm voltage [V]': arm_voltage
# 'Main voltage [V]': psu_voltage
# 'dig_in_count': dig_in_count
# 'an_in_count': an_in_count
# f'dig_in {index}': dig_in
# 'an_in_curr_mode {index}': an_in_curr_mode
# 'an_in_value {index}': an_in_value
# 'dig_out_count': dig_out_count
# 'an_out_count': an_out_count
# 'dig_out {index}': dig_out
# 'an_out_curr_mode {index}': an_out_curr_mode
# 'an_out_value {index}': an_out_value
# 'Control mode j{index}': jointInfo
# 'Voltage [V] j{index}': jointInfo
# 'Actual current [А] j{index}': jointInfo



PARAMETERS_LIST = (
    ("cycle_time", 1),
    ("cycle_duty", 1),
    ("state", 1),
    ("servo_mode", 1),
    ("motion_mode", 1),
    ("jcond", 1),
    ("buff_sz", 1),
    ("buff_fill", 1),
    ("cmd_cntr", 1),
    ("res0", 1),
    ("move_des_q", 6),
    ("move_des_qd", 6),
    ("move_des_x", 6),
    ("move_des_xd", 6),
    ("act_q", 6),
    ("act_qd", 6),
    ("act_x", 6),
    ("act_xd", 6),
    ("act_tq", 6),
    ("frict_t", 6),
    ("act_ext_tq", 6),
    ("act_force_e", 6),
    ("act_force_0", 6),
    ("des_trq", 6),
    ("des_qd", 6),
    ("temp_m", 6),
    ("temp_e", 6),
    ("arm_current", 1),
    ("arm_voltage", 1),
    ("psu_voltage", 1),
    ("dig_in_count", 1),
    ("an_in_count", 1),
    ("dig_in", 8),
    ("an_in_curr_mode", 4),
    ("an_in_value", 4),
    ("dig_out_count", 1),
    ("an_out_count", 1),
    ("dig_out", 8),
    ("an_out_curr_mode", 4),
    ("an_out_value", 4)
)

STRUCT_FORMAT = "6d4H105d14b4d14b4d"


class Control:
    def __init__(self, ip, port=PORT_INPUT):
        self.data = {}
        self.ip = ip
        self.port = port
        self.is_run = False
        self.sd = None
        self._struct_size = struct.calcsize(STRUCT_FORMAT)
        self.logger = logging.getLogger(__name__)
        self.run = True

    def _connect(self):
        try:
            self.sd = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sd.connect((self.ip, self.port))
            self.sd.settimeout(1)
            logging.debug(f"Socket connect [{self.ip}:{self.port}] --> Ok")
        except Exception as error:
            logging.debug(f"Socket connect [{self.ip}:{self.port}] --> False\n{error}")
            self.run = False
            sys.exit()

    def _recv(self, l):
        d = self.sd.recv(l)
        if d == b'':
            print("RTD connection lost")
            self.run = False
            sys.exit()
        
        return d

    def _recive_data(self):
        try:
            raw_data = self._recv(self._struct_size)
                
            while len(raw_data) < self._struct_size:             
                chunk = self._recv(self._struct_size - len(raw_data))
                raw_data += chunk
                        
            unpack_data = struct.unpack(STRUCT_FORMAT, raw_data)
            
            n = 0
            for i, p in enumerate(PARAMETERS_LIST):
                if p[1] > 1:
                    self.data[p[0]] = unpack_data[n:n+p[1]]
                else:
                    self.data[p[0]] = unpack_data[n]
                n += p[1]
            #print(unpack_data)
        except Exception as error:
            self.logger.error(error)
            self.run = False
            sys.exit()
    def _stop(self):
        print("Exiting...")
        sys.exit()

    def _send(self, d):
        try:
            s = self.sd.send(d)
            if s == 0:
                print("CTRL connection lost")
                self._stop()
        except Exception as error:
            print("CTRL _send err")
            self._stop()
            return False

    def _cmd(self, cmd_type, data = []):
        self._send(struct.pack("i", len(data) + 4))
        self._send(struct.pack("i", cmd_type))
        if len(data) > 0:
            self._send(data)

    def _thread(self, main_thread):
        self.logger.debug("Recive data thread started")
        while self.run:
            if not main_thread.is_alive():
                self.sd.close()
                break
            self._recive_data()
        self.logger.debug("Recive data thread stopped")

    def start_thread(self):
        self._connect()
        if not self.sd:
            return False
        thread = threading.Thread(target=self._thread, args=(threading.current_thread(),))
        thread.start()
        return True


if __name__ == '__main__':
    ctrl = Control('192.168.0.50')
    print(ctrl.start_thread())
    input()

