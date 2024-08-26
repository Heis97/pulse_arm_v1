from dataclasses import dataclass
from source.models.constants import (CTRLR_MAX_DIG_IN_BYTES,
                                         CTRLR_MAX_DIG_OUT_BYTES,
                                         CTRLR_MAX_AN_IN,
                                         CTRLR_MAX_AN_OUT,
                                         JOINTS_COUNT)


STRUCT_FORMAT = 'B 7d 4H 54d dB 48d 5d 14B4d 14B4d 6B 6I 24d QQ 7x B'

@dataclass(order=True)
class RTD:
    # Почему в STRUCT_FORMAT вначале и вконце B, тогда как в packet_begin 
    # и в packet_end аннотируется как float, неверная аннотация?
    packet_begin: float = 1  # package beginning check-byte (B)

    cycle_time: float = 1  # actual cycle time (d)
    cycle_duty: float = 1  # time in % occupied by calculations (d)
    state: float = 1  # state of robot controller (d)
    safety: float = 1  # safety state of robot controller (d)
    servo_mode: float = 1  # mode of joint's servomotors (d)
    motion_mode: float = 1  # actual motion mode (d)
    jcond: float = 1  # Jacobi condition (d)

    buff_sz: float = 1  # waypoint ring buffer size (H)
    buff_fill: float = 1  # counter of accepted, but not completed yet waypoints (H)
    wp_cntr: float = 1  # counter of all accepted waypoints (H)
    res: float = 1  # reserved (H)

    move_des_q: tuple = JOINTS_COUNT  # traj. gen. desired position for joint space (6d)
    move_des_qd: tuple = JOINTS_COUNT  # traj. gen. desired velocity for joint space (6d)
    move_des_x: tuple = JOINTS_COUNT  # traj. gen. desired position for cart. space(6d)
    move_des_xd: tuple = JOINTS_COUNT  # traj. gen. desired velocity for cart. space (6d)
    act_q: tuple = JOINTS_COUNT  # actual positions of joints (6d)
    act_qd: tuple = JOINTS_COUNT  # actual vel. of joints (6d)
    act_x: tuple = JOINTS_COUNT  # actual cart. position XYZ, RPY (6d)
    act_xd: tuple = JOINTS_COUNT  # act cart. vel. XYZ, RPY (6d)
    act_tq: tuple = JOINTS_COUNT  # actual joints torque (6d)

    act_vel_scale: float = 1  # actual velocity scale (from 0.0 to 1.0) (d)
    act_vel_limit_reason: float = 1  # reason of velocity limiting by applying scale (0 - velocity is not limited) (B)

    frict_tq: tuple = JOINTS_COUNT  # predicted friction torque (6d)
    ne_tq: tuple = JOINTS_COUNT  # predicted gravity/coriolis (newton-euler algorithm) torque (6d)
    act_force_e: tuple = JOINTS_COUNT  # actual force(XYZ)/torque(RPY) applied to TCP in TCP coordinate frame (6d)
    act_force_0: tuple = JOINTS_COUNT  # the same in base coordinate frame (6d)
    des_tq: tuple = JOINTS_COUNT  # current command for servomotors (6d)
    des_qd: tuple = JOINTS_COUNT  # velocity command for servomotors (6d)
    des_amp: tuple = JOINTS_COUNT  # desired current (6d)
    des_iwin: tuple = JOINTS_COUNT  # possible difference between des_iq and act_iq (6d)

    arm_current: float = 1  # (d)
    arm_voltage: float = 1  # (d)
    io_current: float = 1  # maximal of currents, provided to IO (d)
    psu_voltage: float = 1  # power supply unit voltage (Volts) (d)
    cbox_temp: float = 1  # maximal of temperatures inside of robot controller (d)

    dig_in_count: float = 1  # number of bits (B)
    an_in_count: float = 1  # (B)
    dig_in: tuple = CTRLR_MAX_DIG_IN_BYTES  # (8B)
    an_in_curr_mode: tuple = CTRLR_MAX_AN_IN  # (4B)
    an_in_value: tuple = CTRLR_MAX_AN_IN  # (4d)

    dig_out_count: float = 1  # number of bits (B)
    an_out_count: float = 1  # (B)
    dig_out: tuple = CTRLR_MAX_DIG_OUT_BYTES  # (8B)
    an_out_curr_mode: tuple = CTRLR_MAX_AN_OUT  # (4B)
    an_out_value: tuple = CTRLR_MAX_AN_OUT  # (4d)

    joint_state: tuple = JOINTS_COUNT  # 0 - joint brakes applied, 1 - joint brakes released (6B)
    joint_err_state: tuple = JOINTS_COUNT  # error state of motors (6L)
    joint_volt: tuple = JOINTS_COUNT  # actual voltage of motors (Volts) (6d)
    joint_amp: tuple = JOINTS_COUNT  # actual currents of motors (Amperes) (6d)
    joint_temp_motor: tuple = JOINTS_COUNT  # actual temperatures of motors (deg. Celsius) (6d)
    joint_temp_board: tuple = JOINTS_COUNT  # actual temperatures of motor electronics (deg. Celsius) (6d)

    sec: float = 1  # timestamp's seconds (Q)
    nsec: float = 1  # timestamp's nanoseconds (Q)

    # 7 padding bytes here

    packet_end: float = 1  # package ending check-byte (B)


@dataclass(frozen=True)
class RTDataPackageBorders:
    rtd_packet_begin: int = 0x12  # package beginning check-byte
    rtd_packet_end: int = 0x21  # package ending check-byte
