from __future__ import annotations

import logging
import struct
from dataclasses import dataclass, fields
from typing import Any

from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    RTDParsingError,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    CTRLR_MAX_AN_IN,
    CTRLR_MAX_AN_OUT,
    CTRLR_MAX_DIG_IN_BYTES,
    CTRLR_MAX_DIG_OUT_BYTES,
    JOINTS_COUNT,
    PADDING_RES1_LENGTH,
    PADDING_RES2_LENGTH,
    REAL_WRIST_MAX_AN_IN,
    TCP_POSITION_COUNT,
    WRIST_ACCEL,
    WRIST_GYRO,
)

STRUCT_FORMAT = "B7B2f 7d 4H 60d d2B 2Bf 10d 48d 5d 16B4d 16B4d 8B2f f I 6d 2f 6B 2B 6I 24d QQ f 4B"


@dataclass(frozen=True)
class RTDataPackageBorders:
    rtd_packet_begin: int = 0x12  # package beginning check-byte
    rtd_packet_begin_structure: str = "B"
    rtd_packet_end: int = 0x21  # package ending check-byte
    rtd_packet_end_structure: str = "B"


@dataclass(order=True)
class RTD:
    packet_begin: int = 1
    # package beginning check-byte (B)
    byte_res1: tuple[int, ...] = (0,) * PADDING_RES1_LENGTH
    # padding for packet start, reserved values can be used for debug (7B)
    act_point: float = 1
    # actual point (progress of current movement). From 0.0 to `desired_point`. m or rad (f)
    desired_point: float = 1
    # desired point (distance to be traveled to the next waypoint, m or rad (f)

    cycle_time: float = 1
    # actual cycle time (d)
    cycle_duty: float = 1
    # time in % occupied by calculations (d)
    state: float = 1
    # state of robot controller (d)
    safety: float = 1
    # safety state of robot controller (d)
    servo_mode: float = 1
    # mode of joint's servomotors (d)
    motion_mode: float = 1
    # actual motion mode (d)
    jcond: float = 1
    # Jacobi condition (d)

    buff_sz: int = 1
    # waypoint ring buffer size (H)
    buff_fill: int = 1
    # counter of accepted, but not completed yet waypoints (H)
    wp_cntr: int = 1
    # counter of all accepted waypoints (H)
    res: int = 1
    # reserved (H)

    move_des_q: tuple[float, ...] = (0,) * JOINTS_COUNT
    # traj. gen. desired position for joint space (6d)
    move_des_qd: tuple[float, ...] = (0,) * JOINTS_COUNT
    # traj. gen. desired velocity for joint space (6d)
    move_des_tcp_x: tuple[float, ...] = (0,) * JOINTS_COUNT
    # traj. gen. desired position for cart. space (6d)
    move_des_tcp_xd: tuple[float, ...] = (0,) * JOINTS_COUNT
    # traj. gen. desired velocity for cart. space (6d)
    act_q: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual positions of joints (6d)
    act_qd: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual vel. of joints (6d)
    act_tcp_x: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual cart. position of TCP XYZ, RPY (6d)
    act_flange_x: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual cart. position of flange (zero tool offset) XYZ, RPY (6d)
    act_tcp_xd: tuple[float, ...] = (0,) * JOINTS_COUNT
    # act cart. vel. XYZ, RPY (6d)
    act_trq: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual joints torque (6d)

    act_vel_scale: float = 1
    # actual velocity scale (from 0.0 to 1.0) (d)
    act_vel_limit_reason: int = 1
    # reason of velocity limiting by applying scale (0 - velocity is not limited) (B)
    state_flags: int = 1
    # additional flags (look `ctrlr_warning_flags_t`) (B)

    byte_res2: tuple[int, ...] = (0,) * PADDING_RES2_LENGTH
    # padding to 8 (2B)
    float_res2: float = 1
    # padding to 8 (f)

    payload_mass: float = 1
    # payload mass (kg) (d)
    payload_com: tuple[float, ...] = (0,) * TCP_POSITION_COUNT
    # position of center of mass (related to end-effector), (XYZ) (3d)
    tool_offset: tuple[float, ...] = (0,) * JOINTS_COUNT
    # position and orientation of tool (related to end-effector), # (XYZ, RPY) (6d)

    frict_trq: tuple[float, ...] = (0,) * JOINTS_COUNT
    # predicted friction torque (6d)

    ne_trq: tuple[float, ...] = (0,) * JOINTS_COUNT
    # predicted gravity/coriolis (newton-euler algorithm) torque (6d)
    act_force_e: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual force(XYZ)/torque(RPY) applied to TCP in TCP coordinate frame (6d)
    act_force_0: tuple[float, ...] = (0,) * JOINTS_COUNT
    # the same in base coordinate frame (6d)

    des_trq: tuple[float, ...] = (0,) * JOINTS_COUNT
    # current command for servomotors (6d)

    des_qd: tuple[float, ...] = (0,) * JOINTS_COUNT
    # velocity command for servomotors (6d)

    des_amp: tuple[float, ...] = (0,) * JOINTS_COUNT
    # desired current (6d)
    des_iwin: tuple[float, ...] = (0,) * JOINTS_COUNT
    # possible difference between des_iq and act_iq (6d)

    arm_current: float = 1  # (d)
    arm_voltage: float = 1  # (d)
    io_current: float = 1
    # maximal of currents, provided to IO (d)
    psu_voltage: float = 1
    # power supply unit voltage (Volts) (d)
    cbox_temp: float = 1
    # maximal of temperatures inside of robot controller (d)

    dig_in_count: int = 1
    # number of bits (B)
    an_in_count: int = 1
    # number of `double` values (analog inputs)(B)
    dig_in: tuple[int, ...] = (0,) * (CTRLR_MAX_DIG_IN_BYTES)
    # digital input values, packed in bytes (8B)

    an_in_curr_mode: tuple[int, ...] = (0,) * CTRLR_MAX_AN_IN
    # modes for analog inputs(4B)
    res_bytes: tuple[int, ...] = (0,) * 2
    # padding (2B)
    an_in_value: tuple[float, ...] = (0,) * (CTRLR_MAX_AN_IN)
    # values of voltage/current on analog inputs (4d)

    dig_out_count: int = 1
    # number of bits (digital outputs) (B)
    an_out_count: int = 1
    # number of `double` values (analog outputs) (B)
    dig_out: tuple[int, ...] = (0,) * (CTRLR_MAX_DIG_OUT_BYTES)
    # digital output values, packed in bytes (8B)

    an_out_curr_mode: tuple[int, ...] = (0,) * CTRLR_MAX_AN_OUT
    # modes for analog outputs (4B)
    res_bytes2: tuple[int, ...] = (0,) * 2
    # padding (2B)
    an_out_value: tuple[float, ...] = (0,) * (CTRLR_MAX_AN_OUT)
    # values of voltage/current on analog outputs (4d)

    # WIRST
    wrist_dig_in_count: int = 1
    # number of bits (B)
    wrist_dig_in: int = 1
    # digital input values, packed in single byte. First two inputs are buttons (B)
    wrist_dig_out_count: int = 1
    # number of bits (digital outputs) (B)
    wrist_dig_out: int = 1
    # digital output values, packed in single byte (B)
    wrist_an_in_count: int = 1
    # number of `double` values (analog inputs)(B)
    wrist_an_in_curr_mode: tuple[int, ...] = (0,) * (REAL_WRIST_MAX_AN_IN)
    # modes for analog inputs (2B)
    wrist_mode: int = 1
    # mode of board - see `tool_mode_t` (B)
    wrist_an_in_value: tuple[float, ...] = (0,) * REAL_WRIST_MAX_AN_IN
    # (2f)

    wrist_temperature: float = 1
    # maximum of temperatures on circuit (CPU, MEMS) (f)

    wrist_safety_flags: float = 1
    # safety flags (L)

    wrist_accel: tuple[float, ...] = (0,) * WRIST_ACCEL
    # accelerometer values (XYZ)(3d)
    wrist_gyro: tuple[float, ...] = (0,) * WRIST_GYRO
    # gyroscope values (XYZ)(3d)

    tool_dig_out_current: float = 1
    # real current of do pins on tool connector (f)

    tool_power_current: float = 1
    # real current of power on tool connector (f)

    joint_state: tuple[int, ...] = (0,) * JOINTS_COUNT
    # 0 - joint brakes applied, 1 - joint brakes released (6B)
    res_bytes3: tuple[int, ...] = (0,) * 2
    # padding (2B)
    joint_err_state: tuple[float, ...] = (0,) * JOINTS_COUNT
    # error state of motors (6L)
    joint_volt: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual voltage of motors (Volts) (6d)
    joint_amp: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual currents of motors (Amperes) (6d)
    joint_temp_motor: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual temperatures of motors (deg. Celsius) (6d)
    joint_temp_board: tuple[float, ...] = (0,) * JOINTS_COUNT
    # actual temperatures of motor electronics (deg. Celsius) (6d)

    sec: int = 1
    # timestamp's seconds (Q)
    nsec: int = 1
    # timestamp's nanoseconds (Q)

    dist_to_blend: float = 1
    # distance to blend left until blend between waypoints will start (f)

    realtime_queue_avail: int = 1
    # guaranteed count of realtime motion commands that can be accepted right now (B)

    realtime_waypoints_received: int = 1
    # total amount of received realtime targets, overflows from 255 to 0 (B)

    byte_res3: int = 1
    # padding for packet end, can be used for debug values (B)
    packet_end: int = 1
    # package ending check-byte (B)

    @classmethod
    def get_struct_size(cls) -> int:
        """Получить размер структуры в байтах."""
        return struct.calcsize(STRUCT_FORMAT)

    @classmethod
    def from_bytes(cls, data: bytes | bytearray) -> RTD:
        """
        Создать объект RTD из бинарных данных.

        Args:
            data: байты размером с структуру (STRUCT_SIZE)

        Returns:
            RTD: распакованный объект

        Raises:
            RTDParsingError: если размер данных не совпадает с ожидаемым
        """
        if len(data) != cls.get_struct_size():
            raise RTDParsingError(
                f"Invalid data size: expected {cls.get_struct_size()}, "
                f"got {len(data)}"
            )

        unpacked = struct.unpack(STRUCT_FORMAT, data)
        if not cls._is_valid_rtd_format(unpacked):
            raise RTDParsingError("Invalid received RTD data format")
        return cls._from_unpacked(unpacked)

    @classmethod
    def _from_unpacked(cls, unpacked_data: tuple[Any, ...]) -> RTD:
        """
        Создать объект RTD из кортежа распакованных struct данных.
        """
        n = 0
        instance = cls()
        for field in fields(cls):
            default_value = getattr(cls, field.name)
            if isinstance(default_value, tuple):
                amount = len(default_value)
            else:
                amount = 1

            if amount == 1:
                setattr(instance, field.name, unpacked_data[n])
            else:
                setattr(instance, field.name, unpacked_data[n : n + amount])
            n += amount
        return instance

    def is_valid_content(self, logger: logging.Logger | None = None) -> bool:  # noqa: C901
        """
        Проверка физического смысла данных.
        """

        def log_error(msg: str):
            if logger:
                logger.warning(msg)

        # Напряжения
        if not (-1 <= self.arm_voltage <= 100):
            log_error(f"Invalid arm_voltage: {self.arm_voltage}")
            return False
        if not (-1 <= self.psu_voltage <= 100):
            log_error(f"Invalid psu_voltage: {self.psu_voltage}")
            return False

        # Токи
        if not (-5 <= self.arm_current <= 50):
            log_error(f"Invalid arm_current: {self.arm_current}")
            return False
        if not (-5 <= self.io_current <= 50):
            log_error(f"Invalid io_current: {self.io_current}")
            return False

        # Температура
        if not (-1 <= self.cbox_temp <= 100):
            log_error(f"Invalid cbox_temp: {self.cbox_temp}")
            return False

        # Время цикла
        if not (0.000001 <= self.cycle_time <= 1.0):
            log_error(f"Invalid cycle_time: {self.cycle_time}")
            return False

        # Позиции суставов
        for q in self.act_q:
            if abs(q) > 100:  # rad
                log_error(f"Invalid joint position: {q}")
                return False

        return True

    @staticmethod
    def _is_valid_rtd_format(unpacked_data: tuple[Any, ...]) -> bool:
        """
        Проверить формат структуры RTD данных.
        """
        if (
            unpacked_data[0] == RTDataPackageBorders.rtd_packet_begin
            and unpacked_data[-1] == RTDataPackageBorders.rtd_packet_end
        ):
            return True
        return False

    def __str__(self) -> str:
        """
        Автоматически форматирует все поля датакласса.
        """
        # Паттерны имен полей, которые стоит скрыть из логов
        skip_patterns = (
            "packet_begin",
            "packet_end",
            "res",
            "byte_",
        )

        output = []
        for field in fields(self):
            if any(pattern in field.name.lower() for pattern in skip_patterns):
                continue

            value = getattr(self, field.name)

            if isinstance(value, tuple):
                if value and isinstance(value[0], float):
                    formatted = (
                        "(" + ", ".join(f"{v:.3f}" for v in value) + ")"
                    )
                else:
                    formatted = str(value)
            elif isinstance(value, float):
                formatted = f"{value:.3f}"
            elif isinstance(value, int):
                formatted = str(value)
            else:
                formatted = str(value)

            output.append(f"{field.name}: {formatted}")

        return "RTD(" + ", ".join(output) + ")"
