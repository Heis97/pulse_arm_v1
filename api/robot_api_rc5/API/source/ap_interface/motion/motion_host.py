from __future__ import annotations
from struct import pack
from typing import TYPE_CHECKING

from API.source.ap_interface.motion.joint_motion import JointMotion
from API.source.ap_interface.motion.kinematics_solution import Kinematics
from API.source.ap_interface.motion.linear_motion import LinearMotion
from API.source.ap_interface.motion.motion_mode import MotionMode
from API.source.ap_interface.motion.move_scaling import MoveScaling
from API.source.core.exceptions.data_validation_error.argument_error import (
    validation
)
from API.source.core.exceptions.data_validation_error.generic_error import (
    AddWaypointError
)
from API.source.features.gui.ui_controller import SimpleJoystickUI
from API.source.features.tools import dataclass_to_tuple, sleep
from API.source.models.classes.data_classes.command_templates import (
    MOTION_SETUP, MoveCommandTemplate
)
from API.source.models.classes.enum_classes.controller_commands import (
    AddWayPointCommand as Awp
)
from API.source.models.classes.enum_classes.state_classes import (
    OutComingMotionMode as Omm
)
from API.source.models.constants import (
    CTRLR_ADD_WP_CMD_PACK_FORMAT, OMM_ENABLE_DISABLE_PACK_FORMAT,
    WP_ADD_TIMEOUT, CHECK_FREQUENCY_SEC, WP_COUNT_LIMITS, WP_COUNTER_MAX_VALUE
)
from API.source.models.type_aliases import AngleUnits

if TYPE_CHECKING:
    from logging import Logger

    from API.source.ap_interface.motion.coordinate_system import (
        CoordinateSystem
    )
    from API.source.core.network.controller_socket import Controller
    from API.source.core.network.rtd_receiver_socket import RTDReceiver


validate_value = validation.validate_value


class Motion:

    _controller: Controller
    _rtd_receiver: RTDReceiver

    joint: JointMotion
    linear: LinearMotion
    scale_setup: MoveScaling
    mode: MotionMode
    kinematics: Kinematics

    def __init__(
        self, controller: Controller, rtd_receiver: RTDReceiver, logger: Logger
    ) -> None:

        self._logger = logger
        self._controller = controller
        self._rtd_receiver = rtd_receiver

        self.joint = JointMotion(
            self._controller, rtd_receiver, self._add_waypoint
        )
        self.linear = LinearMotion(
            self._controller, rtd_receiver, self._add_waypoint
        )
        self.scale_setup = MoveScaling(self._controller)
        self.mode = MotionMode(self._controller, rtd_receiver, self._logger)
        self.kinematics = Kinematics(self._controller, self.joint)
        self.waypoint_setup = None
        self._waypoint_counter = None

    def _add_waypoint(self, command_template: MoveCommandTemplate) -> bool:
        """
        Полный функционал добавления целевой точки (рад, м, с) в системе
        координат основания робота.

        Args:
            command_template: Предзаполненный объект дата-класса, содержащий
            все необходимые поля для команды.

        Returns:
            True: В случае успешного добавления точки.

        Raises:
            AddWaypointError: В случае таймаута ожидания добавления точки.
        """

        if self._waypoint_counter is None:
            self._waypoint_counter = self._rtd_receiver.rt_data.wp_cntr
        self._waypoint_counter = (
            (self._waypoint_counter + 1) & WP_COUNTER_MAX_VALUE
        )
        self._controller.send(
            Awp.ctrlr_coms_move_add_wp,
            pack(
                CTRLR_ADD_WP_CMD_PACK_FORMAT,
                *dataclass_to_tuple(command_template)
            )
        )
        for _ in sleep(
            await_sec=WP_ADD_TIMEOUT, frequency=CHECK_FREQUENCY_SEC
        ):
            if self._rtd_receiver.rt_data.wp_cntr == self._waypoint_counter:
                return True
        self._waypoint_counter -= 1
        raise AddWaypointError

    @staticmethod
    def set_motion_config(
        units: AngleUnits = None,
        joint_speed: float = None,
        joint_acceleration: float = None,
        linear_speed: float = None,
        linear_acceleration: float = None,
        blend: float = None
    ):
        """
        Установка глобальной настройки для линейного и моторного типов
        движения.

        Args:
            units: Единицы измерения.
                'deg' — градусы.
                'rad' — радианы.
            joint_speed: Скорость моторов ('units'/c) (0-180 deg / 0-3.14 rad).
            joint_acceleration: Ускорение моторов ('units'/c)
                (0-1500 deg / 0-26.18 rad).
            linear_speed: Скорость перемещения точки (0 - 3 м/с).
            linear_acceleration: Ускорение перемещения точки (0 - 15 м/c^2).
            blend: Радиус сглаживания движения (м).
                (Радиус вокруг точки, при пересечении которого траекторией
                движения робота начинается/заканчивается сглаживание).
        """

        if units:
            MOTION_SETUP.units = units
        if joint_speed:
            MOTION_SETUP.joint_speed = joint_speed
        if joint_acceleration:
            MOTION_SETUP.joint_acceleration = joint_acceleration
        if linear_speed:
            MOTION_SETUP.linear_speed = linear_speed
        if linear_acceleration:
            MOTION_SETUP.linear_acceleration = linear_acceleration
        if blend:
            MOTION_SETUP.blend = blend

    def free_drive(self, enable: bool = True) -> bool:
        """
        Режим 'FREE DRIVE'. Позволяет управлять роботом вручную.
        Команда является циклической. Для корректной работы режима необходимо
        вызывать метод в цикле, с частотой 100 Hz.

        Args:
            enable: Состояние режима.
        Returns:
            True: В случае успешной отправки команды.
        """

        return self._controller.send(
            Omm.zero_gravity,
            pack(OMM_ENABLE_DISABLE_PACK_FORMAT, bool(enable))
        )

    def simple_joystick(
        self,
        coordinate_system: CoordinateSystem = None
    ) -> bool:
        """
        Простой графический интерфейс для режима 'TCP JOGGING', основанный на
        функции 'jog_once'. Для работы с пользовательской СК необходимо
        передать ее в качестве аргумента. Функция является блокирующей. Для
        выхода из функции и продолжения исполнения программы необходимо закрыть
        пользовательский интерфейс.

        Args:
            coordinate_system: Пользовательская система координат.
        Returns:
            True: По завершении работы функции.
        """

        SimpleJoystickUI(
            self.linear.jog_once,
            self.joint.jog_once,
            self.joint.set_jog_param_in_tcp,
            self.linear.get_actual_position,
            self.joint.get_actual_position,
            self.free_drive,
            self.mode.set,
            self.joint.add_new_waypoint,
            self.linear.add_new_offset,
            self.scale_setup.set,
            coordinate_system
        )
        return True

    def wait_waypoint_completion(
        self, waypoint_count: int = 0, await_sec: int = -1
    ) -> bool:
        """
        Ожидать завершения исполнения текущих заданных целевых точек.

        Args:
            waypoint_count: Количество точек в очереди, после которого можно
                продолжить исполнение программы.
            await_sec: Лимит времени ожидания.
                -1 — безлимитное ожидание.
                0 — одна итерация цикла ожидания (эквивалентно разовому
                условию).
        Returns:
            True: В случае успешного исполнения заданных точек.
            False: В случае таймаута (если await_sec >= 0).
        """

        validate_value(waypoint_count, WP_COUNT_LIMITS)
        waypoint_amount = 0
        for _ in sleep(await_sec=await_sec, frequency=CHECK_FREQUENCY_SEC):
            if waypoint_amount != self._rtd_receiver.rt_data.buff_fill:
                self._logger.debug(
                    (
                        f'Waypoint in queue: '
                        f'{self._rtd_receiver.rt_data.buff_fill}'
                    )
                )
                waypoint_amount = self._rtd_receiver.rt_data.buff_fill
            if waypoint_count >= self._rtd_receiver.rt_data.buff_fill:
                self._logger.debug('Waypoint queue is empty')
                return True
        return False
