from __future__ import annotations

from logging import Logger

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.base_api_error import ApiError
from api.robot_api_rc5v15.API.source.core.exceptions.connection_error import (
    RuntimeMotionError,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    AddWaypointError,
)
from api.robot_api_rc5v15.API.source.core.network import Controller, RealtimeController, RTDReceiver
from api.robot_api_rc5v15.API.source.features.decorators import transforms_coordinates
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.mathematics.coordinate_system import (
    convert_position_orientation,
    convert_velocity,
)
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    degrees_to_radians,
    radians_to_degrees,
)
from api.robot_api_rc5v15.API.source.features.tools import (
    sleep,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    MoveCommandTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    AddWayPointErrorCode,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    ADD_WAY_POINT_ERROR_DESCRIPTION,
    CHECK_FREQUENCY_SEC,
    FKINE_IKINE_RESPONSE_JOINT_POSITION_SLICE,
    MOVE_TO_HOME_POSE_ACCEL,
    MOVE_TO_HOME_POSE_SPEED,
    ORIENTATION_SLICE,
    POSITION_ORIENTATION_LENGTH,
    POSITION_SLICE,
    WP_ADD_TIMEOUT,
    WP_COUNT_LIMITS,
    WP_COUNTER_MAX_VALUE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    PositionFormat,
    PositionOrientation,
)

from .coordinate_system import CoordinateSystem
from .joint_motion import JointMotion
from .kinematics_solution import Kinematics
from .linear_motion import LinearMotion
from .motion_mode import MotionMode
from .move_scaling import MoveScaling
from .realtime_motion import Realtime
from .rpmp_motion import RPMPMotion


class Motion(LoggerMixin):
    """
    Класс для управления движением робота.
    """

    _controller: Controller
    _realtime_controller: RealtimeController
    _rtd_receiver: RTDReceiver
    _connection_state: ConnectionState

    joint: JointMotion
    """Подкласс для работы с движением типа 'joint'."""
    linear: LinearMotion
    """Подкласс для работы с движением типа 'linear'."""
    advanced: RPMPMotion
    """Подкласс для работы с движением типа 'advanced'."""
    _realtime: Realtime
    """Подкласс для управления движением в реальном времени."""
    scale_setup: MoveScaling
    """Подкласс для работы с настройками параметров движения."""
    mode: MotionMode
    """Подкласс для работы с режимами движения."""
    kinematics: Kinematics
    """Подкласс для получения решений задач кинематики."""

    @property
    def realtime(self) -> Realtime:
        """
        Управление в реальном времени доступно только если подключен
        соответствующий контроллер и установлен режим движения 'realtime'.
        """
        if not self._realtime_controller.is_connected():
            raise RuntimeMotionError(
                "Realtime controller is not connected, probably current "
                "robot does not support realtime control"
            )
        return self._realtime

    def __init__(
        self,
        controller: Controller,
        realtime_controller: RealtimeController,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: Logger | None,
        log_realtime: bool = True,
    ) -> None:
        self._set_logger(logger)
        self._controller = controller
        self._realtime_controller = realtime_controller
        self._connection_state = connection_state
        self._rtd_receiver = rtd_receiver

        self.joint = JointMotion(
            controller=self._controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
            motion_host=self,
        )
        self.linear = LinearMotion(
            controller=self._controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
            motion_host=self,
        )
        self.advanced = RPMPMotion(
            controller=self._controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
            logger=self._get_logger(),
        )
        self._realtime = Realtime(
            realtime_controller=self._realtime_controller,
            rtd_receiver=self._rtd_receiver,
            connection_state=self._connection_state,
            logger=self._get_logger(),
            log_realtime=log_realtime,
        )

        self.scale_setup = MoveScaling(
            controller=self._controller,
            connection_state=self._connection_state,
        )
        self.mode = MotionMode(
            controller=self._controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
            logger=self._get_logger(),
        )
        self.kinematics = Kinematics(
            controller=self._controller,
            rtd_receiver=rtd_receiver,
            connection_state=self._connection_state,
        )
        self._waypoint_counter = None

    @handle_connection(available_in_read_only=False)
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

        self._waypoint_counter = self._rtd_receiver.get_data().wp_cntr
        self._waypoint_counter = (
            self._waypoint_counter + 1
        ) & WP_COUNTER_MAX_VALUE
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_MOVE_ADD_WP, command_template
        )
        if response[0] != AddWayPointErrorCode.success:
            self._waypoint_counter = None
            error_description = ADD_WAY_POINT_ERROR_DESCRIPTION.get(
                response[0], None
            )
            if error_description is None:
                self._write_log(
                    "error",
                    f"Received unknown add waypoint error status: {response[0]}",
                )
            raise AddWaypointError(error_description)
        for _ in sleep(
            await_sec=WP_ADD_TIMEOUT,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error", "Failed to add waypoint - connection was lost"
                )
                return False
            if self._rtd_receiver.get_data().wp_cntr == self._waypoint_counter:
                return True
        self._waypoint_counter = None
        raise AddWaypointError

    @staticmethod
    def set_motion_config(
        units: AngleUnits | None = None,
        joint_speed: float | None = None,
        joint_acceleration: float | None = None,
        linear_speed: float | None = None,
        linear_acceleration: float | None = None,
        blend: float | None = None,
        advanced_joints_speed: PositionOrientation | None = None,
        advanced_joints_accel: PositionOrientation | None = None,
        advanced_rotation_speed: float | None = None,
        advanced_rotation_acceleration: float | None = None,
    ):
        """Устанавливает глобальные параметры движения для всех последующих команд.

        Настройки применяются к линейным и угловым (по сочленениям) движениям
        и влияют на скорость, ускорение и плавность траекторий.
        Параметры, переданные как `None` **не изменяются**.

        Метод доступен в режиме «read only», так как не запускает движение,
        а только конфигурирует его параметры.

        Args:
            units (AngleUnits, optional): Единицы измерения углов.
                - `'deg'` — градусы (диапазон: 0–180 °/с для скорости);
                - `'rad'` — радианы (диапазон: 0–3.14 рад/с).
                По умолчанию сохраняется текущая настройка.
            joint_speed (float, optional): Максимальная скорость сочленений.
                - В градусах: 0–180 °/с;
                - В радианах: 0–3.14 рад/с.
            joint_acceleration (float, optional): Максимальное ускорение сочленений.
                - В градусах: 0–1500 °/с²;
                - В радианах: 0–26.18 рад/с².
            linear_speed (float, optional): Максимальная линейная скорость
                конечного звена (TCP). Диапазон: 0–3 м/с.
            linear_acceleration (float, optional): Максимальное линейное ускорение
                TCP. Диапазон: 0–15 м/с².
            blend (float, optional): Радиус сглаживания траектории в метрах.
                При приближении к точке на расстояние ≤ `blend` робот начинает
                плавный переход к следующему сегменту траектории, не останавливаясь.
                Значение `0` отключает сглаживание (движение по точкам с остановкой).
            advanced_joints_speed (PositionOrientation, optional): Желаемые
                скорости поворотов каждого сочленения в режиме движения 'advanced'.
            advanced_joints_accel (PositionOrientation, optional): Желаемые
                ускорения поворотов каждого сочленения в режиме движения 'advanced'.
            advanced_rotation_speed (float, optional): Желаемая скорость вращения
                ЦТИ в режиме движения 'advanced'.
            advanced_rotation_acceleration (float, optional): Желаемое ускорение
                вращения ЦТИ в режиме движения 'advanced'.

        Examples:
            >>> # Установить умеренные параметры движения в градусах
            >>> robot.motion.set_motion_config(
            ...     units='deg',
            ...     joint_speed=30,
            ...     joint_acceleration=300,
            ...     linear_speed=0.5,
            ...     linear_acceleration=2.0,
            ...     blend=0.05  # 5 см сглаживания
            ... )

            >>> # Только изменить скорость, остальное оставить как есть
            >>> robot.motion.set_motion_config(linear_speed=1.0)

        Notes:
            - Изменения применяются **глобально** и влияют на все будущие движения,
              пока не будут изменены снова или в команде движения явно не указано другое.
        """

        MOTION_SETUP.set_value("units", units)
        MOTION_SETUP.set_value("joint_speed", joint_speed, units)
        MOTION_SETUP.set_value("joint_acceleration", joint_acceleration, units)
        MOTION_SETUP.set_value("linear_speed", linear_speed, units)
        MOTION_SETUP.set_value(
            "linear_acceleration", linear_acceleration, units
        )
        MOTION_SETUP.set_value("blend", blend, units)
        MOTION_SETUP.set_value("joints_speed", advanced_joints_speed, units)
        MOTION_SETUP.set_value("joints_accel", advanced_joints_accel, units)
        MOTION_SETUP.set_value(
            "rotation_speed", advanced_rotation_speed, units
        )
        MOTION_SETUP.set_value(
            "rotation_acceleration", advanced_rotation_acceleration, units
        )

    @handle_connection(available_in_read_only=True)
    @transforms_coordinates(system_param="coordinate_system")
    def get_actual_position(
        self,
        orientation_units: AngleUnits | None = None,
        position_format: PositionFormat = "tcp",
        coordinate_system: CoordinateSystem | None = None,
    ) -> PositionOrientation:
        """Возвращает текущее положение робота в заданном формате и системе координат.

        Метод позволяет получить либо углы сочленений (joint-space),
        либо декартову позицию и ориентацию конечного инструмента (TCP)
        в линейном пространстве (task-space).

        Метод доступен в режиме «read only».

        Args:
            orientation_units (AngleUnits, optional): Единицы измерения углов
                ориентации. Допустимые значения:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
            position_format (PositionFormat, optional): Формат выходных данных.
                - `'tcp'` — возвращает позицию и ориентацию TCP в виде
                  `(X, Y, Z, Rx, Ry, Rz)`, где:
                    * `(X, Y, Z)` — координаты в метрах,
                    * `(Rx, Ry, Rz)` — углы поворота вокруг осей в указанных
                      единицах (`orientation_units`);
                - `'joints'` — возвращает углы сочленений в виде
                  `(J1, J2, J3, J4, J5, J6)`, где `Ji` — угол поворота i-го
                  звена от основания. Углы возвращаются в тех же единицах,
                  что указаны в `orientation_units`.
                По умолчанию: `'tcp'`.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой возвращается TCP. Если не задана, используется
                глобальная система координат основания робота.
                Может быть пользовательской системой, заданной через
                `robot.coordinate_system`.
        Returns:
            PositionOrientation: кортеж, содержащий 6 чисел:
                - для `'tcp'`: `(X, Y, Z, Rx, Ry, Rz)`;
                - для `'joints'`: `(J1, J2, J3, J4, J5, J6)`.

        Examples:
            >>> # Получить углы сочленений в градусах
            >>> joints = robot.motion.get_actual_position(position_format='joints')
            >>> print(f"Углы: {joints}")

            >>> # Получить позицию TCP в пользовательской системе координат
            >>> from API.coords import CoordinateSystem
            >>> local_coord_system = CoordinateSystem(
            ...     position_orientation=((-0.3332, -0.1838, -0.0198, 3.138, 0, 0.8195)),
            ...     orientation_units="deg",
            ... )
            >>> pose = robot.motion.get_actual_position(
            ...     coordinate_system=local_coord_system,
            ...     orientation_units='deg'
            ... )
            >>> x, y, z, rx, ry, rz = pose

        Notes:
            - При `position_format='joints'` параметр `coordinate_system`
              игнорируется.
            - Ориентация `(Rx, Ry, Rz)` задана в формате **поворотов вокруг
              неподвижных осей (extrinsic XYZ)**.
            - Все линейные значения возвращаются в **метрах**.
        """

        orientation_units = orientation_units or MOTION_SETUP.units
        Validation.literal("angle", orientation_units)
        Validation.literal("position", position_format)

        if position_format == "joints":
            joints_pose = list(self._rtd_receiver.get_data().act_q)
            if orientation_units == "rad":
                return joints_pose
            return radians_to_degrees(joints_pose)
        tcp_pose = list(self._rtd_receiver.get_data().act_tcp_x)
        if coordinate_system:
            tcp_pose = list(
                convert_position_orientation(
                    coordinate_system,
                    tcp_pose,
                    orientation_units="rad",
                    to_local=True,
                )
            )
        if orientation_units == "rad":
            return tcp_pose
        pose = tcp_pose[POSITION_SLICE] + radians_to_degrees(
            tcp_pose[ORIENTATION_SLICE]
        )
        return pose

    @handle_connection(available_in_read_only=True)
    @transforms_coordinates(system_param="coordinate_system")
    def get_actual_velocity(
        self,
        orientation_units: AngleUnits | None = None,
        position_format: PositionFormat = "tcp",
        coordinate_system: CoordinateSystem | None = None,
    ) -> PositionOrientation:
        """Возвращает текущие скорости сочленений или центра инструмента (TCP) в
        заданном формате и системе координат.

        Метод позволяет получить либо угловые скорости всех приводов (joint-space),
        либо линейную и угловую скорости конечного инструмента в декартовом
        пространстве (task-space).

        Метод доступен в режиме «read only».

        Args:
            orientation_units (AngleUnits, optional): Единицы измерения угловых скоростей.
                Допустимые значения:
                - `'deg'` — градусы в секунду (по умолчанию);
                - `'rad'` — радианы в секунду.
                Линейные скорости всегда возвращаются в метрах в секунду (м/с).
            position_format (PositionFormat, optional): Формат выходных данных.
                - `'tcp'` — возвращает скорость TCP в виде `(vx, vy, vz, wx, wy, wz)`, где:
                  * `(vx, vy, vz)` — линейная скорость центра инструмента (м/с),
                  * `(wx, wy, wz)` — угловая скорость (в единицах `orientation_units`);
                - `'joints'` — возвращает угловые скорости сочленений в виде
                  `(dq1, dq2, dq3, dq4, dq5, dq6)`, где `dqi` — скорость вращения i-го
                  сочленения от основания к фланцу (в единицах `orientation_units`).
                По умолчанию: `'tcp'`.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой возвращаются векторы скорости TCP. Преобразование выполняется
                строго через матрицу поворота (без учёта смещения начала координат,
                что физически корректно для векторов скорости). Если не задана,
                используется глобальная система координат основания робота.

        Returns:
            PositionOrientation: Кортеж из 6 чисел:
                - для `'tcp'`: `(vx, vy, vz, wx, wy, wz)`;
                - для `'joints'`: `(dq1, dq2, dq3, dq4, dq5, dq6)`.

        Examples:
            >>> # Получить угловые скорости сочленений в радианах в секунду
            >>> joints_vel = robot.motion.get_actual_velocity(
            ...     position_format='joints', orientation_units='rad'
            ... )
            >>> print(f"Скорости суставов: {joints_vel}")

            >>> # Получить скорость TCP в локальной системе координат
            >>> from API.coords import CoordinateSystem
            >>> local_cs = CoordinateSystem(
            ...     position_orientation=((0.5, 0.0, 0.0, 0.0, 0.0, 90.0)),
            ...     orientation_units="deg",
            ... )
            >>> tcp_vel = robot.motion.get_actual_velocity(
            ...     coordinate_system=local_cs,
            ...     orientation_units='deg'
            ... )
            >>> vx, vy, vz, wx, wy, wz = tcp_vel

        Notes:
            - При `position_format='joints'` параметр `coordinate_system` игнорируется.
            - Линейная скорость `(vx, vy, vz)` всегда возвращается в **метрах в секунду (м/с)**.
            - Угловая скорость `(wx, wy, wz)` возвращается в единицах, указанных в
              `orientation_units` (`deg/s` или `rad/s`).
            - Преобразование в пользовательскую систему координат учитывает только
              **ориентацию** базиса. Смещение начала координат (трансляция) не влияет
              на векторы скорости, так как это свободные векторы в кинематике.
            - Значения отражают мгновенную скорость на момент вызова и могут
              использоваться для мониторинга нагрузки или диагностики динамики движения.
        """

        orientation_units = orientation_units or MOTION_SETUP.units
        Validation.literal("angle", orientation_units)
        Validation.literal("position", position_format)

        if position_format == "joints":
            joints_velocity = list(self._rtd_receiver.get_data().act_qd)
            if orientation_units == "rad":
                return joints_velocity
            return radians_to_degrees(joints_velocity)
        tcp_velocity = list(self._rtd_receiver.get_data().act_tcp_xd)
        if coordinate_system:
            tcp_velocity = list(
                convert_velocity(
                    coordinate_system,
                    tcp_velocity,
                    orientation_units="rad",
                    to_local=True,
                )
            )
        if orientation_units == "rad":
            return tcp_velocity
        pose = tcp_velocity[POSITION_SLICE] + radians_to_degrees(
            tcp_velocity[ORIENTATION_SLICE]
        )
        return pose

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    @transforms_coordinates(system_param="coordinate_system")
    def get_last_saved_position(
        self,
        orientation_units: AngleUnits | None = None,
        position_format: PositionFormat = "joints",
        coordinate_system: CoordinateSystem | None = None,
    ) -> PositionOrientation | None:
        """Возвращает последнюю сохранённую позицию робота в заданном формате.

        Сохранённая позиция — это состояние, зафиксированное в памяти контроллера.
        Метод позволяет получить её либо в виде углов сочленений, либо в виде
        декартовой позиции и ориентации TCP.

        Args:
            orientation_units (AngleUnits, optional): Единицы измерения углов.
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
            position_format (PositionFormat, optional): Формат вывода:
                - `'joints'` — возвращает углы сочленений `(J1, J2, ..., J6)`
                  в указанных единицах;
                - `'tcp'` — возвращает позицию и ориентацию конечного инструмента
                  `(X, Y, Z, Rx, Ry, Rz)`, где линейные координаты в метрах,
                  углы — в `orientation_units`.
                По умолчанию: `'joints'`.
            coordinate_system (CoordinateSystem, optional): Система координат
                для формата `'tcp'`. Если не указана, используется система
                координат основания робота. Игнорируется при `position_format='joints'`.

        Returns:
            PositionOrientation | None:
                - Объект с 6 значениями (в зависимости от `position_format`),
                  соответствующий последней сохранённой позиции;
                - `None`, если получить позицию не удалось.

        Examples:
            >>> # Получить последнюю сохранённую позицию в углах сочленений (градусы)
            >>> last_joints = robot.motion.get_last_saved_position()
            >>> if last_joints:
            ...     print(f"Последняя позиция: {last_joints}")

            >>> # Получить в TCP-формате в радианах
            >>> last_pose = robot.motion.get_last_saved_position(
            ...     position_format='tcp',
            ...     orientation_units='rad'
            ... )

        Notes:
            - Возвращаемая позиция **не обязательно совпадает** с текущей
                фактической позицией робота — она отражает состояние на момент
                последнего сохранения.
            - При `position_format='joints'` параметр `coordinate_system` игнорируется.
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        Validation.literal("angle", orientation_units)
        Validation.literal("position", position_format)

        last_pose = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_LAST_POS
        )
        if len(last_pose) != 6:
            return None

        if position_format == "joints":
            if orientation_units == "deg":
                return radians_to_degrees(last_pose)
            return last_pose

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_FKINE, last_pose
        )
        tcp_pose = list(response[FKINE_IKINE_RESPONSE_JOINT_POSITION_SLICE])

        if not response or response[0] != 0:
            return None
        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system,
                tcp_pose,
                orientation_units="rad",
                to_local=True,
            )
        if orientation_units == "deg":
            tcp_pose = list(tcp_pose[POSITION_SLICE]) + radians_to_degrees(
                tcp_pose[ORIENTATION_SLICE]
            )
        return tcp_pose

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def is_point_reachable(
        self,
        tcp_pose: PositionOrientation,
        angle_pose: PositionOrientation | None = None,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Проверяет достижимость точки роботом.

        Проверяет достижимость точки роботом. Конвертация позиции и
        ориентации из локальной СК (пользовательской) в глобальную производится
        при передаче локальной системы координат в качестве аргумента.

        Args:
            tcp_pose: Позиция и ориентация ЦТИ в глобальной
                системе координат (система координат основания робота) в
                формате:
                (X, Y, Z, Rx, Ry, Rz), где (X, Y, Z) — м,
                (Rx, Ry,Rz) — 'orientation_units'.
            angle_pose: Положение углов поворота сочленений, относительно которого
                будет рассчитано ближайшее решение обратной задачи кинематики.
                По-умолчанию: текущие углы поворотов.
            orientation_units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задается точка. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.

        Returns:
            bool: True если точка достижима, иначе False.

        Examples:
            >>> # Проверить достижимость точки
            >>> target = (0.4, 0.0, 0.5, 0.0, 3.14, 0.0)
            >>> reachable = robot.motion.is_point_reachable(tcp_pose=target)
            >>> if reachable:
            >>>     print("Точка достижима")
            >>> else:
            >>>     print("Точка недостижима")

            >>> # Указать опорную позицию для выбора позы в точке
            >>> reference = (0, -90, 0, -90, 0, 0)
            >>> reachable = robot.motion.is_point_reachable(
            ...     tcp_pose=target,
            ...     angle_pose=reference,
            ...     orientation_units='deg'
            ... )
            >>> if reachable:
            >>>     print("Точка достижима")
            >>> else:
            >>>     print("Точка недостижима")

        Notes:
            - Если `angle_pose` не задан, в качестве опорной используется
              **текущая позиция робота** (из RTD).
            - Этот метод **недоступен в режиме read-only**.
        """
        pose = self.kinematics.get_inverse(
            tcp_pose=tcp_pose,
            angle_pose=angle_pose,
            orientation_units=orientation_units,
            get_all=False,
            coordinate_system=coordinate_system,
        )
        if pose is None or isinstance(pose, tuple):
            return False
        return all(abs(param) >= 1e-6 for param in pose)

    @handle_connection(available_in_read_only=True)
    def check_waypoint_completion(self, waypoint_count: int = 0) -> bool:
        """Проверяет, завершено ли выполнение текущих целевых точек — без блокировки.

        Метод сравнивает количество оставшихся невыполненных точек в буфере
        контроллера с заданным порогом `waypoint_count`. Возвращает `True`,
        если точек **осталось не более**, чем указано.

        Метод доступен в режиме «read only».

        Проверить состояние исполнения текущих заданных целевых точек
        (без ожидания).

        Args:
            waypoint_count (int, optional): Пороговое количество точек в буфере.
                По умолчанию `0` — проверяется полное прохождение всех точек.

        Returns:
            True: Если точки исполнены (целевых точек в буфере меньше, чем
                `waypoint_count`).
            False: Если точки не исполнены (точек в буфере больше, чем
                `waypoint_count`).

        Examples:
            >>> # Проверить, пройдены ли все точки
            >>> if robot.motion.check_waypoint_completion():
            ...     print("Все точки пройдены")
        """

        Validation.value(waypoint_count, WP_COUNT_LIMITS, "waypoint_count")
        return self._rtd_receiver.get_data().buff_fill <= waypoint_count

    @handle_connection(available_in_read_only=True)
    def get_waypoint_buffer_size(self) -> int:
        """Возвращает текущее количество точек в буфере выполнения контроллера.

        Значение включает **точку, находящуюся в процессе выполнения**, и все
        **ожидающие точки** в очереди. Полезно для мониторинга загрузки буфера
        и отладки траекторий.

        Метод доступен в режиме «read only».

        Returns:
            int: Текущее количество точек в буфере выполнения.

        Examples:
            >>> # Узнать, сколько точек сейчас в буфере
            >>> buffer_size = robot.motion.get_waypoint_buffer_size()
            >>> print(f"В буфере: {buffer_size} точек")
        """
        return self._rtd_receiver.get_data().buff_fill

    @handle_connection(available_in_read_only=False)
    def get_home_pose(
        self, units: AngleUnits | None = None
    ) -> PositionOrientation:
        """Возвращает текущую домашнюю позицию робота.

        Домашняя позиция — это позиция, которая считается "начальной" для робота.

        Args:
            units (AngleUnits, optional): Единицы измерения углов сочленений:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.

        Returns:
            PositionOrientation: Объект, содержащий 6 углов сочленений
                `(J1, J2, J3, J4, J5, J6)` в указанных единицах,
                представляющих домашнюю позицию от основания до фланца робота.

        Examples:
            >>> # Получить домашнюю позицию в градусах
            >>> home = robot.motion.get_home_pose()
            >>> print(f"Домашняя позиция: {home}")

            >>> # Получить в радианах
            >>> home_rad = robot.motion.get_home_pose(units='rad')

        Notes:
            - Возвращаемая позиция **не обязательно совпадает** с текущей
              фактической позицией робота.
            - Для перемещения робота в домашнюю позицию используйте метод
              `move_to_home_pose()`.
        """
        units = units or MOTION_SETUP.units
        Validation.literal("angle", units)
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_HOME_POSE
        )
        if units == "deg":
            return radians_to_degrees(response)
        return response

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=True
    )
    def free_drive(self, enable: bool = True) -> bool:
        """Активирует режим ручного перемещения «Free Drive».

        В этом режиме оператор может физически перемещать руку робота,
        а система управления компенсирует гравитацию и минимизирует сопротивление.
        Режим предназначен для ручного позиционирования, обучения или настройки.

        Команда **требует циклического подтверждения**. Для корректной
        работы метод должен вызываться **не реже 100 Гц** (каждые 10 мс),
        пока режим активен. Если вызовы прекращаются, контроллер автоматически
        выходит из режима Free Drive.

        Args:
            enable (bool, optional):
                - `True` — активировать режим Free Drive;
                - `False` — деактивировать.
                По умолчанию: `True`.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Активировать Free Drive на 5 секунд
            >>> import time
            >>> start = time.time()
            >>> while time.time() - start < 5.0:
            ...     robot.motion.free_drive(True)  # вызывать ≥100 Гц
            ...     time.sleep(0.005)  # 200 Гц — безопасная частота
            >>> robot.motion.free_drive(False)  # явное отключение
        """

        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_ZG, bool(enable)
        )

    @handle_connection(available_in_read_only=False)
    def move_to_home_pose(self):
        """Перемещает робота в домашнюю позицию.

        Перед началом движения метод автоматически:
        1. Останавливает любое текущее движение;
        2. Очищает буфер целевых точек;
        3. Инициирует перемещение к домашней позиции.

        Returns:
            True: В случае начала перемещения в домашнюю позицию.

        Examples:
            >>> # Переместиться в домашнюю позицию
            >>> if robot.motion.move_to_home_pose():
            ...     print("Начато перемещение домой")
            ... else:
            ...     print("Не удалось инициировать движение")

            >>> # Дождаться завершения (опционально)
            >>> robot.motion.wait_waypoint_completion()

        Notes:
            - Метод **не ждёт завершения движения** — он только запускает его.
              Используйте `wait_waypoint_completion()`, если требуется синхронное выполнение.
            - Домашняя позиция должна быть предварительно задана (через
              `set_home_pose`) или определена по умолчанию.
        """
        return (
            self.mode.set("hold")
            and self.joint.add_new_waypoint(
                angle_pose=self.get_home_pose(units="rad"),
                speed=MOVE_TO_HOME_POSE_SPEED,
                accel=MOVE_TO_HOME_POSE_ACCEL,
                units="rad",
            )
            and self.mode.set("move")
        )

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    def set_home_pose(
        self,
        angle_pose: PositionOrientation,
        units: AngleUnits | None = None,
    ) -> bool:
        """Устанавливает новую домашнюю позицию робота.

        Домашняя позиция — это "исходной" положение, используемое для возврата
        робота в исходное положение (например, через `move_to_home_pose`).
        Эта позиция сохраняется в памяти контроллера и может быть восстановлена
        после перезапуска.

        Args:
            angle_pose (PositionOrientation): Последовательность из 6 углов
                сочленений `(J1, J2, J3, J4, J5, J6)`, представляющих желаемую
                домашнюю позицию от основания до фланца робота.
            units (AngleUnits, optional): Единицы измерения углов в `angle_pose`:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
                Если не указано, предполагаются градусы.

        Returns:
            bool: True — если домашняя позиция успешно установлена в контроллере;

        Examples:
            >>> # Установить домашнюю позицию в градусах
            >>> home_angles = (0.0, -90.0, 0.0, -90.0, 0.0, 0.0)
            >>> robot.motion.set_home_pose(home_angles)

            >>> # Установить в радианах
            >>> import math
            >>> home_rad = (0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0)
            >>> robot.motion.set_home_pose(home_rad, units='rad')

        Notes:
            - Рекомендуется устанавливать домашнюю позицию в безопасной,
              легко достижимой конфигурации (например, в центре рабочей зоны).
        """
        units = units or MOTION_SETUP.units
        Validation.literal("angle", units)
        Validation.length(
            angle_pose, POSITION_ORIENTATION_LENGTH, "angle_pose"
        )
        if units == "deg":
            angle_pose = degrees_to_radians(angle_pose)
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_HOME_POSE, angle_pose
        )

    @handle_connection(
        available_in_read_only=False, available_in_impulse_compat_mode=False
    )
    @transforms_coordinates(system_param="coordinate_system")
    def simple_joystick(
        self, coordinate_system: CoordinateSystem | None = None
    ) -> bool:
        """Запускает встроенный графический интерфейс для интерактивного управления роботом.

        Интерфейс позволяет управлять роботом в реальном времени с помощью
        виртуального джойстика и предоставляет следующий функционал:
            - изменение углового положения каждого звена по отдельности;
            - изменение положения и ориентации ЦТИ по осям системы координат
                основания или осям системы координат ЦТИ;
            - изменение глобального множителя скорости;
            - переход в положение «семерка»;
            - активация режима «Свободный привод»;
            - отображение и возможность копирования текущих углов поворота
                шарниров и положения ЦТИ в выбранной системе координат с
                указанными единицами измерения угловых положений;
            - перемещение робота в указанное угловое положение
                (по углам сочленений);
            - линейное перемещение робота в указанное положение ЦТИ в
                выбранной системе координат;
            - линейное смещение робота на указанное расстояние от текущего
                положения ЦТИ в выбранной системе координат.

        Метод **блокирующий**: выполнение программы приостанавливается до тех пор,
        пока пользователь не закроет окно интерфейса.

        Метод может быть востребован при пусконаладочных работах или во время
        реализации какой-либо операции для калибровки положений целевых точек
        или их сбора.

        Args:
            coordinate_system (CoordinateSystem, optional): Пользовательская
                система координат. Если не задана, то предоставляется выбор
                только из системы координат основания робота и ЦТИ.

        Returns:
            bool: True — после корректного завершения работы интерфейса
                  (пользователь закрыл окно). В случае исключения (например,
                  отсутствие tkinter) метод выбросит ошибку.

        Examples:
            >>> # Запустить джойстик в глобальной СК
            >>> robot.motion.simple_joystick()

            >>> # Запустить в пользовательской системе координат
            >>> from API.coords import CoordinateSystem
            >>> local_coord_system = CoordinateSystem(
            ...     position_orientation=((-0.3332, -0.1838, -0.0198, 0.138, 0, 0.8195)),
            ...     orientation_units="rad",
            ... )
            >>> robot.motion.simple_joystick(coordinate_system=local_coord_system)

        Notes:
            - Интерфейс предназначен для отладки и ручного управления —
                не рекомендуется для использования в промышленных
                автоматизированных сценариях.
            - Графический интерфейс написан с помощью фреймворка 'tkinter', при
                этом его импорт реализован «ленивым» способом, то есть импорт
                происходит только при вызове данного метода. Такой подход
                позволяет обеспечить совместимость API с системами, на которых
                не установлен 'tkinter'. Если при вызове метода возникает
                ошибка, то следует проверить установку 'tkinter' на вашей
                системе.
        """

        try:
            # Безопасный импорт GUI. Используется для совместимости основной
            # части API с системами, не имеющими установленный tkinter.
            from api.robot_api_rc5v15.API.source.features.gui import (
                SimpleJoystickController,
            )

            SimpleJoystickController(
                motion=self, coordinate_system=coordinate_system
            )
            return True

        except ImportError as e:
            self._write_log(
                "error", f"Failed to start simple joystick, error: {e}"
            )
            raise ApiError(
                "Tkinter is needed to use this method, probably your python "
                "installation or system doesn't contain it"
            )

    @handle_connection(available_in_read_only=True)
    def wait_waypoint_completion(
        self, waypoint_count: int = 0, await_sec: int | float = -1
    ) -> bool:
        """Ожидает завершения выполнения целевых точек в буфере.

        Метод блокирует выполнение программы до тех пор, пока количество
        оставшихся невыполненных точек в буфере контроллера не станет
        **меньше или равно** `waypoint_count`, либо не истечёт заданное
        время ожидания.

        Метод доступен в режиме «read only».

        Args:
            waypoint_count (int, optional): Пороговое значение количества точек.
                Ожидание завершается, когда в буфере остаётся ≤ `waypoint_count` точек.
                По умолчанию `0` — ожидается полное выполнение всех точек.
            await_sec (int, optional): Максимальное время ожидания в секундах:
                - `-1` — ожидание без ограничения по времени (по умолчанию);
                - `0` — неблокирующая проверка: выполнить одну итерацию и вернуться;
                - `> 0` — ожидать не более указанного числа секунд.

        Returns:
            bool: True — если количество точек в буфере стало ≤ `waypoint_count`
                  в течение заданного времени;
                  False — если произошёл таймаут (`await_sec >= 0` и условие не выполнено).

        Examples:
            >>> # Дождаться полного выполнения всех точек (без таймаута)
            >>> robot.motion.wait_waypoint_completion()

            >>> # Дождаться, пока не останется ≤ 1 точки, максимум 10 секунд
            >>> if robot.motion.wait_waypoint_completion(waypoint_count=1, await_sec=10):
            ...     print("Готов к следующему действию")
            ... else:
            ...     print("Таймаут: движение не завершено")

            >>> # Неблокирующая проверка (аналог check_waypoint_completion)
            >>> done = robot.motion.wait_waypoint_completion(await_sec=0)

        Notes:
            - При `await_sec = -1` вызов может блокировать программу неограниченно —
              используйте с осторожностью в автоматизированных системах.
            - Этот метод часто используется после добавления серии точек.
        """

        Validation.value(waypoint_count, WP_COUNT_LIMITS, "waypoint_count")
        waypoint_amount = 0
        self._write_log(
            "debug",
            f"Waiting for {waypoint_count} waypoints in buffer "
            + (
                f"for no more then {await_sec:.2f} sec"
                if await_sec >= 0
                else ""
            ),
        )
        for _ in sleep(
            await_sec=await_sec,
            frequency=CHECK_FREQUENCY_SEC,
        ):
            if not self._connection_state.is_connected():
                self._write_log(
                    "error",
                    "Failed to wait waypoint completion - connection was lost",
                )
                return False
            if waypoint_amount != self._rtd_receiver.get_data().buff_fill:
                self._write_log(
                    "debug",
                    (
                        f"Waypoint in queue: "
                        f"{self._rtd_receiver.get_data().buff_fill}"
                    ),
                )
                waypoint_amount = self._rtd_receiver.get_data().buff_fill
            if waypoint_count >= self._rtd_receiver.get_data().buff_fill:
                self._write_log(
                    "debug", f"Waypoint queue is equals {waypoint_count}"
                )
                return True
        return False
