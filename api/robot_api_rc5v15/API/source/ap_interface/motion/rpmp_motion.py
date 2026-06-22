from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.exceptions.data_error import (
    AddWaypointError,
)
from api.robot_api_rc5v15.API.source.core.network import (
    Controller,
    RTDReceiver,
)
from api.robot_api_rc5v15.API.source.features.decorators import transforms_coordinates
from api.robot_api_rc5v15.API.source.features.logger import LoggerMixin
from api.robot_api_rc5v15.API.source.features.mathematics.coordinate_system import (
    convert_position_orientation,
)
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    degrees_to_radians,
)
from api.robot_api_rc5v15.API.source.features.tools import (
    set_position_orientation_units,
    sleep,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    RPMPCartesianWayPointTemplate,
    RPMPJointTcpWayPointTemplate,
    RPMPJointWayPointTemplate,
    RPMPMoveCommandTemplate,
    RPMPMoveCWayPointTemplate,
    RPMPMoveJTcpWayPointTemplate,
    RPMPMoveJWayPointTemplate,
    RPMPMoveLWayPointTemplate,
    RPMPMovePWayPointTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
    RpmpWayPointType,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    AddWayPointErrorCode,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    ACCEL_LIMITS,
    ADD_WAY_POINT_ERROR_DESCRIPTION,
    BLEND_LIMITS,
    CHECK_FREQUENCY_SEC,
    ORIENTATION_SLICE,
    POSITION_ORIENTATION_LENGTH,
    POSITION_SLICE,
    SPEED_LIMITS,
    WP_ADD_TIMEOUT,
    WP_COUNTER_MAX_VALUE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    IkSolutionId,
    PositionOrientation,
)

if TYPE_CHECKING:
    from .coordinate_system import CoordinateSystem


class RPMPMotion(LoggerMixin):
    """
    Класс для работы типом движения 'Advanced'.
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
        logger: logging.Logger | None,
    ) -> None:
        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._connection_state = connection_state
        self._waypoint_counter: int | None = None
        self._set_logger(logger)

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def add_movel_waypoint(
        self,
        tcp_pose: PositionOrientation,
        translation_speed: float | None = None,
        translation_accel: float | None = None,
        rotation_speed: float | None = None,
        rotation_accel: float | None = None,
        blend: float | None = None,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Добавляет точку линейного движения (MoveL) в буфер точек Advanced-движения.

        Добавить целевую точку типа 'linear' для типа движения 'Advanced' в
        глобальной системе координат (система координат основания робота).
        Конвертация позиции и ориентации из локальной СК (пользовательской) в
        глобальную производится при передаче локальной системы координат в
        качестве аргумента.

        Добавление точки типа 'linear' задает движение от предыдущей добавленной
        точки к текущей по линейной траектории с соответствующими ограничениями
        и параметрами.

        Args:
            tcp_pose: Позиция ЦТИ в формате (X, Y, Z, Rx, Ry, Rz),
                где (X, Y, Z) — м, (Rx, Ry,Rz) — 'orientation_units'.
            translation_speed: Желаемая скорость поступательного перемещения
                точки по траектории (0 - 3 м/с).
            translation_accel: Желаемое ускорение поступательного перемещения
                точки по траектории (0 - 15 м/c^2).
            rotation_speed: Желаемая скорость вращательного перемещения
                точки ('orientation_units'/c) (0 - 360 deg/с / 0 - 6.28 rad/с).
            rotation_accel: Желаемое ускорение вращательного перемещения
                точки ('orientation_units'/c^2) (0 - 720 deg/с^2 / 0 - 12.56 rad/с^2).
            blend: Радиус сглаживания движения (м).
                (Радиус вокруг точки, при пересечении которого траекторией
                движения робота начинается/заканчивается сглаживание).
            orientation_units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задается точка. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Добавить MoveL-точку
            >>> pose = (0.4, 0.0, 0.6, 0.0, 3.14, 0.0)
            >>> robot.motion.advanced.add_movel_waypoint(
            ...     tcp_pose=pose,
            ...     translation_speed=0.5,
            ...     rotation_speed=90.0,  # град/с
            ...     blend=0.02
            ... )

        Notes:
            - Все точки добавляются в **очередь** и выполняются после вызова
              `robot.motion.mode.set('move_adv')`.
            - Убедитесь, что единицы измерения углов в `tcp_pose` совпадают с
              `orientation_units`.
            - MoveL гарантирует **прямолинейную траекторию TCP**, но не гарантирует
              постоянную скорость движения.
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        translation_speed = translation_speed or motion_setup.linear_speed
        rotation_speed = rotation_speed or motion_setup.rotation_speed
        translation_accel = (
            translation_accel or motion_setup.linear_acceleration
        )
        rotation_accel = rotation_accel or motion_setup.rotation_acceleration
        blend = blend or motion_setup.blend

        Validation.literal("angle", orientation_units)
        Validation.length(tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose")
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.value(translation_speed, SPEED_LIMITS, "translation_speed")
        Validation.value(translation_accel, ACCEL_LIMITS, "translation_accel")
        Validation.value(
            rotation_speed,
            motion_setup.RPMP_ROTATION_SPEED_LIMITS,
            "rotation_speed",
        )
        Validation.value(
            rotation_accel,
            motion_setup.RPMP_ROTATION_ACCEL_LIMITS,
            "rotation_accel",
        )

        if orientation_units == "deg":
            rotation_speed = degrees_to_radians(rotation_speed)
            rotation_accel = degrees_to_radians(rotation_accel)

        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose,
                orientation_units=orientation_units,
            )
        tcp_pose = set_position_orientation_units(tcp_pose, orientation_units)

        cart_wp = RPMPCartesianWayPointTemplate(
            tr_position=tuple(tcp_pose[POSITION_SLICE]),
            rot_position=tuple(tcp_pose[ORIENTATION_SLICE]),
            tr_velocity=translation_speed,
            rot_velocity=rotation_speed,
            tr_acceleration=translation_accel,
            rot_acceleration=rotation_accel,
        )
        command = RPMPMoveCommandTemplate(
            type=RpmpWayPointType.movel,
            blend_radius=blend,
            wp=RPMPMoveLWayPointTemplate(cart_wp=cart_wp),
        )
        return self._add_waypoint("l", command)

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def add_movep_waypoint(
        self,
        tcp_pose: PositionOrientation,
        translation_speed: float | None = None,
        rotation_speed: float | None = None,
        translation_accel: float | None = None,
        rotation_accel: float | None = None,
        blend: float | None = None,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Добавляет точку процессного движения (MoveP) в буфер точек Advanced-движения.

        Добавить целевую точку типа 'process' для типа движения 'Advanced' в
        глобальной системе координат (система координат основания робота).
        Конвертация позиции и ориентации из локальной СК (пользовательской) в
        глобальную производится при передаче локальной системы координат в
        качестве аргумента.

        Args:
            tcp_pose: Позиция ЦТИ в формате (X, Y, Z, Rx, Ry, Rz),
                где (X, Y, Z) — м, (Rx, Ry,Rz) — 'orientation_units'.
            translation_speed: Желаемая скорость поступательного перемещения
                точки по траектории (0 - 3 м/с).
            translation_accel: Желаемое ускорение поступательного перемещения
                точки по траектории (0 - 15 м/c^2).
            rotation_speed: Желаемая скорость вращательного перемещения
                точки ('orientation_units'/c) (0 - 360 deg/с / 0 - 6.28 rad/с).
            rotation_accel: Желаемое ускорение вращательного перемещения
                точки ('orientation_units'/c^2)(0 - 720 deg/с^2 / 0 - 12.56 rad/с^2).
            blend: Радиус сглаживания движения (м).
                (Радиус вокруг точки, при пересечении которого траекторией
                движения робота начинается/заканчивается сглаживание).
            orientation_units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задается точка. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Добавить MoveL-точку
            >>> pose = (0.4, 0.0, 0.6, 0.0, 3.14, 0.0)
            >>> robot.motion.advanced.add_movep_waypoint(
            ...     tcp_pose=pose,
            ...     translation_speed=0.5,
            ...     rotation_speed=90.0,  # град/с
            ...     blend=0.02
            ... )

        Notes:
            - Все точки добавляются в **очередь** и выполняются после вызова
              `robot.motion.mode.set('move_adv')`.
            - Убедитесь, что единицы измерения углов в `tcp_pose` совпадают с
              `orientation_units`.
            - MoveP гарантирует прямолинейную траекторию TCP и постоянную скорость
                движения.
            - Задаваемые скорости и ускорения (за исключением скорости
                поступательного движения) являются желаемыми и могут
                автоматически уменьшаться во время реализации траектории
                (но не увеличиваться).
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        translation_speed = translation_speed or motion_setup.linear_speed
        rotation_speed = rotation_accel or motion_setup.rotation_speed
        translation_accel = (
            translation_accel or motion_setup.linear_acceleration
        )
        rotation_accel = rotation_accel or motion_setup.rotation_acceleration
        blend = blend or motion_setup.blend

        Validation.literal("angle", orientation_units)
        Validation.length(tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose")
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.value(translation_speed, SPEED_LIMITS, "translation_speed")
        Validation.value(translation_accel, ACCEL_LIMITS, "translation_accel")
        Validation.value(
            rotation_speed,
            motion_setup.RPMP_ROTATION_SPEED_LIMITS,
            "rotation_speed",
        )
        Validation.value(
            rotation_accel,
            motion_setup.RPMP_ROTATION_ACCEL_LIMITS,
            "rotation_accel",
        )

        if orientation_units == "deg":
            rotation_speed = degrees_to_radians(rotation_speed)
            rotation_accel = degrees_to_radians(rotation_accel)

        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose,
                orientation_units=orientation_units,
            )
        tcp_pose = set_position_orientation_units(tcp_pose, orientation_units)

        cart_wp = RPMPCartesianWayPointTemplate(
            tr_position=tuple(tcp_pose[POSITION_SLICE]),
            rot_position=tuple(tcp_pose[ORIENTATION_SLICE]),
            tr_velocity=translation_speed,
            rot_velocity=rotation_speed,
            tr_acceleration=translation_accel,
            rot_acceleration=rotation_accel,
        )

        command = RPMPMoveCommandTemplate(
            type=RpmpWayPointType.movep,
            blend_radius=blend,
            wp=RPMPMovePWayPointTemplate(cart_wp=cart_wp),
        )
        return self._add_waypoint("p", command)

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def add_movec_waypoint(
        self,
        tcp_pose_1: PositionOrientation,
        tcp_pose_2: PositionOrientation,
        translation_speed: float | None = None,
        rotation_speed: float | None = None,
        translation_accel: float | None = None,
        rotation_accel: float | None = None,
        blend: float | None = None,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Добавляет сегмент круговой траектории (MoveC) в буфер Advanced-движения.

        MoveC (Circular Move) определяет **дугу**, проходящую через три точки:
        1. Текущая позиция робота (перед вызовом метода);
        2. `tcp_pose_1` — промежуточная точка на дуге;
        3. `tcp_pose_2` — конечная точка дуги.

        Таким образом, один вызов `add_movec_waypoint` добавляет **одну дугу**
        (две новые точки), а не одну точку. Траектория строится в декартовом
        пространстве с сохранением постоянной ориентации или её плавным изменением.

        Конвертация позиции и ориентации из локальной СК (пользовательской) в
        глобальную производится при передаче локальной системы координат в
        качестве аргумента.

        Args:
            tcp_pose_1: Первая позиция ЦТИ в формате (X, Y, Z, Rx, Ry, Rz),
                где (X, Y, Z) — м, (Rx, Ry,Rz) — 'orientation_units'.
            tcp_pose_2: Вторая позиция ЦТИ в формате (X, Y, Z, Rx, Ry, Rz),
                где (X, Y, Z) — м, (Rx, Ry,Rz) — 'orientation_units'.
            translation_speed: Желаемая скорость поступательного перемещения
                точки по траектории (0 - 3 м/с).
            translation_accel: Желаемое ускорение поступательного перемещения
                точки по траектории (0 - 15 м/c^2).
            rotation_speed: Желаемая скорость вращательного перемещения
                точки ('orientation_units'/c) (0 - 360 deg/с / 0 - 6.28 rad/с).
            rotation_accel: Желаемое ускорение вращательного перемещения
                точки ('orientation_units'/c^2)(0 - 720 deg/с^2 / 0 - 12.56 rad/с^2).
            blend: Радиус сглаживания движения (м).
                (Радиус вокруг точки, при пересечении которого траекторией
                движения робота начинается/заканчивается сглаживание).
            orientation_units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задаются точки. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Выполнить полукруг в плоскости XY
            >>> start = robot.motion.linear.get_actual_position()
            >>> robot.motion.advanced.add_movec_waypoint(start)
            >>> mid = (0.3, 0.1, 0.5, 0, 3.14, 0)    # промежуточная
            >>> end = (0.3, 0.2, 0.5, 0, 3.14, 0)    # конечная
            >>> robot.motion.advanced.add_movec_waypoint(mid, end, translation_speed=0.3)

        Notes:
            - Если три точки (текущая, `tcp_pose_1`, `tcp_pose_2`) коллинеарными
              — дуга вырождается в линию, происходит линейное движение.
            - Если две из трех точек совпадают — дуга вырождается в линию,
                происходит линейное движение.
            - После добавления сегмента движение запускается через
              `robot.motion.mode.set('move_adv')`.
            - Убедитесь, что единицы измерения углов в обеих позициях совпадают
              с `orientation_units`.
            - Задаваемые скорости и ускорения являются желаемыми и могут
                автоматически уменьшаться во время реализации траектории
                (но не увеличиваться)
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        translation_speed = translation_speed or motion_setup.linear_speed
        rotation_speed = rotation_speed or motion_setup.rotation_speed
        translation_accel = (
            translation_accel or motion_setup.linear_acceleration
        )
        rotation_accel = rotation_accel or motion_setup.rotation_acceleration
        blend = blend or motion_setup.blend

        Validation.literal("angle", orientation_units)
        Validation.length(
            tcp_pose_1, POSITION_ORIENTATION_LENGTH, "tcp_pose_1"
        )
        Validation.length(
            tcp_pose_2, POSITION_ORIENTATION_LENGTH, "tcp_pose_2"
        )
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.value(translation_speed, SPEED_LIMITS, "translation_speed")
        Validation.value(translation_accel, ACCEL_LIMITS, "translation_accel")
        Validation.value(
            rotation_speed,
            motion_setup.RPMP_ROTATION_SPEED_LIMITS,
            "rotation_speed",
        )
        Validation.value(
            rotation_accel,
            motion_setup.RPMP_ROTATION_ACCEL_LIMITS,
            "rotation_accel",
        )

        if orientation_units == "deg":
            rotation_speed = degrees_to_radians(rotation_speed)
            rotation_accel = degrees_to_radians(rotation_accel)

        if coordinate_system:
            tcp_pose_1 = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose_1,
                orientation_units=orientation_units,
            )
            tcp_pose_2 = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose_2,
                orientation_units=orientation_units,
            )
        tcp_pose_1 = set_position_orientation_units(
            tcp_pose_1, orientation_units
        )
        tcp_pose_2 = set_position_orientation_units(
            tcp_pose_2, orientation_units
        )

        cart_wp_1 = RPMPCartesianWayPointTemplate(
            tr_position=tuple(tcp_pose_1[POSITION_SLICE]),
            rot_position=tuple(tcp_pose_1[ORIENTATION_SLICE]),
            tr_velocity=translation_speed,
            rot_velocity=rotation_speed,
            tr_acceleration=translation_accel,
            rot_acceleration=rotation_accel,
        )
        cart_wp_2 = RPMPCartesianWayPointTemplate(
            tr_position=tuple(tcp_pose_2[POSITION_SLICE]),
            rot_position=tuple(tcp_pose_2[ORIENTATION_SLICE]),
            tr_velocity=translation_speed,
            rot_velocity=rotation_speed,
            tr_acceleration=translation_accel,
            rot_acceleration=rotation_accel,
        )

        command = RPMPMoveCommandTemplate(
            type=RpmpWayPointType.movec,
            blend_radius=blend,
            wp=RPMPMoveCWayPointTemplate(
                cart_wp_1=cart_wp_1, cart_wp_2=cart_wp_2
            ),
        )
        return self._add_waypoint("c", command, num_of_points=2)

    @handle_connection(available_in_read_only=False)
    def add_movej_waypoint(
        self,
        joints_pose: PositionOrientation,
        joints_speed: PositionOrientation | float | None = None,
        joints_accel: PositionOrientation | float | None = None,
        blend: float = 0.0,
        orientation_units: AngleUnits | None = None,
    ) -> bool:
        """Добавляет точку совместного (joint-space) движения (MoveJ) в буфер Advanced-движения.

        MoveJ (Joint Move) определяет **перемещение в пространстве суставов** от текущей
        конфигурации робота к целевой позиции `joints_pose`. В отличие от MoveL/MoveC,
        траектория **не является прямой в декартовом пространстве** — она оптимизирована
        для плавного изменения углов поворота каждого сустава.

        Каждый вызов добавляет **одну новую точку** в буфер. Движение между точками
        выполняется с заданными скоростями и ускорениями по каждому суставу независимо.

        Args:
            joints_pose: Целевые углы поворота шести суставов робота
                (от основания к фланцу) в порядке `[J1, J2, J3, J4, J5, J6]`.
                Единицы измерения — `orientation_units` (по умолчанию градусы).
            joints_speed: Желаемые скорости поворота суставов
                (`orientation_units`/с). По умолчанию используются глобальные настройки.
                Диапазон: 0–180°/с (0–3.14 рад/с).
            joints_accel: Желаемые ускорения поворота суставов
                (`orientation_units`/с²). По умолчанию — глобальные настройки.
                Диапазон: 0–1500°/с² (0–26.18 рад/с²).
            blend: Радиус сглаживания движения (м).
                Определяет радиус вокруг точки, при пересечении которого начинается
                плавный переход к следующему сегменту траектории.
            orientation_units: Единицы измерения углов. По умолчанию — градусы.
                - `'deg'` — градусы,
                - `'rad'` — радианы.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Переместить робота в "нулевую" конфигурацию
            >>> zero_config = [0, -90, 90, 0, 90, 0]  # в градусах
            >>> robot.motion.advanced.add_movej_waypoint(zero_config, blend=0.05)

        Notes:
            - MoveJ **не гарантирует прямолинейность** траектории TCP — используйте
              MoveL/MoveC, если важна форма пути в рабочем пространстве.
            - Все значения (`joints_pose`, `joints_speed`, `joints_accel`) должны
              содержать ровно 6 элементов — по одному на каждый сустав.
            - Углы интерпретируются в порядке: **J1 → J6** (от основания к фланцу).
            - После добавления точки движение запускается через
              `robot.motion.mode.set('move_adv')`.
            - Задаваемые скорости и ускорения являются желаемыми и могут
              автоматически уменьшаться во время реализации траектории (но не увеличиваться).
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        joints_speed = joints_speed or motion_setup.joints_speed
        joints_accel = joints_accel or motion_setup.joints_accel
        blend = blend or motion_setup.blend

        if isinstance(joints_speed, (float, int)):
            joints_speed = (joints_speed,) * POSITION_ORIENTATION_LENGTH
        if isinstance(joints_accel, (float, int)):
            joints_accel = (joints_accel,) * POSITION_ORIENTATION_LENGTH

        Validation.literal("angle", orientation_units)
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.length(
            joints_pose, POSITION_ORIENTATION_LENGTH, "joints_pose"
        )
        Validation.length(
            joints_speed, POSITION_ORIENTATION_LENGTH, "joints_speed"
        )
        Validation.length(
            joints_accel, POSITION_ORIENTATION_LENGTH, "joints_accel"
        )
        for speed in joints_speed:
            Validation.value(
                speed, motion_setup.JOINT_SPEED_LIMITS, "joints_speed"
            )
        for acceleration in joints_accel:
            Validation.value(
                acceleration, motion_setup.JOINT_ACCEL_LIMITS, "joints_accel"
            )

        if orientation_units == "deg":
            joints_pose = degrees_to_radians(joints_pose)
            joints_speed = degrees_to_radians(joints_speed)
            joints_accel = degrees_to_radians(joints_accel)

        joint_wp = RPMPJointWayPointTemplate(
            q=tuple(joints_pose),
            qd=tuple(joints_speed),
            qdd=tuple(joints_accel),
        )
        command = RPMPMoveCommandTemplate(
            type=RpmpWayPointType.movej,
            blend_radius=blend,
            wp=RPMPMoveJWayPointTemplate(joint_wp=joint_wp),
        )
        return self._add_waypoint("j", command)

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def add_movej_tcp_waypoint(
        self,
        tcp_pose: PositionOrientation,
        init_joints_pose: PositionOrientation | None = None,
        joints_speed: PositionOrientation | float | None = None,
        joints_accel: PositionOrientation | float | None = None,
        ik_solution_id: IkSolutionId = -1,
        blend: float = 0.0,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Добавляет точку совместного движения (MoveJ) с заданием цели в декартовых координатах.

        MoveJ-TCP — гибридный режим:
        - **Цель задаётся в декартовом пространстве** (`tcp_pose`),
        - **Траектория планируется в пространстве суставов**, как в `add_movej_waypoint`.

        Для преобразования декартовой цели в конфигурацию суставов используется **обратная
        кинематика (ОЗК)**. Параметр `init_joints_pose` задаёт начальное приближение для
        решения ОЗК, а `ik_solution_id` позволяет выбрать одно из возможных решений
        (например, «локтём вверх» или «локтём вниз»).

        Этот метод полезен, когда:
        - вы знаете желаемую позицию TCP, но хотите контролировать траекторию через суставы,
        - необходимо избежать сингулярностей, явно указав начальную конфигурацию.

        Args:
            tcp_pose: Целевая позиция ЦТИ в формате (X, Y, Z, Rx, Ry, Rz),
                где (X, Y, Z) — м, (Rx, Ry, Rz) — углы в `orientation_units`.
            init_joints_pose: Начальное приближение для решения ОЗК — углы шести суставов
                [J1, J2, J3, J4, J5, J6] в `orientation_units`.
                Используется как отправная точка для поиска ближайшего допустимого решения.
            joints_speed: Желаемые скорости поворота суставов
                (`orientation_units`/с). По умолчанию — глобальные настройки.
                Диапазон: 0–180°/с (0–3.14 рад/с).
            joints_accel: Желаемые ускорения поворота суставов
                (`orientation_units`/с²). По умолчанию — глобальные настройки.
                Диапазон: 0–1500°/с² (0–26.18 рад/с²).
            ik_solution_id: Идентификатор желаемого решения ОЗК.
                - `-1` — автоматически выбрать решение, ближайшее к `init_joints_pose`,
                - `0`, `1`, ... , `7` — конкретное решение.
            blend: Радиус сглаживания движения (м).
                Определяет радиус вокруг точки, при пересечении которого начинается
                плавный переход к следующему сегменту.
            orientation_units: Единицы измерения углов. По умолчанию — градусы.
                - `'deg'` — градусы,
                - `'rad'` — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задана `tcp_pose`. По умолчанию — глобальная система основания.
                Для локального смещения (например, относительно текущего TCP) передайте
                пользовательскую СК.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Переместить TCP в точку (0.4, 0, 0.5), используя текущую конфигурацию как приближение
            >>> current_joints = robot.motion.joint.get_actual_position()
            >>> target_tcp = (0.4, 0.0, 0.5, 0, 3.14, 0)
            >>> robot.motion.advanced.add_movej_tcp_waypoint(
            ...     tcp_pose=target_tcp,
            ...     init_joints_pose=current_joints,
            ...     blend=0.05
            ... )

        Notes:
            - В отличие от `add_movel_waypoint`, траектория **не будет прямой** в рабочем
              пространстве — она оптимизирована для суставов.
            - Если ОЗК не имеет решения для заданной `tcp_pose` и `init_joints_pose` —
              команда завершится ошибкой.
            - Все массивы (`init_joints_pose`, `joints_speed`, `joints_accel`) должны
              содержать ровно 6 элементов — по одному на сустав (J1 → J6).
            - После добавления точки движение запускается через
              `robot.motion.mode.set('move_adv')`.
            - Углы в `tcp_pose` интерпретируются в соответствии с `orientation_units`
              и преобразуются в радианы перед отправкой.
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        init_joints_pose = init_joints_pose or motion_setup.movej_init_pose
        joints_speed = joints_speed or motion_setup.joints_speed
        joints_accel = joints_accel or motion_setup.joints_accel
        blend = blend or motion_setup.blend

        if isinstance(joints_speed, (float, int)):
            joints_speed = (joints_speed,) * POSITION_ORIENTATION_LENGTH
        if isinstance(joints_accel, (float, int)):
            joints_accel = (joints_accel,) * POSITION_ORIENTATION_LENGTH

        Validation.literal("ik_id", ik_solution_id)
        Validation.literal("angle", orientation_units)
        Validation.value(blend, BLEND_LIMITS)
        Validation.length(
            init_joints_pose, POSITION_ORIENTATION_LENGTH, "init_joints_pose"
        )
        Validation.length(
            joints_speed, POSITION_ORIENTATION_LENGTH, "joints_speed"
        )
        Validation.length(
            joints_accel, POSITION_ORIENTATION_LENGTH, "joints_accel"
        )
        Validation.length(tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose")
        for speed in joints_speed:
            Validation.value(
                speed, motion_setup.JOINT_SPEED_LIMITS, "joints_speed"
            )
        for acceleration in joints_accel:
            Validation.value(
                acceleration, motion_setup.JOINT_ACCEL_LIMITS, "joints_accel"
            )

        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose,
                orientation_units=orientation_units,
            )
        tcp_pose = set_position_orientation_units(tcp_pose, orientation_units)

        if orientation_units == "deg":
            init_joints_pose = degrees_to_radians(init_joints_pose)
            joints_speed = degrees_to_radians(joints_speed)
            joints_accel = degrees_to_radians(joints_accel)

        joint_tcp_wp = RPMPJointTcpWayPointTemplate(
            tr_position=tuple(tcp_pose[POSITION_SLICE]),
            rot_position=tuple(tcp_pose[ORIENTATION_SLICE]),
            init_q=tuple(init_joints_pose),
            ik_solution=ik_solution_id,
            qd=tuple(joints_speed),
            qdd=tuple(joints_accel),
        )
        command = RPMPMoveCommandTemplate(
            type=RpmpWayPointType.movej_tcp,
            blend_radius=blend,
            wp=RPMPMoveJTcpWayPointTemplate(joint_wp=joint_tcp_wp),
        )
        return self._add_waypoint("j_tcp", command)

    def _add_waypoint(
        self,
        variant: str,
        command_template: RPMPMoveCommandTemplate,
        num_of_points: int = 1,
    ) -> bool:
        """
        Вспомогательная команда отправки сообщения добавления новой 'RPMP' точки.

        Returns:
            True: В случае успешного добавления точки.
        """
        self._waypoint_counter = int(self._rtd_receiver.get_data().wp_cntr)
        self._waypoint_counter = (
            self._waypoint_counter + num_of_points
        ) & WP_COUNTER_MAX_VALUE

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_RPMP_MOVE_ADD_WP,
            command_template,
            variant=variant,
        )

        if response[0] != AddWayPointErrorCode.success:
            self._waypoint_counter = None
            error_description = ADD_WAY_POINT_ERROR_DESCRIPTION.get(
                response[0], None
            )
            if error_description is None:
                self._write_log(
                    "error",
                    f"Received unknown add advanced waypoint error status: {response[0]}",
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
