from __future__ import annotations

from typing import List, cast

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.core.network import (
    Controller,
    RTDReceiver,
)
from api.robot_api_rc5v15.API.source.features.decorators import transforms_coordinates
from api.robot_api_rc5v15.API.source.features.mathematics.coordinate_system import (
    convert_position_orientation,
)
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    degrees_to_radians,
    radians_to_degrees,
)
from api.robot_api_rc5v15.API.source.features.tools import (
    normalize_joint_angles,
    set_position_orientation_units,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    InverseKinematicOptimalTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    FKINE_IKINE_RESPONSE_JOINT_POSITION_SLICE,
    FKINE_SOLUTIONS_COUNT,
    JOINTS_COUNT,
    ORIENTATION_SLICE,
    POSITION_ORIENTATION_LENGTH,
    POSITION_SLICE,
    PREVIOUS_FKINE_SOLUTION_OFFSET,
    RESPONSE_CODE_OFFSET,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    PositionOrientation,
)

from .coordinate_system import CoordinateSystem


class Kinematics:
    """
    Класс для получения решения прямой и обратной задач кинематики.
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        connection_state: ConnectionState,
    ) -> None:
        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._connection_state = connection_state

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def get_forward(
        self,
        angle_pose: PositionOrientation,
        units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> PositionOrientation | None:
        """Решает прямую задачу кинематики (Forward Kinematics).

        Получает решение прямой задачи кинематики в пользовательской системе
        координат. Если система координат не была задана, то будет
        использована система координат основания робота.

        Args:
            joints_angles: 6 углов поворота моторов, от основания до фланца
                робота ('units').
            units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
            coordinate_system: Выбранная система координат. По-умолчанию
                используется система координат основания робота.

        Returns:
            PositionOrientation | None:
                - `(X, Y, Z, Rx, Ry, Rz)` — позиция TCP в метрах и углах
                  в указанной системе координат;
                - `None`, если не удалось получить решение.

        Examples:
            >>> # Получить TCP-позицию для заданных углов (в глобальной СК)
            >>> angles = [10.0, -50.0, 0.0, 0.0, 0.0, 0.0]
            >>> tcp_pose = robot.motion.kinematics.get_forward(angle_pose=angles)
            >>> if tcp_pose:
            ...     x, y, z, rx, ry, rz = tcp_pose
            ...     print(f"TCP: X={x:.3f} м, Y={y:.3f} м, Z={z:.3f} м")

            >>> # Получить позицию в пользовательской СК
            >>> from API.coords import CoordinateSystem
            >>> user_cs = CoordinateSystem((0.5, 0, 0.2, 0, 0, 0))
            >>> tcp_local = robot.motion.kinematics.get_forward(
            ...     angle_pose=angles,
            ...     coordinate_system=user_cs
            ... )
        """

        units = units or MOTION_SETUP.units
        Validation.literal("angle", units)
        Validation.length(angle_pose, JOINTS_COUNT, "angle_pose")
        if units == "deg":
            angle_pose = degrees_to_radians(angle_pose)

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_FKINE, angle_pose
        )
        tcp_pose = list(response[FKINE_IKINE_RESPONSE_JOINT_POSITION_SLICE])

        if not response and response[0] != 0:
            return None
        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system,
                tcp_pose,
                orientation_units="rad",
                to_local=True,
            )
        tcp_pose = cast(List[float], tcp_pose)
        if units == "rad":
            return tcp_pose
        return tcp_pose[POSITION_SLICE] + radians_to_degrees(
            tcp_pose[ORIENTATION_SLICE]
        )

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def get_inverse(
        self,
        tcp_pose: PositionOrientation,
        angle_pose: PositionOrientation | None = None,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
        get_all: bool = False,
    ) -> PositionOrientation | tuple[PositionOrientation, ...] | None:
        """Решает обратную задачу кинематики (Inverse Kinematics).

        Получает решение обратной задачи кинематики. Конвертация позиции и
        ориентации из локальной СК (пользовательской) в глобальную производится
        при передаче локальной системы координат в качестве аргумента.

        Args:
            tcp_pose: Позиция и ориентация ЦТИ в глобальной
                системе координат (система координат основания робота) в
                формате:
                (X, Y, Z, Rx, Ry, Rz), где (X, Y, Z) — м,
                (Rx, Ry,Rz) — 'orientation_units'.
            angle_pose: Положение углов поворота моторов, относительно которого
                будет рассчитано ближайшее решение обратной задачи кинематики.
                По-умолчанию текущее значение, полученное из RTD.
            orientation_units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задается точка. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.
            get_all: Получить ли все решения или только оптимальное.

        Returns:
            PositionOrientation: Оптимальное решение задачи.
            Tuple[PositionOrientation, ...]: 8 решений задачи в формате 6
                углов поворотов моторов, от основания до фланца робота
                ('units').
            None: В случае ошибки в расчетах в контроллере.

        Examples:
            >>> # Получить одно решение, ближайшее к текущей позиции
            >>> target = (0.4, 0.0, 0.5, 0.0, 3.14, 0.0)
            >>> angles = robot.motion.kinematics.get_inverse(tcp_pose=target)
            >>> if angles:
            ...     robot.motion.joint.add_new_waypoint(angle_pose=angles)

            >>> # Получить все возможные решения
            >>> all_solutions = robot.motion.kinematics.get_inverse(
            ...     tcp_pose=target,
            ...     get_all=True
            ... )
            >>> if all_solutions:
            ...     # Выбрать решение с наименьшим сгибом локтя
            ...     best = min(all_solutions, key=lambda j: abs(j[2]))
            ...     robot.motion.joint.add_new_waypoint(angle_pose=best)

            >>> # Указать опорную позицию для выбора решения
            >>> reference = (0, -90, 0, -90, 0, 0)
            >>> angles = robot.motion.kinematics.get_inverse(
            ...     tcp_pose=target,
            ...     angle_pose=reference,
            ...     orientation_units='deg'
            ... )

        Notes:
            - Обратная задача может иметь **до 8 решений** для стандартного
              6-осного манипулятора.
            - Если `angle_pose` не задан, в качестве опорной используется
              **текущая позиция робота** (из RTD).
            - Все решения возвращаются в **тех же единицах**, что указаны
              в `orientation_units`.
            - Этот метод **недоступен в режиме read-only**.
        """
        orientation_units = orientation_units or MOTION_SETUP.units
        Validation.literal("angle", orientation_units)
        Validation.length(tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose")
        if angle_pose is None:
            angle_pose = self._rtd_receiver.get_data().act_q
        else:
            Validation.length(
                angle_pose, POSITION_ORIENTATION_LENGTH, "angle_pose"
            )
            if orientation_units == "deg":
                angle_pose = degrees_to_radians(angle_pose)
        angle_pose = normalize_joint_angles(angle_pose, "rad")
        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose,
                orientation_units=orientation_units,
            )
        tcp_pose = set_position_orientation_units(tcp_pose, orientation_units)
        if get_all:
            response = self._controller.request(
                ControllerCommands.CTRLR_COMS_IKINE, tcp_pose
            )

            solutions = tuple(
                normalize_joint_angles(
                    response[
                        i * POSITION_ORIENTATION_LENGTH
                        + RESPONSE_CODE_OFFSET : i
                        * POSITION_ORIENTATION_LENGTH
                        + PREVIOUS_FKINE_SOLUTION_OFFSET
                    ],
                    "rad",
                )
                for i in range(FKINE_SOLUTIONS_COUNT)
            )
            if orientation_units == "deg":
                return tuple(
                    tuple(radians_to_degrees(solution))
                    for solution in solutions
                )
            return solutions
        ikine_template = InverseKinematicOptimalTemplate(
            target=list(tcp_pose), base_q=list(angle_pose)
        )
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_IKINE_OPTIMAL, ikine_template
        )
        inverse_solution = normalize_joint_angles(
            response[FKINE_IKINE_RESPONSE_JOINT_POSITION_SLICE], "rad"
        )
        if response and response[0] == 0:
            if orientation_units == "deg":
                inverse_solution = radians_to_degrees(inverse_solution)
            return inverse_solution
        return None
