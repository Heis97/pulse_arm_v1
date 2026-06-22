from __future__ import annotations

from typing import TYPE_CHECKING

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
    literal_to_int,
    normalize_joint_angles,
    set_position_orientation_units,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    JointJogCommandTemplate,
    MoveCommandTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
    JointJogModes,
    WayPointType,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    BLEND_LIMITS,
    JOINTS_COUNT,
    POSITION_ORIENTATION_LENGTH,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    JogDirection,
    JointIndex,
    PositionOrientation,
)

if TYPE_CHECKING:
    from .coordinate_system import CoordinateSystem
    from .motion_host import Motion


class JointMotion:
    """
    Класс для работы с моторным типом движения.
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver

    def __init__(
        self,
        controller: Controller,
        rtd_receiver: RTDReceiver,
        motion_host: Motion,
        connection_state: ConnectionState,
    ) -> None:
        self._controller = controller
        self._rtd_receiver = rtd_receiver
        self._motion_host = motion_host
        self._connection_state = connection_state

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def add_new_waypoint(
        self,
        angle_pose: PositionOrientation | None = None,
        tcp_pose: PositionOrientation | None = None,
        speed: float | None = None,
        accel: float | None = None,
        blend: float | None = None,
        units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Добавляет точку движения по сочленениям (Joint) в буфер точек.

        Метод позволяет задать целевую конфигурацию робота одним из трёх способов:
        1. **Только углы сочленений** (`angle_pose`) — прямое указание позиции;
        2. **Только TCP-позиция** (`tcp_pose`) — система решит обратную задачу
           кинематики и вычислит углы;
        3. **Оба параметра** — используется `tcp_pose`, а `angle_pose` позволяет
            указать желаемое положение робота в этой точке.

        Результирующее движение выполняется по кратчайшей траектории в пространстве
        сочленений с учётом заданных скорости и ускорения.

        Args:
            angle_pose (PositionOrientation, optional): Углы сочленений
                `(J1, J2, J3, J4, J5, J6)` в указанных единицах (`units`).
            tcp_pose (PositionOrientation, optional): Позиция TCP
                `(X, Y, Z, Rx, Ry, Rz)`, где линейные компоненты — в метрах,
                угловые — в тех же единицах, что и `units`.
            speed (float, optional): Скорость сочленений:
                - в градусах: 0–180 °/с;
                - в радианах: 0–3.14 рад/с.
                По умолчанию — из глобальной конфигурации (`set_motion_config`).
            accel (float, optional): Ускорение сочленений:
                - в градусах: 0–1500 °/с²;
                - в радианах: 0–26.18 рад/с².
                По умолчанию — из глобальной конфигурации.
            blend (float, optional): Радиус сглаживания в метрах.
                При приближении к точке на расстояние ≤ `blend` робот плавно
                переходит к следующему сегменту. Значение `0` отключает сглаживание.
            units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задается TCP точка. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.

        Returns:
            True: В случае успешного добавления точки.

        Examples:
            >>> # Задать движение по углам
            >>> angles = (0.0, -90.0, 0.0, -90.0, 0.0, 0.0)
            >>> robot.motion.joint.add_new_waypoint(angle_pose=angles)

            >>> # Задать движение через TCP (углы будут вычислены автоматически)
            >>> pose = (0.3, 0.0, 0.5, 0.0, 3.14, 0.0)
            >>> robot.motion.joint.add_new_waypoint(tcp_pose=pose, units='rad')

        Notes:
            - Если указан только `tcp_pose`, решается **обратная задача кинематики**.
              При неоднозначности (несколько решений) выбирается ближайшее
              к текущей конфигурации.
            - Все углы должны быть в **одних и тех же единицах**, что указано в `units`.
            - Добавление точки **не запускает движение** — требуется явный запуск
              через `robot.motion.mode.set('move')`.
        """
        units = units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(units)
        speed = speed or motion_setup.joint_speed
        accel = accel or motion_setup.joint_acceleration
        blend = blend or motion_setup.blend

        Validation.literal("angle", units)
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.value(speed, motion_setup.JOINT_SPEED_LIMITS, "speed")
        Validation.value(accel, motion_setup.JOINT_ACCEL_LIMITS, "accel")
        if angle_pose is not None:
            Validation.length(
                angle_pose, POSITION_ORIENTATION_LENGTH, "angle_pose"
            )

        if units == "deg":
            speed = degrees_to_radians(speed)
            accel = degrees_to_radians(accel)
            if angle_pose is not None:
                angle_pose = degrees_to_radians(angle_pose)

        if angle_pose is not None:
            angle_pose = normalize_joint_angles(angle_pose, "rad")

        if angle_pose is not None and tcp_pose is not None:
            Validation.length(
                tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose"
            )
            if coordinate_system:
                tcp_pose = convert_position_orientation(
                    coordinate_system=coordinate_system,
                    position_orientation=tcp_pose,
                    orientation_units=units,
                )
            tcp_pose = set_position_orientation_units(tcp_pose, units)
            command = MoveCommandTemplate(
                t=WayPointType.tcp_pose,
                des_x=tcp_pose,
                des_q=angle_pose,
                v_max_j=speed,
                a_max_j=accel,
                r_blend=blend,
            )
        elif tcp_pose is not None:
            Validation.length(
                tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose"
            )
            if coordinate_system:
                tcp_pose = convert_position_orientation(
                    coordinate_system=coordinate_system,
                    position_orientation=tcp_pose,
                    orientation_units=units,
                )
            tcp_pose = set_position_orientation_units(tcp_pose, units)
            command = MoveCommandTemplate(
                t=WayPointType.tcp_pose,
                des_x=tcp_pose,
                des_q=normalize_joint_angles(
                    self._rtd_receiver.get_data().act_q, "rad"
                ),
                v_max_j=speed,
                a_max_j=accel,
                r_blend=blend,
            )
        elif angle_pose is not None:
            command = MoveCommandTemplate(
                t=WayPointType.joint,
                des_q=angle_pose,
                v_max_j=speed,
                a_max_j=accel,
                r_blend=blend,
            )
        else:
            return False
        return self._motion_host._add_waypoint(command)

    @handle_connection(available_in_read_only=True)
    def get_actual_position(
        self, units: AngleUnits | None = None
    ) -> PositionOrientation:
        """Возвращает текущие углы сочленений робота (Joint-space).

        Метод предоставляет мгновенные значения углов поворота всех шести
        моторов (от основания до фланца) в указанной системе единиц.
        Результат отражает **фактическое физическое положение** робота
        на момент вызова.

        Метод доступен в режиме «read only».

        Args:
            units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.

        Returns:
            PositionOrientation: Кортеж из 6 чисел:
                `(J1, J2, J3, J4, J5, J6)`, где `Ji` — угол поворота i-го
                сочленения от основания к фланцу, в указанных единицах.

        Examples:
            >>> # Получить углы в градусах
            >>> joints = robot.motion.joint.get_actual_position()
            >>> print(f"Текущие углы: {joints}")

            >>> # Получить углы в радианах
            >>> joints_rad = robot.motion.joint.get_actual_position(units='rad')
        """

        units = units or MOTION_SETUP.units
        Validation.literal("angle", units)
        return self._motion_host.get_actual_position(units, "joints", None)

    @handle_connection(available_in_read_only=False)
    def get_last_saved_position(
        self, units: AngleUnits | None = None
    ) -> PositionOrientation | None:
        """Возвращает последнюю сохранённую позицию робота в формате углов сочленений.

        Args:
            units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.

        Returns:
            PositionOrientation | None:
                - Объект с 6 углами `(J1, J2, J3, J4, J5, J6)` в указанных единицах

        Examples:
            >>> # Получить последнюю сохранённую позицию (или текущую, если в 'run')
            >>> pos = robot.motion.joint.get_last_saved_position()
            >>> if pos is not None:
            ...     print(f"Сохраненные углы: {pos}")

        Notes:
            - Эта функция **не эквивалентна** `get_actual_position()` — только в
              состоянии 'run' они совпадают.
            - Используйте этот метод, например, для восстановления позиции
              после перезапуска или для сравнения с текущим положением
              в безопасном состоянии.
        """
        units = units or MOTION_SETUP.units
        Validation.literal("angle", units)
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_LAST_POS
        )
        if units == "deg":
            return radians_to_degrees(response)
        return response

    @handle_connection(available_in_read_only=False)
    def jog_once(
        self,
        joint_index: JointIndex,
        jog_direction: JogDirection,
    ) -> bool:
        """Выполняет кратковременный шаг джоггинга по указанному сочленению.

        Метод предназначен для ручного управления отдельными моторами робота
        в реальном времени. Это **один цикл управления** — для непрерывного вращения
        метод должен вызываться **циклически с частотой не менее 100 Гц** (каждые ≤10 мс).

        Args:
            joint_index (JointIndex): Индекс сочленения (мотора):
                - `0` — первое звено от основания (J1),
                - `1` — второе звено (J2),
                - ...
                - `5` — последнее звено (J6).
            jog_direction (JogDirection): Направление вращения:
                - `'+'` — по часовой стрелке (в системе отсчёта сочленения);
                - `'-'` — против часовой стрелки.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Непрерывное вращение J2 по часовой стрелке в течение 1.5 сек
            >>> import time
            >>> start = time.time()
            >>> while time.time() - start < 1.5:
            ...     robot.motion.joint.jog_once(joint_index=1, jog_direction='+')
            ...     time.sleep(0.005)  # 200 Гц — надёжная частота

        Notes:
            - Направление `'+'`/`'-'` определяется направлением вращения часовой стрелки.
            - Скорость и ускорение джоггинга можно настроить с помощью scale_setup.
        """

        Validation.index(joint_index, range(JOINTS_COUNT))
        Validation.literal("math", jog_direction)
        jog_template = JointJogCommandTemplate()
        jog_template.joints_rotation_directions[joint_index] = literal_to_int(
            jog_direction
        )
        jog_template.mode = JointJogModes.on
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_JOINT_JOG, jog_template
        )
