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
from api.robot_api_rc5v15.API.source.features.tools import (
    literal_to_int,
    set_position_orientation_units,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.command_templates import (
    JogCommandParametersTemplate,
    JogCommandTemplate,
    MoveCommandTemplate,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
    JogModes,
    WayPointType,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import (
    JogParamInTCP,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    ACCEL_LIMITS,
    BLEND_LIMITS,
    POSITION_ORIENTATION_LENGTH,
    SPEED_LIMITS,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    JogAxis,
    JogDirection,
    PositionOrientation,
    ReferenceFrame,
)

if TYPE_CHECKING:
    from .coordinate_system import CoordinateSystem
    from .motion_host import Motion


class LinearMotion:
    """
    Класс для работы с линейным (декартовым) типом движения в формате
    (X, Y, Z, Rx, Ry, Rz), где (X, Y, Z) — м, (Rx, Ry,Rz) — углы поворотов
    (рад / град).
    """

    _controller: Controller
    _rtd_receiver: RTDReceiver
    _motion_host: Motion

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
        tcp_pose: PositionOrientation,
        speed: float | None = None,
        accel: float | None = None,
        blend: float | None = None,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> bool:
        """Добавляет точку линейного движения (Linear) в буфер точек.

        Добавить целевую точку типа движения 'Linear' в глобальной системе
        координат (система координат основания робота). Конвертация позиции и
        ориентации из локальной СК (пользовательской) в глобальную производится
        при передаче локальной системы координат в качестве аргумента.

        Args:
            tcp_pose (PositionOrientation): Целевая позиция TCP в формате
                `(X, Y, Z, Rx, Ry, Rz)`, где:
                - `(X, Y, Z)` — координаты в метрах;
                - `(Rx, Ry, Rz)` — углы поворота в указанных единицах
                  (`orientation_units`).
            speed (float, optional): Линейная скорость движения TCP, м/с.
                Диапазон: 0–3 м/с. Если не задано — используется значение по умолчанию
                из глобальной конфигурации.
            accel (float, optional): Линейное ускорение TCP, м/с².
                Диапазон: 0–15 м/с². По умолчанию — из глобальной конфигурации.
            blend (float, optional): Радиус сглаживания в метрах.
                При приближении к точке на расстояние ≤ `blend` робот плавно
                переходит к следующему сегменту траектории без остановки.
                Значение `0` отключает сглаживание.
            orientation_units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой задается точка. По умолчанию используется глобальная
                система координат основания. Для локального смещения (например,
                вдоль TCP) передайте соответствующую пользовательскую СК.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Добавить точку в глобальной СК
            >>> pose = (0.3, 0.0, 0.5, 0.0, 3.14, 0.0)  # (X, Y, Z, Rx, Ry, Rz)
            >>> robot.motion.linear.add_new_waypoint(pose, speed=0.5, blend=0.05)

            >>> # Использовать пользовательскую СК
            >>> from API.coords import CoordinateSystem
            >>> local_coord_system = CoordinateSystem(
            ...     position_orientation=((-0.3332, -0.1838, -0.0198, 3.138, 0, 0.8195)),
            ...     orientation_units="deg",
            ... )
            >>> robot.motion.linear.add_new_waypoint(
            ...     pose,
            ...     orientation_units='deg',
            ...     coordinate_system=local_coord_system,
            ... )

        Notes:
            - Все точки добавляются в **очередь** и выполняются последовательно
              после вызова `robot.motion.mode.set('move')`.
            - Убедитесь, что единицы измерения углов в `tcp_pose` совпадают с
              `orientation_units` — иначе ориентация будет интерпретирована неверно.
            - Добавление точек **не запускает движение** — требуется явный запуск
              через `robot.motion.mode.set('move')`.
        """

        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        speed = speed or motion_setup.linear_speed
        accel = accel or motion_setup.linear_acceleration
        blend = blend or motion_setup.blend

        Validation.literal("angle", orientation_units)
        Validation.length(tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose")
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.value(speed, SPEED_LIMITS, "speed")
        Validation.value(accel, ACCEL_LIMITS, "accel")
        if coordinate_system:
            tcp_pose = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=tcp_pose,
                orientation_units=orientation_units,
            )
        tcp_pose = set_position_orientation_units(tcp_pose, orientation_units)
        command = MoveCommandTemplate(
            t=WayPointType.linear_cart,
            des_x=tcp_pose,
            v_max_t=speed,
            v_max_r=speed,
            a_max_t=accel,
            a_max_r=accel,
            r_blend=blend,
        )
        return self._motion_host._add_waypoint(command)

    @handle_connection(available_in_read_only=False)
    @transforms_coordinates(system_param="coordinate_system")
    def add_new_offset(
        self,
        waypoint: PositionOrientation,
        offset: PositionOrientation,
        coordinate_system: CoordinateSystem | None = None,
        speed: float | None = None,
        accel: float | None = None,
        blend: float | None = None,
        orientation_units: AngleUnits | None = None,
    ) -> bool:
        """Добавляет точку линейного движения, вычисленную как смещение от базовой позиции.

        Итоговая целевая позиция рассчитывается как:
        `целевая_точка = waypoint + offset`,
        где сложение выполняется в указанной системе координат.

        Этот метод удобен для относительного позиционирования:
        например, "поднять инструмент на 5 см над текущей точкой" или
        "сдвинуть на 10 мм вдоль локальной оси X".
        Для смещения в пользовательской системе координат необходимо передать
        данную СК в качестве аргумента метода.

        Args:
            waypoint (PositionOrientation): Базовая позиция в формате
                `(X, Y, Z, Rx, Ry, Rz)`, относительно которой применяется смещение.
            offset (PositionOrientation): Вектор смещения в том же формате.
                Линейные компоненты — в метрах, угловые — в единицах,
                указанных в `orientation_units`.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой выполняется сложение `waypoint + offset`.
                По умолчанию используется глобальная система координат основания.
                Для локального смещения (например, вдоль TCP) передайте
                соответствующую пользовательскую СК.
            speed (float, optional): Линейная скорость движения, м/с (0–3).
                По умолчанию — из глобальной конфигурации (`set_motion_config`).
            accel (float, optional): Линейное ускорение, м/с² (0–15).
                По умолчанию — из глобальной конфигурации.
            blend (float, optional): Радиус сглаживания в метрах.
                При приближении к точке на расстояние ≤ `blend` робот плавно
                переходит к следующему сегменту. `0` — без сглаживания.
            orientation_units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Сдвинуть на 5 см вверх от точки (0.3, 0, 0.5)
            >>> base = (0.3, 0.0, 0.5, 0, 3.14, 0)
            >>> offset = (0.0, 0.0, 0.05, 0, 0, 0)
            >>> robot.motion.linear.add_new_offset(base, offset, speed=0.2, orientation_units="rad")

        Notes:
            - Все линейные значения (`waypoint`, `offset`) должны быть в **метрах**.
            - Углы в `waypoint` и `offset` должны быть в **одних и тех же единицах**,
              которые указаны в `orientation_units`.
            - Добавленная точка помещается в очередь — движение запускается через
              `robot.motion.mode.set('move')`.
        """

        orientation_units = orientation_units or MOTION_SETUP.units
        motion_setup = MOTION_SETUP.with_units(orientation_units)
        speed = speed or motion_setup.linear_speed
        accel = accel or motion_setup.linear_acceleration
        blend = blend or motion_setup.blend

        Validation.literal("angle", orientation_units)
        for tcp_pose in (waypoint, offset):
            Validation.length(
                tcp_pose, POSITION_ORIENTATION_LENGTH, "tcp_pose"
            )
        Validation.value(blend, BLEND_LIMITS, "blend")
        Validation.value(speed, SPEED_LIMITS, "speed")
        Validation.value(accel, ACCEL_LIMITS, "accel")
        new_tcp_pose = [
            waypoint_coordinate + offset_coordinate
            for waypoint_coordinate, offset_coordinate in zip(waypoint, offset)
        ]
        if coordinate_system:
            new_tcp_pose = convert_position_orientation(
                coordinate_system=coordinate_system,
                position_orientation=new_tcp_pose,
                orientation_units=orientation_units,
            )
        new_tcp_pose = set_position_orientation_units(
            new_tcp_pose, orientation_units
        )
        command = MoveCommandTemplate(
            t=WayPointType.linear_cart,
            des_x=new_tcp_pose,
            v_max_t=speed,
            v_max_r=speed,
            a_max_t=accel,
            a_max_r=accel,
            r_blend=blend,
        )
        return self._motion_host._add_waypoint(command)

    @handle_connection(available_in_read_only=True)
    @transforms_coordinates(system_param="coordinate_system")
    def get_actual_position(
        self,
        orientation_units: AngleUnits | None = None,
        coordinate_system: CoordinateSystem | None = None,
    ) -> PositionOrientation:
        """Возвращает текущую позицию и ориентацию конечного инструмента (TCP).

        Метод предоставляет актуальные координаты TCP в линейном формате
        `(X, Y, Z, Rx, Ry, Rz)`, где первые три значения — положение в метрах,
        последние три — углы поворота вокруг осей.

        Позиция возвращается в указанной системе координат:
        - если задана `coordinate_system` — в ней;
        - иначе — в глобальной системе координат основания робота.

        Метод доступен в режиме «read only».

        Args:
            orientation_units (AngleUnits, optional): Единицы измерения углов:
                - `'deg'` — градусы (по умолчанию);
                - `'rad'` — радианы.
            coordinate_system (CoordinateSystem, optional): Система координат,
                в которой возвращаются координаты TCP.
                По умолчанию используется система основания робота.

        Returns:
            PositionOrientation: Кортеж из 6 чисел:
                `(X, Y, Z, Rx, Ry, Rz)`, где:
                - `(X, Y, Z)` — координаты в метрах;
                - `(Rx, Ry, Rz)` — углы в указанных единицах измерения.

        Examples:
            >>> # Получить позицию в глобальной СК (градусы)
            >>> pose = robot.motion.linear.get_actual_position()
            >>> x, y, z, rx, ry, rz = pose

            >>> # Получить позицию в пользовательской СК (радианы)
            >>> from API.coords import CoordinateSystem
            >>> local_coord_system = CoordinateSystem(
            ...     position_orientation=((-0.3332, -0.1838, -0.0198, 3.138, 0, 0.8195)),
            ...     orientation_units="rad",
            ... )
            >>> pose = robot.motion.linear.get_actual_position(
            ...     coordinate_system=local_coord_system,
            ...     orientation_units='rad'
            ... )
        """

        orientation_units = orientation_units or MOTION_SETUP.units
        Validation.literal("angle", orientation_units)
        return self._motion_host.get_actual_position(
            orientation_units, "tcp", coordinate_system
        )

    @handle_connection(available_in_read_only=False)
    def jog_once(self, jog_axis: JogAxis, jog_direction: JogDirection) -> bool:
        """Выполняет кратковременный шаг джоггинга (перемещения) по заданной координате.

        Метод предназначен для ручного управления роботом в реальном времени:
        например, смещение TCP по X или вращение вокруг оси Z.
        Это **один цикл управления** — для непрерывного движения метод должен
        вызываться **циклически с частотой не менее 100 Гц** (каждые ≤10 мс).

        Args:
            jog_axis (JogAxis): Ось перемещения. Допустимые значения:
                - 'X', 'Y', 'Z' — линейные перемещения;
                - 'Rx', 'Ry', 'Rz' — угловые повороты.
            jog_direction (JogDirection): Направление движения:
                - '+' — в положительном направлении оси;
                - '-' — в отрицательном направлении оси.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Непрерывное движение по +X в течение 2 секунд (200 Гц)
            >>> import time
            >>> start = time.time()
            >>> while time.time() - start < 2.0:
            ...     robot.motion.linear.jog_once('X', '+')
            ...     time.sleep(0.005)  # 200 Гц — безопасная частота

        Notes:
            - При снижении частоты вызова ниже 100 Гц движение может стать дерганным;
              при полной остановке вызовов — робот останавливается.
            - Скорость и ускорение джоггинга можно настроить через scale_setup.
            - Одновременно можно перемещаться только по одной координате.
        """

        Validation.literal("axis", jog_axis)
        Validation.literal("math", jog_direction)
        jog_template = JogCommandTemplate()
        jog_template.var[int(literal_to_int(jog_axis))] = literal_to_int(
            jog_direction
        )
        jog_template.mode = JogModes.velocity
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_JOG, jog_template
        )

    @handle_connection(available_in_read_only=False)
    def set_jog_param_in_tcp(self, coordinate_system: ReferenceFrame) -> bool:
        """Настраивает систему координат для режима TCP Jogging.

        Определяет, относительно каких осей будет выполняться перемещение
        при вызове `jog_once`:
        - если выбрана система **основания**, то оси X, Y, Z фиксированы
          относительно корпуса робота;
        - если выбрана система **TCP**, то оси X, Y, Z связаны с текущей
          ориентацией инструмента (например, +X всегда «вперёд» от инструмента).

        Args:
            coordinate_system (ReferenceFrame): выбранная для движения система координат.
                'base' — система координат основания робота.
                'tcp' — система координат ЦТИ.

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Переключиться на управление относительно TCP
            >>> robot.motion.linear.set_jog_param_in_tcp('tcp')
            >>> robot.motion.linear.jog_once('X', '+')  # движение "вперёд" от инструмента

            >>> # Вернуться к глобальному управлению
            >>> robot.motion.linear.set_jog_param_in_tcp('base')
            >>> robot.motion.linear.jog_once('Z', '-')  # движение вниз по оси Z базы

        Notes:
            - Настройка действует до следующего изменения или перезапуска контроллера.
        """
        Validation.literal("reference_frame", coordinate_system)

        in_tcp = JogParamInTCP.FALSE
        if coordinate_system == "tcp":
            in_tcp = JogParamInTCP.TRUE

        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_JOG_PARAM,
            JogCommandParametersTemplate(in_tcp=in_tcp),
        )
