from __future__ import annotations

import logging
import re
from dataclasses import dataclass, field
from typing import Literal

from API.rc_api import RobotApi
from api.robot_api_rc5v15.API.source.ap_interface.motion import CoordinateSystem
from api.robot_api_rc5v15.API.source.models.type_aliases import PositionOrientation, PowerUnits

from .utils import gcode_val_to_meters


@dataclass(frozen=True)
class GCodeCommand:
    """Неизменяемое представление одной команды G-кода.

    Содержит разобранный идентификатор команды, её параметры, номер строки
    и исходный текст. Используется интерпретатором для безопасной передачи
    данных обработчикам и отладки.

    Attributes:
        command: Идентификатор команды (например, "G1", "M6", "F").
        params: Словарь параметров. Числовые значения хранятся как `float`,
            логические флаги или отсутствующие значения представлены как `True`.
        line_number: Номер строки в исходном файле. `-1`, если не указан.
        raw: Исходная строка из файла (включает пробелы, N-коды и комментарии).
    """

    command: str
    params: dict[str, float | bool]
    line_number: int = -1
    raw: str = ""

    @classmethod
    def from_raw(
        cls, raw_line: str, line_number: int | None = None
    ) -> GCodeCommand | None:
        """Разбирает сырую строку G-кода в объект `GCodeCommand`.

        Удаляет комментарии (в `()` и после `;`), нормализует регистр,
        игнорирует номера строк (N-коды) при определении команды и выделяет параметры.

        Args:
            raw_line: Строка из G-кода файла или потока.
            line_number: Опциональный номер строки для отладки и логирования.

        Returns:
            Объект `GCodeCommand` или `None`, если строка пустая или содержит только комментарии.

        Examples:
            >>> GCodeCommand.from_raw("N10 G1 X50 Y0 F1000 ; move", line_number=10)
            GCodeCommand(command='G1', params={'X': 50.0, 'Y': 0.0, 'F': 1000.0}, line_number=10, raw='N10 G1 X50 Y0 F1000 ; move')
            >>> GCodeCommand.from_raw("(tool change comment)")
            None
        """
        line = raw_line.split(";")[0].strip()
        if not line:
            return None

        line = re.sub(r"\(.*?\)", "", line).upper()

        parts = line.split()
        if not parts:
            return None

        command = parts[0].upper()
        if command.startswith("N") and len(parts) > 1:
            command = parts[1].upper()

        params: dict = {}
        for part in parts[1:]:
            key = part[0].upper()
            try:
                params[key] = float(part[1:])
            except Exception:  # if param is logical flag
                params[key] = True

        return GCodeCommand(
            command=command,
            params=params,
            line_number=line_number or -1,
            raw=raw_line.strip(),
        )

    def parse_command(self) -> tuple[str, int]:
        """Извлекает тип и номер команды для маршрутизации в обработчики.

        Разделяет идентификатор на буквенный префикс (G, M, T, F и т.д.)
        и целочисленный номер. Если номер отсутствует или не является цифрой,
        возвращается `-1`.

        Returns:
            Кортеж `(prefix, number)`, где `prefix` — буква команды,
            `number` — её числовой идентификатор.

        Examples:
            >>> GCodeCommand("G28", {}).parse_command()
            ('G', 28)
            >>> GCodeCommand("M6", {"T": 2}).parse_command()
            ('M', 6)
            >>> GCodeCommand("F", {"F": 500.0}).parse_command()
            ('F', -1)
        """
        g_val = (
            int(self.command[1:])
            if len(self.command) > 1 and self.command[1:].isdigit()
            else -1
        )

        return (self.command[0], g_val)


@dataclass
class GCodeState:
    """Мутабельное состояние выполнения G-кода.

    Хранит текущую позу робота, активные модальные коды (режим координат,
    единицы измерения, плоскость интерполяции, подача) и рабочие смещения.
    Автоматически обновляется интерпретатором при парсинге каждой строки.

    Attributes:
        current_pose: Текущая позиция и ориентация TCP (X, Y, Z, A, B, C) в метрах и градусах.
        work_offset: Активное смещение рабочей системы координат (G54–G59).
        active_work_offset: Индекс активного смещения (1 для G54, 2 для G55 и т.д.).
        absolute_mode: Флаг режима позиционирования. `True` = G90 (абсолютный), `False` = G91 (инкрементальный).
        units_mm: Флаг единиц измерения. `True` = G21 (мм), `False` = G20 (дюймы).
        feed_m_s: Текущая скорость подачи в метрах в секунду (внутренне конвертируется из мм/мин).
        plane: Активная плоскость круговой интерполяции (`"XY"`, `"XZ"` или `"YZ"`).
        last_motion_cmd: Последняя выполненная команда движения (G0/G1/G2/G3) для модальных перемещений.
        is_machine_mode: Флаг машинных координат (G53). При `True` игнорирует `work_offset`.

    Examples:
        >>> state = GCodeState()
        >>> state.absolute_mode, state.units_mm, state.plane
        (True, True, 'XY')
    """

    current_pose: PositionOrientation = field(
        default_factory=lambda: (0.0,) * 6
    )
    work_offset: list[float] = field(default_factory=lambda: [0.0] * 6)
    active_work_offset: int = 1

    absolute_mode: bool = True  # G90 / G91
    units_mm: bool = True  # G21 / G20
    feed_m_s: float = 1.0
    plane: Literal["XY", "XZ", "YZ"] = "XY"  # G17 / G18 / G19

    last_motion_cmd: str = "G0"
    is_machine_mode: bool = False  # G53

    def update_modal(self, cmd: GCodeCommand) -> None:
        """Обновляет модальные параметры состояния на основе команды.

        Применяет изменения режимов (G90/G91, G17/G18/G19, G20/G21) и обновляет
        скорость подачи при наличии параметра `F`. Остальные команды игнорируются.

        Args:
            cmd: Разобранная команда G-кода. Только модальные коды влияют на состояние.

        Examples:
            G90                 ; Переключает в абсолютный режим (absolute_mode = True)
            G21 G17             ; Устанавливает миллиметры и плоскость XY
            F1200               ; Обновляет feed_m_s до 0.02 м/с
        """
        if cmd.command.startswith("G"):
            g_val = (
                int(cmd.command[1:])
                if len(cmd.command) > 1 and cmd.command[1:].isdigit()
                else 0
            )
            if g_val == 90:
                self.absolute_mode = True
            elif g_val == 91:
                self.absolute_mode = False
            elif g_val == 17:
                self.plane = "XY"
            elif g_val == 18:
                self.plane = "XZ"
            elif g_val == 19:
                self.plane = "YZ"
            elif g_val == 20:
                self.units_mm = False
            elif g_val == 21:
                self.units_mm = True

        if "F" in cmd.params or cmd.command.startswith("F"):
            self.feed_m_s = (
                gcode_val_to_meters(cmd.params["F"], self.units_mm) / 60.0
            )

    def resolve_target(self, cmd: GCodeCommand) -> PositionOrientation:
        """Вычисляет целевую 6D-позу с учётом модальных режимов и смещений.

        Преобразует координаты из команды в абсолютную позу робота.
        Учитывает режим G90/G91, единицы измерения (G20/G21) и активное
        рабочее смещение (G53/G54). Линейные оси конвертируются в метры,
        угловые (A/B/C) остаются в градусах.

        Args:
            cmd: Команда движения с координатами (X, Y, Z, A, B, C).

        Returns:
            Кортеж `(X, Y, Z, A, B, C)` в метрах и градусах, готовый для передачи в API робота.

        Examples:
            G90 X100 Y50        ; Возвращает (0.1, 0.05, Z_cur, A_cur, B_cur, C_cur)
            G91 X10             ; Смещает текущий X на +0.01 м
            G53 G0 Z0           ; Возвращает машинный ноль по Z (игнорирует work_offset)
        """
        cur = self.current_pose
        tgt = list(cur)
        mapping = {"X": 0, "Y": 1, "Z": 2, "A": 3, "B": 4, "C": 5}

        for axis, idx in mapping.items():
            if axis in cmd.params:
                val = cmd.params[axis]
                if axis in "XYZ":
                    val = gcode_val_to_meters(val, self.units_mm)
                # A/B/C остаются в градусах
                tgt[idx] = val if self.absolute_mode else cur[idx] + val

        # G54/G55 добавляют смещение к рабочим координатам
        if not self.is_machine_mode:
            for i in range(6):
                tgt[i] += self.work_offset[i]

        return tuple(tgt)


@dataclass
class GCodeExecutionContext:
    """Контекст выполнения G-кода.

    Передаётся во все обработчики команд. Содержит ссылки на API робота,
    настройки буферизации, параметры интерполяции и состояние исполнения.

    Attributes:
        robot: Экземпляр API робота для отправки команд движения и управления I/O.
        coordinate_system: Активная система координат (фрейм) для трансформации точек.
        offset_table: Таблица рабочих смещений G54–G59. Ключ — индекс смещения, значение — список из 6 float.
        min_buffer_size: Минимальный порог очереди точек для backpressure (ожидание освобождения буфера).
        max_buffer_size: Максимальный размер очереди контроллера. Используется для управления потоком.
        timeout: Максимальное время ожидания (в секундах) для операций очистки очереди или ожидания входов.
        blend: Базовый допуск сглаживания траектории (в метрах) для команд G64.
        active_blend: Текущий активный допуск сглаживания. Динамически обновляется командами G61/G64.
        use_joint_motion_for_g0: Если True то для команд G0 будет использоваться перемещение по углам сочленений.
        interpolate_linear: Флаг разбиения линейных перемещений на мелкие сегменты.
        interpolate_arcs: Флаг аппроксимации дуг линейными сегментами вместо нативных `MoveC`.
        interpolation_step: Максимальная длина сегмента (в метрах) при включённой интерполяции.
        analog_outputs_units: Единицы измерения аналоговых выходов.
        arc_via: Кэшированная промежуточная точка последней рассчитанной дуги (для отладки или API).
        current_line_number: Номер текущей обрабатываемой строки в источнике G-кода.
        current_raw_line: Исходный текст текущей строки (для логирования и отладки).
        logger: Экземпляр логгера. Если `None`, используется стандартный вывод или глобальный логгер.

    Examples:
        >>> ctx = GCodeExecutionContext(robot=api, max_buffer_size=50, blend=0.005)
        >>> ctx.interpolate_arcs = True
        >>> ctx.timeout = 120.0
    """

    robot: RobotApi
    coordinate_system: CoordinateSystem = field(
        default_factory=lambda: CoordinateSystem((0,) * 6)
    )
    offset_table: dict[int, list[float]] = field(
        default_factory=dict
    )  # G54-G59

    min_buffer_size: int = 10
    max_buffer_size: int = 50
    timeout: float = 200  # sec
    logger: logging.Logger | None = None

    blend: float = 0.001  # m
    active_blend: float = 0  # m
    use_joint_motion_for_g0: bool = True
    interpolate_linear: bool = False
    interpolate_arcs: bool = False
    interpolation_step: float = 0.005  # m
    analog_outputs_units: PowerUnits = "V"
    arc_via: PositionOrientation | None = None

    current_line_number: int = 0
    current_raw_line: str = ""
