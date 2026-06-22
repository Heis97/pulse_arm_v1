from __future__ import annotations

import dataclasses
import math
import time
from collections.abc import Generator
from datetime import datetime
from typing import Any

from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    degrees_to_radians,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    CHECK_FREQUENCY_SEC,
    ORIENTATION_SLICE,
    POSITION_SLICE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    PositionOrientation,
)


def flatten_recursive(obj: Any) -> tuple[Any]:
    """Рекурсивно преобразует вложенные структуры данных в плоский кортеж.

    Автоматически раскрывает экземпляры датаклассов, списки и кортежи
    до уровня скалярных значений. Предназначена для подготовки аргументов
    перед бинарной сериализацией через `struct.pack`.

    Args:
        obj: Входной объект. Поддерживаются экземпляры `@dataclass`,
            `list`, `tuple` и любые скалярные типы (`int`, `float`,
            `bool` и др.).

    Returns:
        Плоский кортеж, содержащий все конечные значения в порядке
        обхода «в глубину» (depth-first).

    Examples:
        >>> flatten_recursive([1, [2, 3], (4, 5)])
        ... (1, 2, 3, 4, 5)

        >>> @dataclass
        ... class MotionTpl:
        ...     vel: float = 0.0
        ...     joints: list[float] = field(default_factory=lambda: [0.0] * 6)
        >>> flatten_recursive(MotionTpl(0.5, [0.1, 0.2, 0.0, 0.0, 0.0, 0.0]))
        ... (0.5, 0.1, 0.2, 0.0, 0.0, 0.0, 0.0)

    Notes:
        - Порядок элементов строго соответствует порядку обхода DFS.
        - Функция **не проверяет** соответствие количества элементов формату
          `struct`. Ошибка `struct.error` будет выброшена на этапе
          упаковки, если количество значений не совпадёт со спецификаторами.
        - `dict`, `set` и другие итерируемые типы обрабатываются как
          скаляры и не раскрываются рекурсивно.
        - Для высокочастотных вызовов (>10k/сек) рекомендуется использовать
          явные методы сериализации в датаклассах, так как рекурсия и
          аллокация новых объектов добавляют небольшой оверхед.
    """
    if dataclasses.is_dataclass(obj) and not isinstance(obj, type):
        return flatten_recursive(dataclasses.astuple(obj))
    if isinstance(obj, (list, tuple)):
        result = []
        for item in obj:
            result.extend(flatten_recursive(item))
        return tuple(result)
    return (obj,)


def sleep(
    await_sec: int | float = -1,
    frequency: int | float = CHECK_FREQUENCY_SEC,
) -> Generator[float, int | float, None]:
    """Генератор, эмулирующий асинхронную задержку с возможностью отслеживания оставшегося времени.

    Функция предоставляет итерируемый интерфейс для "ожидания" с заданной длительностью,
    при этом на каждой итерации возвращается оставшееся время в секундах.
    Может использоваться для реализации прогресс-бара, анимации ожидания
    или периодического выполнения проверок без блокировки внешнего цикла.

    Если `await_sec < 0`, функция работает в режиме бесконечного ожидания
    (на каждой итерации возвращает фиктивное значение `-1`), пока не будет
    прервана извне (например, исключением или явным выходом из цикла).

    Args:
        await_sec: Длительность ожидания в секундах. Если значение отрицательное,
            ожидание становится бесконечным (по умолчанию: `-1`).
        frequency: Интервал между итерациями в секундах. Определяет частоту
            проверок и обновлений (по умолчанию: `0.002` секунды, т.е. 2 мс).

    Yields:
        float: Оставшееся время ожидания в секундах.
            При бесконечном режиме (`await_sec < 0`) всегда возвращается `-1.0`.

    Examples:
        Пример 1: Ожидание с отслеживанием оставшегося времени.

        >>> for remaining in sleep(await_sec=2.0, frequency=0.5):
        ...     print(f"Осталось: {remaining} сек")
        Осталось: 2.0 сек
        Осталось: 1.5 сек
        Осталось: 1.0 сек
        Осталось: 0.5 сек

        Пример 2: Бесконечное ожидание с прерыванием по условию.

        >>> counter = 0
        >>> for _ in sleep(await_sec=-1, frequency=0.1):
        ...     counter += 1
        ...     if counter >= 5:
        ...         break
        ...     print("Ожидание...", counter)
        Ожидание... 1
        Ожидание... 2
        Ожидание... 3
        Ожидание... 4

    Notes:
        Несмотря на название, функция **не является заменой** `time.sleep()` —
        она предназначена для использования в циклах, где требуется
        пошаговое управление ожиданием.
    """
    start_time = datetime.now()
    while True:
        current_sec = (
            await_sec
            if await_sec < 0
            else (datetime.now() - start_time).total_seconds()
        )
        if current_sec > await_sec:
            return time.sleep(frequency)
        time.sleep(frequency)
        yield round(await_sec - current_sec, 3)


def literal_to_int(value: str | int | float) -> int | float:
    if isinstance(value, int) or isinstance(value, float):
        return value
    signs = {"-": -1, "+": 1}
    axis = ("X", "Y", "Z", "Rx", "Ry", "Rz")
    if value in axis:
        return axis.index(value)
    if value in signs.keys():
        return signs.get(value, 0)
    return 0


def set_position_orientation_units(
    position_orientation: PositionOrientation,
    orientation_units: AngleUnits,
) -> PositionOrientation:
    """
    Метод для перевода градусов в радианы в переданных позиции и ориентации.
    """

    return (
        list(position_orientation[POSITION_SLICE])
        + degrees_to_radians(position_orientation[ORIENTATION_SLICE])
        if orientation_units == "deg"
        else position_orientation
    )


def normalize_joint_angles(
    angle_pose: PositionOrientation,
    units: AngleUnits,
) -> PositionOrientation:
    """
    Reduce equivalent joint rotations to the inclusive range [-360, 360]
    degrees or [-2pi, 2pi] radians.
    """

    full_turn = 360.0 if units == "deg" else 2.0 * math.pi
    normalized = []
    for angle in angle_pose:
        current = float(angle)
        while current > full_turn:
            current -= full_turn
        while current < -full_turn:
            current += full_turn
        normalized.append(current)
    return normalized
