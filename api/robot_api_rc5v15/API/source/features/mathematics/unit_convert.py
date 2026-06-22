from __future__ import annotations

import math
from typing import overload
from collections.abc import Iterable


@overload
def radians_to_degrees(args: int | float) -> float: ...


@overload
def radians_to_degrees(args: Iterable[int | float]) -> list[float]: ...


def radians_to_degrees(
    args: int | float | Iterable[int | float],
) -> float | list[float]:
    """
    Перевод радианов в градусы.
    Args:
        args: последовательность чисел для конвертации.

    Returns:
        Лист конвертированных чисел.
    """
    if isinstance(args, Iterable):
        return [math.degrees(float(value)) for value in args]
    return math.degrees(float(args))


@overload
def degrees_to_radians(args: int | float) -> float: ...


@overload
def degrees_to_radians(args: Iterable[int | float]) -> list[float]: ...


def degrees_to_radians(
    args: int | float | Iterable[int | float],
) -> float | list[float]:
    """
    Перевод градусов в радианы.
    Args:
        args: последовательность чисел для конвертации.

    Returns:
        Лист конвертированных чисел.
    """
    if isinstance(args, Iterable):
        return [math.radians(float(value)) for value in args]
    return math.radians(float(args))
