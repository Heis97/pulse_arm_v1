"""
Модуль с инструментами для работы с пользовательскими системами координат.

Примеры:
>>> from API import coords
>>> from API.coords import CoordinateSystem
"""

from .source.ap_interface.motion import CoordinateSystem
from .source.features.mathematics.coordinate_system import (
    calculate_plane_from_points,
    convert_position_orientation,
)

__all__ = [
    "CoordinateSystem",
    "calculate_plane_from_points",
    "convert_position_orientation",
]
