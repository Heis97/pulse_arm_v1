from enum import Enum, IntEnum


class AngleUnitTypes(str, Enum):
    DEG = "Градусы"
    RAD = "Радианы"


class CoordinateSystemInfoType(str, Enum):
    POSITION_ORIENTATION = "Позиция ориентация"
    ORIENTATION_UNITS = "Единицы измерения углов"


class GUICoordinateSystem(str, Enum):
    CTI = "ЦТИ"
    GLOBAL = "Основание"
    LOCAL = "Пользовательская"


class JogParamInTCP(IntEnum):
    TRUE = 1
    FALSE = 0


class MotionTypes(str, Enum):
    JOINT = "Joint"
    LINEAR = "Linear"


class AddWayPointErrorCode(IntEnum):
    """
    Класс ошибок добавления точки.
    Значения класса используются для определения успешности добавления
    точки и причины ошибки, если точку добавить не удалось.
    """

    success = 0
    buffer_full = 1
    out_of_limits = 2
    out_of_reach = 3
    non_finite = 4
    singularity = 5


class PowerUnitsCode(IntEnum):
    V = 0
    mA = 1
