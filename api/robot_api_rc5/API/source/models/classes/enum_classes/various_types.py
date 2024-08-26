from enum import Enum

from source.models.classes.enum_classes.base_int_enum import BaseIntEnum


class AngleUnitTypes(str, Enum):
    DEG: str = 'Градусы'
    RAD: str = 'Радианы'


class CoordinateSystemInfoType(str, Enum):
    POSITION_ORIENTATION: str = 'Позиция ориентация'
    ORIENTATION_UNITS: str = 'Единицы измерения углов'


class GUICoordinateSystem(str, Enum):
    CTI: str = 'ЦТИ'
    GLOBAL: str = 'Основание'
    LOCAL: str = 'Пользовательская'


class JogParamInTCP(BaseIntEnum):
    TRUE: int = 1
    FALSE: int = 0


class MotionTypes(str, Enum):
    JOINT: str = 'Joint'
    LINEAR: str = 'Linear'
