from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Literal

from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    PositionOrientation,
)


@dataclass
class MotionSetup:
    """
    Глобальные настройки движения. Все параметры заданы в метрах и градусах.
    """

    units: AngleUnits = "deg"

    linear_speed: float = 0.25
    linear_acceleration: float = 1.0

    joint_speed: float = 60
    joint_acceleration: float = 80

    blend: float = 0.0

    joints_speed: PositionOrientation = (60.0,) * 6  # for RPMP
    joints_accel: PositionOrientation = (120.0,) * 6  # for RPMP

    rotation_speed: float = 45.0  # TCP, for RPMP
    rotation_acceleration: float = 90.0  # TCP, for RPMP

    movej_init_pose: PositionOrientation = (
        0.0,
        -120.0,
        120.0,
        -90.0,
        -90.0,
        0.0,
    )  # for RPMP

    JOINT_SPEED_LIMITS: tuple[float, float] = (0, 180)  # deg/s
    JOINT_ACCEL_LIMITS: tuple[float, float] = (0, 1500)  # deg/s^2
    RPMP_ROTATION_SPEED_LIMITS: tuple[float, float] = (0, 360)  # deg/s
    RPMP_ROTATION_ACCEL_LIMITS: tuple[float, float] = (0, 720)  # deg/s^2

    _VALUES_UNITS: AngleUnits = "deg"
    _FIELDS_NAMES = Literal[
        "units",
        "linear_speed",
        "linear_acceleration",
        "joint_speed",
        "joint_acceleration",
        "blend",
        "joints_speed",
        "joints_accel",
        "rotation_speed",
        "rotation_acceleration",
        "movej_init_pose",
        "JOINT_SPEED_LIMITS",
        "JOINT_ACCEL_LIMITS",
        "RPMP_ROTATION_SPEED_LIMITS",
        "RPMP_ROTATION_ACCEL_LIMITS",
    ]

    # Список полей, требующих конвертации rad <-> deg
    _ANGULAR_FIELDS: frozenset[str] = frozenset(
        {
            "joint_speed",
            "joint_acceleration",
            "joints_speed",
            "joints_accel",
            "rotation_speed",
            "rotation_acceleration",
            "movej_init_pose",
            "JOINT_SPEED_LIMITS",
            "JOINT_ACCEL_LIMITS",
            "RPMP_ROTATION_SPEED_LIMITS",
            "RPMP_ROTATION_ACCEL_LIMITS",
        }
    )

    def set_value(
        self,
        field: _FIELDS_NAMES,
        value: float | PositionOrientation | AngleUnits | None,
        units: AngleUnits | None = None,
    ) -> None:
        """
        Явно задаёт значение с указанием единиц измерения.
        Безопасно меняет существующий объект, конвертируя в deg.
        """
        if value is None:
            return
        if field not in self._ANGULAR_FIELDS:
            setattr(self, field, value)
            return

        if units is None:
            units = self.units

        if units != self._VALUES_UNITS:
            value = self._convert_angle(
                value,  # type: ignore
                from_unit=units,
                to_unit=self._VALUES_UNITS,
            )
        setattr(self, field, value)

    def get_value(
        self, field: _FIELDS_NAMES, target_units: AngleUnits | None = None
    ) -> float | PositionOrientation:
        """
        Возвращает значение параметра, сконвертированное в указанные единицы.

        Args:
            field: Имя параметра (например, 'rotation_speed')
            target_unit: Целевая единица ('deg' или 'rad'). Если None, используется self.units
        """
        target = target_units or self.units
        if field not in self._ANGULAR_FIELDS:
            return getattr(self, field)

        return self._convert_angle(
            getattr(self, field), self._VALUES_UNITS, target
        )

    def with_units(self, target_units: AngleUnits) -> MotionSetup:
        """
        Возвращает новую копию конфигурации с конвертированными угловыми параметрами.
        Безопасная замена прямой модификации self.units.
        """
        if target_units == self._VALUES_UNITS:
            return self

        data = asdict(self)
        data["units"] = target_units

        for field_name in self._ANGULAR_FIELDS:
            data[field_name] = self._convert_angle(
                data[field_name], self._VALUES_UNITS, target_units
            )

        return MotionSetup(**data)

    def _convert_angle(
        self,
        value: float | PositionOrientation,
        from_unit: AngleUnits,
        to_unit: AngleUnits,
    ) -> float | PositionOrientation:
        """Внутренняя конвертация угловых значений."""
        if from_unit == to_unit:
            return value

        to_rad = to_unit == "rad"
        if isinstance(value, (list, tuple)):
            return tuple(
                math.radians(v) if to_rad else math.degrees(v) for v in value
            )

        return math.radians(value) if to_rad else math.degrees(value)


MOTION_SETUP: MotionSetup = MotionSetup()
