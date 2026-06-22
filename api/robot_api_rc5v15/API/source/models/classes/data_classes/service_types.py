from __future__ import annotations

from dataclasses import dataclass, fields
from typing import NamedTuple


class JointAngleDiscrepancy(NamedTuple):
    """
    Класс для передачи информации о расхождении сохраненного положения звена с
    реальным положением.

    Args:
        joint_number: номер звена считая от основания;
        allowed_discrepancy: допустимое рассогласование углов (deg);
        actual_position: снимаемое с привода положение (deg);
        saved_position: сохраненная в контроллере позиция (deg).
    """

    joint_number: int = 0
    allowed_discrepancy: float = 0
    actual_position: float = 0
    saved_position: float = 0


@dataclass
class DhModelParams:
    """
    Класс для представления информации о DH параметрах робота.

    Args:
        alpha: угол скручивания (rad);
        a: длина звена (m);
        d: смещение звена (m);
        theta: угол сочленения (rad);
        offset: смещение нулевого положения звена (rad).
    """

    alpha: tuple[float, ...] = (0,) * 6
    a: tuple[float, ...] = (0,) * 6
    d: tuple[float, ...] = (0,) * 6
    theta: tuple[float, ...] = (0,) * 6
    offset: tuple[float, ...] = (0,) * 6


@dataclass
class JointsLimitsParams:
    """
    Класс для представления информации о пределах шарниров.

    Args:
        joint_position_low_limits: крайнее нижнее угловое положение звеньев (deg или rad);
        joint_position_high_limits: крайнее верхнее угловое положение звеньев (deg или rad).
    """

    low_limits: tuple[float, ...]
    high_limits: tuple[float, ...]

    def __str__(self) -> str:
        """
        Автоматически форматирует все поля датакласса.
        """
        output = []
        for field in fields(self):
            value = getattr(self, field.name)

            if isinstance(value, tuple):
                if value and isinstance(value[0], float):
                    formatted = (
                        "(" + ", ".join(f"{v:.3f}" for v in value) + ")"
                    )
                else:
                    formatted = str(value)
            elif isinstance(value, float):
                formatted = f"{value:.3f}"
            elif isinstance(value, int):
                formatted = str(value)
            else:
                formatted = str(value)

            output.append(f"{field.name}: {formatted}")

        return f"{self.__class__.__name__}(" + ", ".join(output) + ")"


@dataclass
class SafetyLimitsParams:
    """
    Класс для представления информации о пределах безопасности робота.

    Args:
        max_tcp_velocity (float): Максимальная скорость центра инструмента (TCP), м/с.
            Используется как лимит при планировании траектории.
        max_joints_velocity (Tuple[float, ...]): Максимальные угловые скорости
            сочленений, рад/с. Порядок: от основания (J1) к фланцу (J6).
        joints_limits (JointsLimitsParams): Ограничения положений шарниров.
        joint_following_error (float): Допустимая ошибка следования по положению
            сочленения, рад.
        cartesian_translation_following_error (float): Допустимая ошибка
            позиционирования ЦТИ в декартовом пространстве при движении, мм.
        cartesian_rotation_following_error (float): Допустимая ошибка ориентации
            ЦТИ в декартовом пространстве при движении, рад.
        max_stop_time (float): Максимальное время полной остановки после
            получения команды, с.
    """

    max_tcp_velocity: float
    max_joints_velocity: tuple[float, ...]
    joints_limits: JointsLimitsParams
    joint_following_error: float
    cartesian_translation_following_error: float
    cartesian_rotation_following_error: float
    max_stop_time: float

    def __str__(self) -> str:
        """
        Автоматически форматирует все поля датакласса.
        """
        output = []
        for field in fields(self):
            value = getattr(self, field.name)

            if isinstance(value, tuple):
                if value and isinstance(value[0], float):
                    formatted = (
                        "(" + ", ".join(f"{v:.3f}" for v in value) + ")"
                    )
                else:
                    formatted = str(value)
            elif isinstance(value, float):
                formatted = f"{value:.3f}"
            elif isinstance(value, int):
                formatted = str(value)
            else:
                formatted = str(value)

            output.append(f"{field.name}: {formatted}")

        return f"{self.__class__.__name__}(" + ", ".join(output) + ")"
