from dataclasses import dataclass, field
from typing import List, Tuple
from API.source.models.constants import (CTRLR_MAX_DIG_OUT_BYTES,
                                         CTRLR_MAX_AN_OUT)
from API.source.models.type_aliases import AngleUnits, PositionOrientation


@dataclass
class MoveCommandTemplate:
    t: int = 0
    des_q: PositionOrientation = (0, ) * 6
    des_x: PositionOrientation = (0, ) * 6
    force: Tuple[float, ...] = (0, ) * 6
    force_en: Tuple[float, ...] = (0, ) * 6
    in_tcp: float = 0
    v_max_t: float = 0
    v_max_r: float = 0
    a_max_t: float = 0
    a_max_r: float = 0
    v_max_j: float = 0
    a_max_j: float = 0
    r_blend: float = 0
    pseg: int = 0


@dataclass
class JogCommandParametersTemplate:
    in_tcp: int = 0
    force_en: List[float] = field(default_factory=lambda: [0] * 6)
    force: List[float] = field(default_factory=lambda: [0] * 6)
    speed_max: List[float] = field(
        default_factory=lambda: [0.2] * 3 + [0.5] * 3
    )
    accel: List[float] = field(default_factory=lambda: [0.1] * 3 + [0.25] * 3)
    decel: List[float] = field(default_factory=lambda: [2.0] * 3 + [2.5] * 3)


@dataclass
class JogCommandTemplate:
    mode: int = 0
    force_en: List[float] = field(default_factory=lambda: [0] * 6)
    force_max: List[float] = field(default_factory=lambda: [0] * 6)
    force_const: List[float] = field(default_factory=lambda: [0] * 6)
    stiff: List[float] = field(default_factory=lambda: [0] * 6)
    var: List[float] = field(default_factory=lambda: [0] * 6)


@dataclass
class JointJogCommandTemplate:
    mode: int = 0
    joints_rotation_directions: List[float] = field(
        default_factory=lambda: [0] * 6
    )


@dataclass
class SetOutputTemplate:
    dig_out_mask: List[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_DIG_OUT_BYTES
    )
    dig_out: List[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_DIG_OUT_BYTES
    )
    an_out_mask: List[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_AN_OUT
    )
    an_out_curr_mode: List[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_AN_OUT
    )
    an_out_value: List[float] = field(
        default_factory=lambda: [0] * CTRLR_MAX_AN_OUT
    )


@dataclass
class InverseKinematicOptimalTemplate:
    target: list[float] = field(default_factory=lambda: [0] * 6)
    base_q: list[float] = field(default_factory=lambda: [0] * 6)


@dataclass
class MotionSetup:
    units: AngleUnits = 'deg'

    joint_speed: float = 60
    joint_acceleration: float = 80

    linear_speed: float = 0.25
    linear_acceleration: float = 0.25

    blend: float = 0


MOTION_SETUP: MotionSetup = MotionSetup()
