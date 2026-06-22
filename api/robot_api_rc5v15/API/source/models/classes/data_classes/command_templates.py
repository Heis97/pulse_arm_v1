from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import WristMode
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.various_types import PowerUnitsCode
from api.robot_api_rc5v15.API.source.models.constants import (
    BYTE_SIZE,
    CTRLR_MAX_AN_OUT,
    CTRLR_MAX_DIG_OUT_BYTES,
    REAL_WRIST_MAX_AN_IN,
    WRIST_MAX_DIG_OUT,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    PositionOrientation,
    PowerUnits,
)


@dataclass
class RPMPCartesianWayPointTemplate:
    tr_position: tuple[float, ...] = (0.0,) * 3
    rot_position: tuple[float, ...] = (0.0,) * 3
    tr_velocity: float = 0.0
    rot_velocity: float = 0.0
    tr_acceleration: float = 0.0
    rot_acceleration: float = 0.0


@dataclass
class RPMPJointWayPointTemplate:
    q: tuple[float, ...] = (0.0,) * 6
    qd: tuple[float, ...] = (0.0,) * 6
    qdd: tuple[float, ...] = (0.0,) * 6


@dataclass
class RPMPJointTcpWayPointTemplate:
    tr_position: tuple[float, ...] = (0.0,) * 3
    rot_position: tuple[float, ...] = (0.0,) * 3
    init_q: tuple[float, ...] = (0.0,) * 6
    ik_solution: int = -1  # -1 - ближайшее
    qd: tuple[float, ...] = (0.0,) * 6
    qdd: tuple[float, ...] = (0.0,) * 6


@dataclass
class RPMPMoveLWayPointTemplate:
    cart_wp: RPMPCartesianWayPointTemplate = field(
        default_factory=RPMPCartesianWayPointTemplate
    )


@dataclass
class RPMPMovePWayPointTemplate:
    cart_wp: RPMPCartesianWayPointTemplate = field(
        default_factory=RPMPCartesianWayPointTemplate
    )


@dataclass
class RPMPMoveCWayPointTemplate:
    cart_wp_1: RPMPCartesianWayPointTemplate = field(
        default_factory=RPMPCartesianWayPointTemplate
    )
    cart_wp_2: RPMPCartesianWayPointTemplate = field(
        default_factory=RPMPCartesianWayPointTemplate
    )


@dataclass
class RPMPMoveJWayPointTemplate:
    joint_wp: RPMPJointWayPointTemplate = field(
        default_factory=RPMPJointWayPointTemplate
    )


@dataclass
class RPMPMoveJTcpWayPointTemplate:
    joint_wp: RPMPJointTcpWayPointTemplate = field(
        default_factory=RPMPJointTcpWayPointTemplate
    )


@dataclass
class RPMPMoveCommandTemplate:
    type: int = 0
    blend_radius: float = 0.0
    wp: (
        RPMPMoveLWayPointTemplate
        | RPMPMovePWayPointTemplate
        | RPMPMoveCWayPointTemplate
        | RPMPMoveJWayPointTemplate
        | RPMPMoveJTcpWayPointTemplate
    ) = field(default_factory=RPMPMoveJWayPointTemplate)


@dataclass
class MoveCommandTemplate:
    t: int = 0
    des_q: PositionOrientation = (0,) * 6
    des_x: PositionOrientation = (0,) * 6
    force: tuple[float, ...] = (0,) * 6
    force_en: tuple[float, ...] = (0,) * 6
    in_tcp: float = 0
    v_max_t: float = 0
    v_max_r: float = 0
    a_max_t: float = 0
    a_max_r: float = 0
    v_max_j: float = 0
    a_max_j: float = 0
    r_blend: float = 0
    pseg: int = -1


@dataclass
class JogCommandParametersTemplate:
    in_tcp: int = 0
    force_en: list[float] = field(default_factory=lambda: [0] * 6)
    force: list[float] = field(default_factory=lambda: [0] * 6)
    speed_max: list[float] = field(
        default_factory=lambda: [0.2] * 3 + [0.5] * 3
    )
    accel: list[float] = field(default_factory=lambda: [0.1] * 3 + [0.25] * 3)
    decel: list[float] = field(default_factory=lambda: [2.0] * 3 + [2.5] * 3)


@dataclass
class JogCommandTemplate:
    mode: int = 0
    force_en: list[float] = field(default_factory=lambda: [0] * 6)
    force_max: list[float] = field(default_factory=lambda: [0] * 6)
    force_const: list[float] = field(default_factory=lambda: [0] * 6)
    stiff: list[float] = field(default_factory=lambda: [0] * 6)
    var: list[float] = field(default_factory=lambda: [0] * 6)


@dataclass
class JointJogCommandTemplate:
    mode: int = 0
    joints_rotation_directions: list[float] = field(
        default_factory=lambda: [0] * 6
    )


@dataclass
class SetOutputTemplate:
    dig_out_mask: list[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_DIG_OUT_BYTES
    )
    dig_out: list[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_DIG_OUT_BYTES
    )
    an_out_mask: list[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_AN_OUT
    )
    an_out_curr_mode: list[int] = field(
        default_factory=lambda: [0] * CTRLR_MAX_AN_OUT
    )
    an_out_value: list[float] = field(
        default_factory=lambda: [0] * CTRLR_MAX_AN_OUT
    )

    def set_digital_output(self, index: int, value: bool) -> None:
        if not 0 <= index < len(self.dig_out_mask) * BYTE_SIZE:
            raise ValueError(
                f"Output index {index} out of range [0, {len(self.dig_out_mask) * BYTE_SIZE - 1}]"
            )

        byte_idx, bit_offset = divmod(index, BYTE_SIZE)
        bit_mask = 1 << bit_offset

        self.dig_out_mask[byte_idx] |= bit_mask

        if value:
            self.dig_out[byte_idx] |= bit_mask
        else:
            self.dig_out[byte_idx] &= ~bit_mask

    def set_analog_output(
        self, index: int, value: float, units: PowerUnits
    ) -> None:
        if not 0 <= index < len(self.an_out_mask):
            raise ValueError(
                f"Analog output index {index} out of range [0, {len(self.an_out_mask) - 1}]"
            )

        self.an_out_mask[index] = 1
        try:
            self.an_out_curr_mode[index] = PowerUnitsCode[units]
        except KeyError as exc:
            raise ValueError(f"Invalid power unit: {units!r}") from exc
        self.an_out_value[index] = float(value)


@dataclass
class InverseKinematicOptimalTemplate:
    target: list[float] = field(default_factory=lambda: [0] * 6)
    base_q: list[float] = field(default_factory=lambda: [0] * 6)


@dataclass
class SetWristInputOutputTemplate:
    dig_out_mask: int = 0
    dig_out: int = 0
    an_in_mask: list[int] = field(
        default_factory=lambda: [0] * REAL_WRIST_MAX_AN_IN
    )
    an_in_mode: list[int] = field(
        default_factory=lambda: [0] * REAL_WRIST_MAX_AN_IN
    )
    mux_mode: int = 0

    def set_digital_output(self, index: int, value: bool) -> None:
        """Безопасно устанавливает/сбрасывает бит цифрового выхода запястья."""
        if not 0 <= index < WRIST_MAX_DIG_OUT:
            raise ValueError(
                f"Wrist digital output index {index} out of range [0, 7]"
            )

        bit_mask = 1 << index
        self.dig_out_mask |= (
            bit_mask  # ✅ Активируем бит в маске (не затирая остальные)
        )

        if value:
            self.dig_out |= bit_mask  # ✅ Включаем выход
        else:
            self.dig_out &= (
                ~bit_mask
            )  # ✅ Выключаем выход, не трогая соседние биты

    def set_mux_mode(self, mode: WristMode) -> None:
        self.mux_mode = mode

    def set_analog_input(self, index: int, units: Any) -> None:
        """Настраивает аналоговый вход запястья: включает маску и задаёт единицы."""
        if not 0 <= index < REAL_WRIST_MAX_AN_IN:
            raise ValueError(
                f"Wrist analog input index {index} out of range [0, {REAL_WRIST_MAX_AN_IN - 1}]"
            )

        self.an_in_mask[index] = 1
        try:
            val = PowerUnitsCode[units]
            self.an_in_mode[index] = getattr(val, "value", val)
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f"Invalid power unit for wrist analog input: {units!r}"
            ) from exc
