from __future__ import annotations

import re
from collections.abc import Iterable, Sized
from typing import Any, get_args

from api.robot_api_rc5v15.API.source.core.exceptions.argument_error import (
    ArgControllerStateError,
    ArgInputFunctionError,
    ArgMotionModeError,
    ArgOutputFunctionError,
    ArgSafetyStatusError,
    ArgSignError,
    ArgToolModeError,
    ArgUnitsError,
    ArgValueError,
    ArrayLengthError,
    WristOutputActivationTypeError,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.io_functions import (
    InputFunction,
    OutputFunction,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.state_classes import (
    InComingSafetyStatus,
    OutComingControllerState,
    OutComingMotionMode,
    WristMode,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    ALLOWED_GUI_ENTRY_SYMBOLS,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    CompareSigns,
    IkSolutionId,
    JogAxis,
    JogDirection,
    LiteralType,
    PositionFormat,
    PowerUnits,
    ReferenceFrame,
    WristInputActivationType,
)


def _extract_allowed(source: Any) -> frozenset:
    if hasattr(source, "__members__"):  # Enum
        return frozenset(e.name for e in source)
    # Literal-алиас
    return frozenset(get_args(source))


class Validation:
    _VALIDATION_RULES: dict[LiteralType, tuple[Any, type[Exception]]] = {
        "compare": (CompareSigns, ArgSignError),
        "math": (JogDirection, ArgSignError),
        "angle": (AngleUnits, ArgUnitsError),
        "position": (PositionFormat, ArgUnitsError),
        "power": (PowerUnits, ArgUnitsError),
        "axis": (JogAxis, ArgValueError),
        "reference_frame": (ReferenceFrame, ArgValueError),
        "activation_type": (
            WristInputActivationType,
            WristOutputActivationTypeError,
        ),
        "ik_id": (IkSolutionId, ArgValueError),
        # Enums
        "cs": (OutComingControllerState, ArgControllerStateError),
        "mm": (OutComingMotionMode, ArgMotionModeError),
        "ss": (InComingSafetyStatus, ArgSafetyStatusError),
        "in_f": (InputFunction, ArgInputFunctionError),
        "out_f": (OutputFunction, ArgOutputFunctionError),
        "Wm": (WristMode, ArgToolModeError),
    }

    _ALLOWED_VALUES: dict[str, frozenset] = {
        key: _extract_allowed(source)
        for key, (source, _) in _VALIDATION_RULES.items()
    }

    @classmethod
    def literal(cls, literal_type: LiteralType, value: Any) -> None:
        allowed = cls._ALLOWED_VALUES.get(literal_type)
        if allowed is None:
            raise ValueError(f"Undefined literal type: {literal_type!r}")

        if value not in allowed:
            error_cls = cls._VALIDATION_RULES[literal_type][1]
            raise error_cls(str(value))

    @staticmethod
    def index(index: int, range_: Iterable[int], param_name: str = ""):
        if index not in range_:
            raise ArgValueError(
                str(index)
                + (f" for param name '{param_name}'" if param_name else "")
            )

    @staticmethod
    def value(
        value: float,
        gap: tuple[float | None, float | None],
        param_name: str = "",
    ):
        """
        Проверка числа на вхождение в промежуток.

        Args:
            value: Число для проверки.
            gap(tuple): Ограничения промежутка (промежуток включает границы).
                (None, int) — промежуток с ограничением по верхнему значению.
                (int, None) — промежуток с ограничением по нижнему значению.
                (int, int) — промежуток с ограничениями по верхнему и нижнему
                    значениям.

        Returns:
            None: В случае прохождения проверки.
        Raises:
            ArgValueError: В случае неуспешного прохождения проверки.
        """

        if gap[0] is None and gap[1] is not None:
            if value <= gap[1]:
                return
        elif gap[0] is not None and gap[1] is None:
            if gap[0] <= value:
                return
        elif gap[0] is not None and gap[1] is not None:
            if gap[0] <= value <= gap[1]:
                return
        raise ArgValueError(
            str(value)
            + f" value not in gap [{gap}]"
            + (f" for param name '{param_name}'" if param_name else "")
        )

    @staticmethod
    def length(
        obj: Sized,
        length: int,
        param_name: str = "",
    ):
        if len(obj) < length:
            raise ArrayLengthError(
                f"object length less then {length}"
                + (f" for param name '{param_name}'" if param_name else "")
            )

    @staticmethod
    def gui_entry(entry_text: str):
        return (
            True if re.match(ALLOWED_GUI_ENTRY_SYMBOLS, entry_text) else False
        )
