from __future__ import annotations
from typing import TYPE_CHECKING
from struct import pack

from source.models.type_aliases import AngleUnits, PositionOrientation
from source.models.classes.enum_classes.controller_commands import (
    Getters as Get, Setters as Set
)

from source.core.exceptions.data_validation_error.argument_error import (
    validation
)
from source.models.constants import (
    CTRLR_SET_GET_TOOL_PACK_UNPACK_FORMAT, ORIENTATION_SLICE,
    POSITION_ORIENTATION_LENGTH, POSITION_SLICE
)
from source.models.classes.data_classes.command_templates import (
    MOTION_SETUP
)
import source.features.mathematics.unit_convert as math_s


if TYPE_CHECKING:
    from source.core.network.controller_socket import Controller


validate_literal = validation.validate_literal
validate_length = validation.validate_length


class Tool:
    """
    Класс для работы с инструментом робота.
    """

    _controller: Controller

    def __init__(self, controller: Controller) -> None:
        self._controller = controller

    def set(
        self,
        tool_end_point: PositionOrientation,
        units: AngleUnits = None
    ) -> bool:
        """
        Установить позицию конца инструмента.
        Args:
            tool_end_point: Позиция конца инструмента
                в формате (X, Y, Z, Rx, Ry, Rz), где (X, Y, Z) — м,
                (Rx, Ry,Rz) — 'units'.
            units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
        Returns:
            True: В случае успешной отправки команды.
        """

        if units is None:
            units = MOTION_SETUP.units
        validate_length(tool_end_point, POSITION_ORIENTATION_LENGTH)
        validate_literal('angle', units)
        if units == 'deg':
            tool_end_point = (
                list(tool_end_point[POSITION_SLICE])
                + math_s.degrees_to_radians(*tool_end_point[ORIENTATION_SLICE])
            )
        return self._controller.send(
            Set.ctrlr_coms_set_tool,
            pack(CTRLR_SET_GET_TOOL_PACK_UNPACK_FORMAT, *tool_end_point)
        )

    def get(
        self, units: AngleUnits = None
    ) -> PositionOrientation:
        """
        Получить позицию конца инструмента.

        Args:
            units: Единицы измерения. По-умолчанию градусы.
                'deg' — градусы.
                'rad' — радианы.
        Returns:
            list: Позиция конца инструмента робота
                в формате (X, Y, Z, Rx, Ry, Rz), где (X, Y, Z) — м,
                (Rx, Ry,Rz) — 'units'.
        """
        
        if units is None:
            units = MOTION_SETUP.units
        validate_literal('angle', units)
        self._controller.send(Get.ctrlr_coms_get_tool)
        response = list(
            self._controller.receive(
                Get.ctrlr_coms_get_tool, CTRLR_SET_GET_TOOL_PACK_UNPACK_FORMAT
            )
        )
        if units == 'deg':
            response = (
                response[POSITION_SLICE]
                + math_s.radians_to_degrees(*response[ORIENTATION_SLICE])
            )
        return response
