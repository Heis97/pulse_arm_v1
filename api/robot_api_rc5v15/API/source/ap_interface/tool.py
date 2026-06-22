from __future__ import annotations

from typing import TYPE_CHECKING

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.features.mathematics.unit_convert import (
    radians_to_degrees,
)
from api.robot_api_rc5v15.API.source.features.tools import (
    set_position_orientation_units,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.data_classes.motion_config import (
    MOTION_SETUP,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    ORIENTATION_SLICE,
    POSITION_ORIENTATION_LENGTH,
    POSITION_SLICE,
)
from api.robot_api_rc5v15.API.source.models.type_aliases import (
    AngleUnits,
    PositionOrientation,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller


class Tool:
    """
    Класс для работы с инструментом робота.
    """

    _controller: Controller

    def __init__(
        self,
        controller: Controller,
        connection_state: ConnectionState,
    ) -> None:
        self._controller = controller
        self._connection_state = connection_state

    @handle_connection(available_in_read_only=False)
    def set(
        self,
        tool_end_point: PositionOrientation,
        units: AngleUnits | None = None,
    ) -> bool:
        """Устанавливает положение и ориентацию конца инструмента (ЦТИ, TCP) относительно фланца робота.

        Метод задаёт смещение рабочей точки инструмента — например, кончика паяльника,
        сопла клеевого аппликатора или захвата манипулятора. Это позволяет роботу
        корректно управлять движением именно этой точки, а не центром фланца.

        Args:
            tool_end_point (PositionOrientation): Смещение TCP в формате
                `(X, Y, Z, Rx, Ry, Rz)`, где:
                - `X, Y, Z` — линейное смещение в метрах относительно фланца;
                - `Rx, Ry, Rz` — угловое смещение (вращение) вокруг осей X, Y, Z.
            units (Optional[AngleUnits]): Единицы измерения углов:
                - `'deg'` — градусы (используется по умолчанию, если `units=None`);
                - `'rad'` — радианы.

        Returns:
            bool: `True`, если команда успешно отправлена и TCP обновлён.

        Examples:
            >>> # Установить TCP: смещение 100 мм вперёд, без поворота (в градусах по умолчанию)
            >>> robot.tool.set((0.1, 0.0, 0.0, 0.0, 0.0, 0.0))

            >>> # Установить TCP с поворотом на 45 градусов вокруг Z
            >>> robot.tool.set((0.05, 0.0, 0.0, 0.0, 0.0, 45.0), 'deg')

            >>> # Установить TCP с углами в радианах
            >>> import math
            >>> robot.tool.set((0.0, 0.0, 0.05, 0.0, 0.0, math.pi / 4), 'rad')

        Notes:
            - Линейные компоненты (`X, Y, Z`) **всегда задаются в метрах**.
            - Угловые компоненты (`Rx, Ry, Rz`) интерпретируются в зависимости от `units`.
            - Если `units` не указан, углы считаются заданными в **градусах**.
            - После изменения TCP все последующие команды перемещения будут относиться
              именно к новой рабочей точке.
        """

        units = units or MOTION_SETUP.units
        Validation.length(
            tool_end_point, POSITION_ORIENTATION_LENGTH, "tool_end_point"
        )
        Validation.literal("angle", units)
        tool_end_point = set_position_orientation_units(tool_end_point, units)

        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_TOOL, tool_end_point
        )

    @handle_connection(available_in_read_only=False)
    def get(self, units: AngleUnits | None = None) -> PositionOrientation:
        """Получает текущее смещение конца инструмента (TCP) относительно фланца робота.

        Метод возвращает координаты рабочей точки инструмента, заданные ранее
        через `set()`. Эти данные определяют, какая точка в пространстве считается
        «концом» инструмента при планировании и выполнении траекторий.

        Args:
            units (Optional[AngleUnits]): Единицы измерения угловых компонент:
                - `'deg'` — градусы (по умолчанию, если `units=None`);
                - `'rad'` — радианы.

        Returns:
            PositionOrientation: Кортеж вида `(X, Y, Z, Rx, Ry, Rz)`, где:
                - `X, Y, Z` — линейное смещение в **метрах**;
                - `Rx, Ry, Rz` — угловое смещение вокруг соответствующих осей
                  в указанных единицах (`'deg'` или `'rad'`).

        Examples:
            >>> # Получить TCP в градусах (по умолчанию)
            >>> tcp = robot.tool.get()
            >>> x, y, z, rx, ry, rz = tcp
            >>> print(f"TCP: ({x:.3f}, {y:.3f}, {z:.3f}) м, углы: ({rx:.1f}, {ry:.1f}, {rz:.1f})°")

            >>> # Получить TCP с углами в радианах
            >>> tcp_rad = robot.tool.get('rad')
            >>> _, _, _, rx, ry, rz = tcp_rad
            >>> print(f"Углы в радианах: ({rx:.3f}, {ry:.3f}, {rz:.3f})")

        Notes:
            - Линейные компоненты (`X, Y, Z`) всегда возвращаются в **метрах**.
            - Угловые компоненты преобразуются в указанные единицы при возврате.
            - Если TCP не был задан явно, метод может вернуть нулевое смещение
              `(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)`.
        """

        units = units or MOTION_SETUP.units
        Validation.literal("angle", units)
        response = list(
            self._controller.request(ControllerCommands.CTRLR_COMS_GET_TOOL)
        )
        if units == "deg":
            response = response[POSITION_SLICE] + radians_to_degrees(
                response[ORIENTATION_SLICE]
            )
        return response
