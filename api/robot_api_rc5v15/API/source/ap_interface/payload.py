from __future__ import annotations

from contextlib import contextmanager
from typing import TYPE_CHECKING, Tuple, cast

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.features.validation import (
    Validation,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)
from api.robot_api_rc5v15.API.source.models.constants import (
    TCP_POSITION_COUNT,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import Controller


class PayLoad:
    """
    Класс для работы с полезной нагрузкой робота.
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
        mass: float,
        tcp_mass_center: tuple[float, float, float],
    ) -> bool:
        """Устанавливает массу и положение центра масс полезной нагрузки робота.

        Метод позволяет задать физические параметры устанавливаемого на фланец
        инструмента или груза. Эти данные используются системой управления для
        корректного расчёта моментов, компенсации инерции и обеспечения
        стабильного движения.

        Args:
            mass (float): Масса полезной нагрузки в килограммах.
            tcp_mass_center (tuple[float, float, float]): Координаты центра масс
                относительно системы координат фланца, в метрах. Формат: `(X, Y, Z)`, где:
                - `X` — смещение вдоль оси фланца (обычно вперёд/назад),
                - `Y` — смещение влево/вправо,
                - `Z` — смещение вверх/вниз.

        Returns:
            bool: `True`, если параметры успешно переданы в контроллер робота.

        Examples:
            >>> # Установить массу 2.5 кг с центром масс в центре фланца
            >>> robot.payload.set(2.5, (0.0, 0.0, 0.0))

            >>> # Установить длинный инструмент: масса 1.8 кг, центр масс смещён на 80 мм вперёд
            >>> robot.payload.set(1.8, (0.08, 0.0, 0.0))

            >>> # Лёгкий датчик с центром масс чуть вниз и вправо
            >>> robot.payload.set(0.3, (0.01, -0.02, -0.015))

        Notes:
            - Значения центра масс задаются **в метрах**, а не в миллиметрах.
            - Рекомендуется устанавливать эти параметры **до** запуска движения.
        """

        Validation.length(
            tcp_mass_center, TCP_POSITION_COUNT, "tcp_mass_center"
        )
        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_PAYLOAD, mass, tcp_mass_center
        )

    @handle_connection(available_in_read_only=False)
    def get(
        self,
    ) -> tuple[float, tuple[float, float, float]]:
        """Получает текущие параметры массы и центра масс полезной нагрузки робота.

        Метод возвращает ранее установленные (или используемые по умолчанию) значения
        массы и положения центра масс инструмента или груза, закреплённого на фланце.
        Эти данные используются системой управления для расчёта динамики и компенсации
        инерционных эффектов при движении.

        Returns:
            Tuple[float, Tuple[float, float, float]]: Кортеж вида `(масса, (X, Y, Z))`, где:
                - `масса` — масса полезной нагрузки в килограммах (float);
                - `(X, Y, Z)` — координаты центра масс в метрах относительно
                  системы координат фланца;

        Examples:
            >>> payload = robot.payload.get()
            ... mass, (x, y, z) = payload
            ... print(f"Масса: {mass} кг, ЦМ: ({x:.3f}, {y:.3f}, {z:.3f}) м")

            >>> # Проверить, совпадает ли центр масс с центром фланца
            >>> _, (x, y, z) = robot.payload.get()
            >>> if abs(x) < 0.001 and abs(y) < 0.001 and abs(z) < 0.001:
            ...     print("ЦМ в центре фланца")

        Notes:
            - Значения возвращаются в **метрах** и **килограммах**.
        """
        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_PAYLOAD
        )
        return cast(
            Tuple[float, Tuple[float, float, float]],
            (response[0], response[1:]),
        )

    @handle_connection(available_in_read_only=False)
    @contextmanager
    def using(
        self,
        mass: float | None = None,
        tcp_mass_center: tuple[float, float, float] | None = None,
    ):
        """
        Временно устанавливает новую параметры полезной нагрузки в рамках контекстного менеджера.

        Args:
            mass (float): Временная масса в кг.
            tcp_mass_center (Tuple[float, float, float]): Временный центр масс в метрах.

        Yields:
            None

        Examples:
            >>> # Временно изменить все параметры
            >>> with robot.payload.using(0.5, (0.0, 0.0, 0.05)):
            ...     print(robot.payload.get())
            ... # После выхода — параметры нагрузки будут восстановлены

            >>> # Временно изменить только массу
            >>> with robot.payload.using(mass = 0.5):
            ...     print(robot.payload.get())
            ... # После выхода — параметры нагрузки будут восстановлены
        """
        original_mass, original_center = self.get()
        if mass is None:
            mass = original_mass
        if tcp_mass_center is None:
            tcp_mass_center = original_center
        self.set(mass, tcp_mass_center)

        try:
            yield self
        finally:
            self.set(original_mass, original_center)

    @handle_connection(available_in_read_only=False)
    def get_max(self) -> float:
        """Получает максимально допустимую массу полезной нагрузки робота.

        Метод возвращает максимально допустимую массу полезной нагрузки робота
        согласно его паспортным данным.

        Returns:
            float: масса полезной нагрузки в килограммах (float).

        Examples:
            >>> max_payload = robot.payload.get_max()
            ... print(
            ...     f"Максимально допустимая масса полезной нагрузки: {max_payload} кг"
            ... )

        Notes:
            - Значение возвращается в **килограммах**.
        """

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_MAX_PAYLOAD
        )
        return response[0]
