from __future__ import annotations

from typing import TYPE_CHECKING, Tuple, cast

from api.robot_api_rc5v15.API.source.core.connection_state import (
    ConnectionState,
    handle_connection,
)
from api.robot_api_rc5v15.API.source.models.classes.enum_classes.controller_commands import (
    ControllerCommands,
)

if TYPE_CHECKING:
    from api.robot_api_rc5v15.API.source.core.network import (
        Controller,
    )


class MoveScaling:
    """
    Класс для работы со скоростью и ускорением робота.
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
    def set(self, velocity: float = 1, acceleration: float = 1) -> bool:
        """
        Устанавливает глобальные множители скорости и ускорения для всех типов движения.

        Множители применяются как коэффициенты к текущим настройкам скорости
        и ускорения, ограничивая их до заданной доли от номинала.
        Это позволяет динамически регулировать динамику робота без изменения
        базовой конфигурации.

        Args:
            velocity: Множитель скорости (0.0 — 1.0).
            acceleration: Множитель ускорения (0.0 — 1.0).

        Returns:
            True: В случае успешной отправки команды.

        Examples:
            >>> # Замедлить робота до 30% скорости и ускорения на траекториях
            >>> robot.motion.scale_setup.set(velocity=0.3, acceleration=0.3)

            >>> # Полностью остановить (без отключения)
            >>> robot.motion.scale_setup.set(velocity=0.0, acceleration=0.0)

            >>> # Вернуть полную динамику
            >>> robot.motion.scale_setup.set()

        Notes:
            - Значения вне диапазона [0.0, 1.0] будут проигнорированы или вызовут ошибку.
            - Множители применяются **мгновенно** ко всем активным и будущим движениям.
        """

        return self._controller.send_command(
            ControllerCommands.CTRLR_COMS_SET_MOVE_SCALE,
            velocity,
            acceleration,
        )

    @handle_connection(available_in_read_only=False)
    def get(self) -> tuple[float, float] | None:
        """Возвращает текущие множители скорости и ускорения робота.

        Метод предоставляет актуальные значения коэффициентов, применяемых
        к динамическим параметрам движения. Эти множители влияют на все активные
        режимы: джоггинг, выполнение траекторий и т.д.

        Returns:
            Tuple[float, float] | None:
                - `(velocity_scale, acceleration_scale)` — пара значений
                  в диапазоне [0.0, 1.0], где:
                    * `velocity_scale` — текущий множитель скорости,
                    * `acceleration_scale` — текущий множитель ускорения;
                - `None`, если не удалось получить настройки.

        Examples:
            >>> scaling = robot.motion.scale_setup.get()
            >>> if scaling is not None:
            ...     vel_scale, acc_scale = scaling
            ...     print(f"Скорость: {vel_scale:.0%}, Ускорение: {acc_scale:.0%}")

        Notes:
            - Возвращаемые значения всегда нормированы в диапазон [0.0, 1.0].
            - Эти множители **не являются абсолютными скоростями**, а лишь
              коэффициентами, применяемыми к текущей конфигурации движения.
        """

        response = self._controller.request(
            ControllerCommands.CTRLR_COMS_GET_MOVE_SCALE
        )
        return cast(Tuple[float, float], response)
